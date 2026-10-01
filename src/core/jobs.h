// Minimal fork-join job system tuned for low-latency physics dispatch.
// The calling (main) thread always participates in the work; workers spin briefly
// after each batch before going to sleep, so back-to-back batches are cheap.
#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace bl {

class JobSystem {
public:
    using RangeFn = std::function<void(int begin, int end, int thread)>;

    static JobSystem& get();
    // Performance cores of the machine (Apple silicon: without the efficiency cores), else all hardware threads.
    static int performance_cores();

    void init(int num_workers);
    void shutdown();

    // Total number of threads that execute jobs (workers + caller).
    int num_threads() const { return (int)m_workers.size() + 1; }
    // 0 for the main thread, 1..N for workers.
    static int thread_index();
    // True while executing inside a parallel_for chunk.
    static bool in_job();

    // Calls fn(begin, end, thread) over [0, count) split into chunks of `grain` items.
    // Chunks are handed out dynamically (work sharing). Blocks until everything is done.
    // Nested calls from inside a job run serially on the calling thread.
    void parallel_for(int count, int grain, const RangeFn& fn);

    // Convenience: one item per chunk.
    void parallel_items(int count, const std::function<void(int item, int thread)>& fn) {
        parallel_for(count, 1, [&](int b, int e, int t) { for (int i = b; i < e; i++) fn(i, t); });
    }

    // Parallel phases inside one long job (a big simulation island), see Team. Idle threads (workers without a
    // batch item, the caller waiting for its batch) take chunks from the open boards.
    using ChunkFn = std::function<void(int chunk)>;
    struct Board {
        // claims: one fetch_add on (generation << 32 | next chunk); the phase's function and count are read from the
        // descriptor of that generation (two alternate, each tagged with its generation: a claim that raced with the
        // owner moving on sees the tag change and gives up)
        alignas(128) std::atomic<uint64_t> state{0};
        alignas(128) std::atomic<int> done{0};   // chunks finished (the owner waits for all)
        struct alignas(128) Desc {
            std::atomic<uint32_t> gen{0xffffffffu};
            std::atomic<const ChunkFn*> fn{nullptr};
            std::atomic<int> count{0};
        } desc[2];
        alignas(128) std::atomic<bool> in_use{false};
        uint32_t gen = 0;                        // (owner only)
    };
    static constexpr int kMaxBoards = 8;
    Board* open_board();
    void close_board(Board* b);
    // Runs one phase on a board: fn(chunk) for every chunk in [0, count), the caller working too. Returns when all
    // chunks are done (it never waits for a helper to arrive: no deadlock, whatever the number of free threads).
    void run_board(Board* b, int count, const ChunkFn& fn);
    // The same in two: post_board publishes the phase and returns at once (the helpers take its chunks while the caller
    // does something else; fn must live until wait_board), wait_board works on what is left and returns when all are done.
    void post_board(Board* b, int count, const ChunkFn& fn);
    void wait_board(Board* b, int count);
    // Phase ends the owner waited for helpers still in a chunk: total wait (us) and waits over 0.2 ms, since the last call.
    void take_board_waits(double& wait_ms, int& stalls) {
        wait_ms = m_wait_ms.exchange(0) * 1e-3;
        stalls = m_stalls.exchange(0);
    }
    // Takes and runs one chunk of an open board; false if there was none.
    bool help_boards();
    // (diagnostics: the chunks the owners ran themselves and the ones the helpers took, since the last call)
    void take_chunk_counts(long long& own, long long& helped) {
        own = m_own_chunks.exchange(0);
        helped = m_helped_chunks.exchange(0);
    }

private:
    void worker_main(int index);
    bool run_chunks(int thread);
    static bool claim(Board& b, int& chunk, const ChunkFn*& fn);
    Board m_boards[kMaxBoards];
    std::atomic<int> m_open_boards{0};
    std::atomic<uint64_t> m_wait_ms{0}; // (microseconds)
    alignas(128) std::atomic<uint32_t> m_post{0};   // bumped whenever there may be new work (parked helpers wait on it)
    alignas(128) std::atomic<int> m_parked{0};
    void wake_parked();
    std::atomic<int> m_stalls{0};
    std::atomic<long long> m_own_chunks{0}, m_helped_chunks{0};

    std::vector<std::thread> m_workers;
    std::mutex m_mutex;
    std::condition_variable m_cv;
    std::atomic<bool> m_quit{false};

    // current batch
    const RangeFn* m_fn = nullptr;
    int m_count = 0;
    int m_grain = 1;
    std::atomic<int> m_next{0};
    std::atomic<int> m_done{0};
    std::atomic<uint32_t> m_generation{0};
    std::atomic<int> m_active{0};
    std::atomic<bool> m_in_batch{false};
};

// A team for one long job: phases of chunks shared with the idle threads of the pool (Fratarcangeli-style
// "work sharing inside a task"). Without free threads, or single-threaded, the owner runs the chunks itself in order.
// Chunk functions must not wait for each other; their results must not depend on which thread ran them.
class Team {
public:
    explicit Team(bool parallel) : m_board(parallel ? JobSystem::get().open_board() : nullptr) {}
    ~Team() {
        wait();
        if (m_board) JobSystem::get().close_board(m_board);
    }
    Team(const Team&) = delete;
    Team& operator=(const Team&) = delete;
    bool parallel() const { return m_board != nullptr; }
    template <class F>
    void run(int count, F&& f) {
        if (count <= 0) return;
        if (!m_board || count == 1) {
            for (int i = 0; i < count; i++) f(i);
            return;
        }
        const JobSystem::ChunkFn fn = [&f](int c) { f(c); };
        JobSystem::get().run_board(m_board, count, fn);
    }
    // A phase posted, its chunks taken by the helpers while the caller goes on; wait() works on the rest and returns
    // when it is all done (the posted function kept here till then; without helpers it runs at wait)
    void post(int count, JobSystem::ChunkFn fn) {
        wait();
        if (count <= 0) return;
        m_posted = std::move(fn), m_posted_count = count;
        if (m_board) JobSystem::get().post_board(m_board, count, m_posted);
    }
    void wait() {
        if (m_posted_count <= 0) return;
        if (m_board) JobSystem::get().wait_board(m_board, m_posted_count);
        else
            for (int i = 0; i < m_posted_count; i++) m_posted(i);
        m_posted_count = 0;
        m_posted = nullptr;
    }

private:
    JobSystem::Board* m_board;
    JobSystem::ChunkFn m_posted;
    int m_posted_count = 0;
};

} // namespace bl
