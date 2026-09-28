#include "core/jobs.h"
#include "core/profiler.h"

#include <algorithm>
#include <cstdlib>
#include <chrono>
#if defined(__APPLE__)
#include <pthread.h>
#include <sys/qos.h>
#endif
#if defined(__APPLE__)
#include <sys/sysctl.h>
#endif

#if defined(__aarch64__) || defined(_M_ARM64)
#define BL_CPU_RELAX() asm volatile("yield" ::: "memory")
#elif defined(__x86_64__) || defined(_M_X64)
#include <immintrin.h>
#define BL_CPU_RELAX() _mm_pause()
#else
#define BL_CPU_RELAX() ((void)0)
#endif

namespace bl {

static thread_local int t_thread_index = 0;
static thread_local bool t_inside_job = false;

JobSystem& JobSystem::get() {
    static JobSystem js;
    return js;
}

int JobSystem::thread_index() { return t_thread_index; }

int JobSystem::performance_cores() {
#if defined(__APPLE__)
    int n = 0;
    size_t sz = sizeof(n);
    if (sysctlbyname("hw.perflevel0.physicalcpu", &n, &sz, nullptr, 0) == 0 && n > 0) return n;
#endif
    return (int)std::max(2u, std::thread::hardware_concurrency());
}
bool JobSystem::in_job() { return t_inside_job; }

void JobSystem::init(int num_workers) {
    shutdown();
    m_quit = false;
    for (int i = 0; i < num_workers; i++) m_workers.emplace_back([this, i] { worker_main(i + 1); });
}

void JobSystem::shutdown() {
    if (m_workers.empty()) return;
    {
        std::lock_guard<std::mutex> lk(m_mutex);
        m_quit = true;
        m_generation.fetch_add(1);
    }
    wake_parked();
    m_cv.notify_all();
    for (auto& t : m_workers) t.join();
    m_workers.clear();
}

bool JobSystem::run_chunks(int thread) {
    bool did = false;
    const RangeFn* fn = m_fn;
    const int count = m_count, grain = m_grain;
    for (;;) {
        int b = m_next.fetch_add(grain, std::memory_order_relaxed);
        if (b >= count) break;
        int e = std::min(count, b + grain);
        t_inside_job = true;
        (*fn)(b, e, thread);
        t_inside_job = false;
        m_done.fetch_add(e - b, std::memory_order_acq_rel);
        did = true;
    }
    return did;
}

void JobSystem::worker_main(int index) {
    t_thread_index = index;
    prof::register_thread(index);
#if defined(__APPLE__)
    // (the physics runs under the frame's deadline: the scheduler keeps such threads on the performance cores and
    // preempts them less; a helper preempted in the middle of a chunk stalls its whole island)
    pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE, 0);
#endif
    uint32_t seen = m_generation.load();
    for (;;) {
        // Spin for a short while waiting for the next batch (helping open boards meanwhile), then sleep.
        auto spin_start = std::chrono::steady_clock::now();
        int spins = 0;
        uint64_t idle_since = 0;
        while (m_generation.load(std::memory_order_acquire) == seen) {
            if (m_open_boards.load(std::memory_order_acquire) > 0) {
                if (help_boards()) {
                    spin_start = std::chrono::steady_clock::now();
                    spins = 0;
                    idle_since = 0;
                    continue;
                }
                // Nothing to take: spin for a while (an island's next phase usually follows within microseconds), then
                // park until something is posted. A helper spinning through an owner's long serial parts only takes
                // the core away from it (and from the rest of the system): the OS then preempts somebody in a chunk.
                const uint64_t now = prof::now();
                if (!idle_since) idle_since = now;
                if (prof::ticks_to_ms(now - idle_since) < 0.1) {
                    for (int k = 0; k < 16; k++) BL_CPU_RELAX();
                    continue;
                }
                const uint32_t post = m_post.load(std::memory_order_acquire);
                if (help_boards()) {
                    idle_since = 0;
                    continue;
                }
                if (m_open_boards.load(std::memory_order_acquire) > 0 && m_generation.load(std::memory_order_acquire) == seen) {
                    m_parked.fetch_add(1, std::memory_order_acq_rel);
                    m_post.wait(post, std::memory_order_acquire);
                    m_parked.fetch_sub(1, std::memory_order_acq_rel);
                }
                idle_since = 0;
                spin_start = std::chrono::steady_clock::now();
                continue;
            }
            BL_CPU_RELAX();
            if (++spins > 256) {
                spins = 0;
                if (std::chrono::steady_clock::now() - spin_start > std::chrono::microseconds(300)) {
                    std::unique_lock<std::mutex> lk(m_mutex);
                    m_cv.wait(lk, [&] { return m_generation.load() != seen || m_open_boards.load() > 0 || m_quit; });
                    spin_start = std::chrono::steady_clock::now();
                    if (m_generation.load() == seen && !m_quit) continue; // (woken for a board)
                    break;
                }
            }
        }
        seen = m_generation.load(std::memory_order_acquire);
        if (m_quit) return;
        m_active.fetch_add(1, std::memory_order_acq_rel);
        if (m_in_batch) run_chunks(index);
        m_active.fetch_sub(1, std::memory_order_acq_rel);
    }
}

JobSystem::Board* JobSystem::open_board() {
    if (m_workers.empty()) return nullptr;
    for (Board& b : m_boards) {
        bool expected = false;
        if (b.in_use.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
            if (m_open_boards.fetch_add(1, std::memory_order_acq_rel) == 0) {
                std::lock_guard<std::mutex> lk(m_mutex); // (wake sleeping workers)
            }
            m_cv.notify_all();
            return &b;
        }
    }
    return nullptr;
}

void JobSystem::close_board(Board* b) {
    b->in_use.store(false, std::memory_order_release);
    m_open_boards.fetch_sub(1, std::memory_order_acq_rel);
    wake_parked();
}

void JobSystem::wake_parked() {
    m_post.fetch_add(1, std::memory_order_acq_rel);
    if (m_parked.load(std::memory_order_acquire) > 0) m_post.notify_all();
}

// A chunk of the board's current phase: one fetch_add on the claim word (no retry loop), then the descriptor of the
// generation it hit. The descriptor was complete before that generation was published; if it has been reused since
// (tag changed), that generation is over and the index was past its end anyway.
bool JobSystem::claim(Board& b, int& chunk, const ChunkFn*& fn) {
    const uint64_t s = b.state.load(std::memory_order_acquire);
    {
        const Board::Desc& d = b.desc[(s >> 32) & 1];
        if (d.gen.load(std::memory_order_acquire) != (uint32_t)(s >> 32) || (uint32_t)s >= (uint32_t)d.count.load(std::memory_order_acquire))
            return false; // (nothing left: no write to the shared line)
    }
    const uint64_t r = b.state.fetch_add(1, std::memory_order_acq_rel);
    const uint32_t g = (uint32_t)(r >> 32), idx = (uint32_t)r;
    const Board::Desc& d = b.desc[g & 1];
    const uint32_t g1 = d.gen.load(std::memory_order_acquire);
    const int count = d.count.load(std::memory_order_acquire);
    const ChunkFn* f = d.fn.load(std::memory_order_acquire);
    const uint32_t g2 = d.gen.load(std::memory_order_acquire);
    if (g1 != g || g2 != g || idx >= (uint32_t)count || !f) return false;
    chunk = (int)idx;
    fn = f;
    return true;
}

bool JobSystem::help_boards() {
    for (Board& b : m_boards) {
        if (!b.in_use.load(std::memory_order_acquire)) continue;
        int chunk;
        const ChunkFn* fn;
        if (!claim(b, chunk, fn)) continue;
        const bool was = t_inside_job;
        t_inside_job = true;
        (*fn)(chunk);
        t_inside_job = was;
        b.done.fetch_add(1, std::memory_order_acq_rel);
        return true;
    }
    return false;
}

void JobSystem::run_board(Board* b, int count, const ChunkFn& fn) {
    // publish: the descriptor of the new generation (tag last), the counters, then the claim word (release)
    const uint32_t g = ++b->gen;
    Board::Desc& d = b->desc[g & 1];
    d.gen.store(0xffffffffu, std::memory_order_seq_cst);
    d.fn.store(&fn, std::memory_order_seq_cst);
    d.count.store(count, std::memory_order_seq_cst);
    d.gen.store(g, std::memory_order_seq_cst);
    b->done.store(0, std::memory_order_seq_cst);
    b->state.store((uint64_t)g << 32, std::memory_order_seq_cst);
    if (count > 1) wake_parked();
    const bool was = t_inside_job;
    t_inside_job = true;
    int chunk, done = 0;
    const ChunkFn* f;
    while (claim(*b, chunk, f)) {
        (*f)(chunk);
        done++;
    }
    t_inside_job = was;
    if (done) b->done.fetch_add(done, std::memory_order_acq_rel);
    if (b->done.load(std::memory_order_acquire) < count) {
        const uint64_t w0 = prof::now();
        while (b->done.load(std::memory_order_acquire) < count) BL_CPU_RELAX();
        const double ms = prof::ticks_to_ms(prof::now() - w0);
        m_wait_ms.fetch_add((uint64_t)(ms * 1000.0), std::memory_order_relaxed);
        if (ms > 0.2) m_stalls.fetch_add(1, std::memory_order_relaxed);
    }
}

void JobSystem::parallel_for(int count, int grain, const RangeFn& fn) {
    if (count <= 0) return;
    grain = std::max(1, grain);
    if (m_workers.empty() || t_inside_job || count <= grain) {
        fn(0, count, t_thread_index);
        return;
    }
    m_fn = &fn;
    m_count = count;
    m_grain = grain;
    m_next.store(0, std::memory_order_relaxed);
    m_done.store(0, std::memory_order_relaxed);
    m_in_batch = true;
    {
        std::lock_guard<std::mutex> lk(m_mutex);
        m_generation.fetch_add(1, std::memory_order_acq_rel);
    }
    m_cv.notify_all();
    wake_parked();
    run_chunks(t_thread_index);
    while (m_done.load(std::memory_order_acquire) < count)
        if (!(m_open_boards.load(std::memory_order_acquire) > 0 && help_boards())) BL_CPU_RELAX();
    m_in_batch = false;
    // Wait until no worker is still inside run_chunks() reading batch state.
    while (m_active.load(std::memory_order_acquire) != 0) BL_CPU_RELAX();
    m_fn = nullptr;
}

} // namespace bl
