// Lightweight multi-threaded instrumentation profiler.
//
//  * PROFILE_ZONE(name)       - timed scope: accumulates per-frame time for the zone, records a
//                               timeline event for the thread, and - when executed on the main
//                               thread outside of jobs - also feeds imgui_perfmon's NanoProfiler.
//  * PROFILE_ACCUM(name)      - cheap accumulating-only scope for hot code (per substep etc).
//
// prof::end_frame() snapshots everything (call on the main thread when workers are idle).
#pragma once

#include <atomic>
#include <cstdint>
#include <vector>

namespace bl::prof {

// raw ticks (Apple's arm64: the system counter read in place - mach_absolute_time's barrier in every zone of the
// hot loops' small tasks was a twentieth of a crash's physics)
#if defined(__APPLE__) && defined(__aarch64__)
inline uint64_t now() { return __builtin_arm_rsr64("CNTVCT_EL0"); }
#else
uint64_t now();
#endif
double ticks_to_ms(uint64_t t); // conversion

// Accumulators of one thread (PROFILE_ACCUM in parallel phases: no shared cache line, no atomic read-modify-write).
struct alignas(64) ZoneSlot {
    std::atomic<uint64_t> ticks{0};
    std::atomic<uint32_t> calls{0};
};
constexpr int kZoneSlots = 32;
inline thread_local int t_zone_slot = 0; // set by register_thread

struct Zone {
    const char* name = nullptr;
    int id = 0;
    std::atomic<uint64_t> ticks{0};
    std::atomic<uint32_t> calls{0};
    ZoneSlot slot[kZoneSlots];
    // snapshot of the last finished frame
    double last_ms = 0.0;
    uint32_t last_calls = 0;
    double avg_ms = 0.0; // exponential moving average
};

struct Event {
    const Zone* zone;
    uint64_t t0, t1;
    int depth;
};

struct ThreadTimeline {
    int thread = 0;
    std::vector<Event> events;
};

Zone* zone(const char* name);
void register_thread(int index);
void end_frame();
void set_timeline_enabled(bool e);
bool timeline_enabled();

const std::vector<Zone*>& zones();
Zone* find_zone(const char* name);
// Timeline of the last completed frame (one entry per thread that recorded events).
const std::vector<ThreadTimeline>& last_timeline();
uint64_t last_frame_begin();
uint64_t last_frame_end();

class Scope {
public:
    explicit Scope(Zone* z);
    ~Scope();

private:
    Zone* m_zone;
    uint64_t m_t0;
    int m_event = -1;
    bool m_nano = false;
};

// (diagnostics: every PROFILE_ACCUM scope's interval on its thread while armed - World arms it for the substeps
// BL_TRACE names; trace_dump writes them as JSON)
extern std::atomic<bool> g_trace;
void trace_event(const Zone* z, uint64_t t0, uint64_t t1);
void trace_mark(const char* what, int value); // (a marker in the trace: a substep's start, its short step)
void trace_dump(const char* path);

class AccumScope {
public:
    explicit AccumScope(Zone* z) : m_zone(z), m_t0(now()) {}
    ~AccumScope() {
        const uint64_t t1 = now();
        ZoneSlot& s = m_zone->slot[t_zone_slot];
        s.ticks.store(s.ticks.load(std::memory_order_relaxed) + (t1 - m_t0), std::memory_order_relaxed);
        s.calls.store(s.calls.load(std::memory_order_relaxed) + 1, std::memory_order_relaxed);
        if (g_trace.load(std::memory_order_relaxed)) trace_event(m_zone, m_t0, t1);
    }

private:
    Zone* m_zone;
    uint64_t m_t0;
};

} // namespace bl::prof

#define BL_PCAT2(a, b) a##b
#define BL_PCAT(a, b) BL_PCAT2(a, b)
#define PROFILE_ZONE(NAME)                                                        \
    static ::bl::prof::Zone* BL_PCAT(_pz_, __LINE__) = ::bl::prof::zone(NAME); \
    ::bl::prof::Scope BL_PCAT(_ps_, __LINE__)(BL_PCAT(_pz_, __LINE__))
#define PROFILE_ACCUM(NAME)                                                       \
    static ::bl::prof::Zone* BL_PCAT(_pz_, __LINE__) = ::bl::prof::zone(NAME); \
    ::bl::prof::AccumScope BL_PCAT(_ps_, __LINE__)(BL_PCAT(_pz_, __LINE__))
