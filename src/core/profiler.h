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

uint64_t now();                 // raw ticks
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

class AccumScope {
public:
    explicit AccumScope(Zone* z) : m_zone(z), m_t0(now()) {}
    ~AccumScope() {
        ZoneSlot& s = m_zone->slot[t_zone_slot];
        s.ticks.store(s.ticks.load(std::memory_order_relaxed) + (now() - m_t0), std::memory_order_relaxed);
        s.calls.store(s.calls.load(std::memory_order_relaxed) + 1, std::memory_order_relaxed);
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
