#include "core/profiler.h"
#include "core/jobs.h"

#include "NanoProfiler.h"

#include <cstring>
#include <mutex>

#if defined(__APPLE__)
#include <mach/mach_time.h>
#else
#include <chrono>
#endif

namespace bl::prof {

namespace {
struct ThreadData {
    int index = 0;
    std::vector<Event> events;
    int depth = 0;
};

std::mutex g_mutex;
std::vector<Zone*> g_zones;
ThreadData g_threads[64];
std::vector<ThreadTimeline> g_last_timeline;
uint64_t g_frame_begin = 0, g_last_begin = 0, g_last_end = 0;
bool g_timeline = true;
thread_local ThreadData* t_data = &g_threads[0];

double g_tick_ms = 1e-6;
struct TickInit {
    TickInit() {
#if defined(__APPLE__)
        mach_timebase_info_data_t tb;
        mach_timebase_info(&tb);
        g_tick_ms = (double)tb.numer / (double)tb.denom * 1e-6;
#endif
        g_frame_begin = now();
    }
} g_tick_init;
} // namespace

uint64_t now() {
#if defined(__APPLE__)
    return mach_absolute_time();
#else
    return (uint64_t)std::chrono::steady_clock::now().time_since_epoch().count();
#endif
}

double ticks_to_ms(uint64_t t) { return (double)t * g_tick_ms; }

Zone* zone(const char* name) {
    std::lock_guard<std::mutex> lk(g_mutex);
    for (Zone* z : g_zones)
        if (std::strcmp(z->name, name) == 0) return z;
    Zone* z = new Zone();
    z->name = name;
    z->id = (int)g_zones.size();
    g_zones.push_back(z);
    return z;
}

Zone* find_zone(const char* name) {
    std::lock_guard<std::mutex> lk(g_mutex);
    for (Zone* z : g_zones)
        if (std::strcmp(z->name, name) == 0) return z;
    return nullptr;
}

const std::vector<Zone*>& zones() { return g_zones; }

void register_thread(int index) {
    if (index < 0 || index >= 64) return;
    t_data = &g_threads[index];
    t_data->index = index;
    t_zone_slot = index % kZoneSlots;
}

void set_timeline_enabled(bool e) { g_timeline = e; }
bool timeline_enabled() { return g_timeline; }

Scope::Scope(Zone* z) : m_zone(z) {
    m_nano = JobSystem::thread_index() == 0 && !JobSystem::in_job();
    if (m_nano) Perfmon::NanoProfiler::BeginSample(z->name);
    m_t0 = now();
    if (g_timeline) {
        ThreadData* td = t_data;
        m_event = (int)td->events.size();
        td->events.push_back({z, m_t0, m_t0, td->depth});
        td->depth++;
    }
}

Scope::~Scope() {
    uint64_t t1 = now();
    m_zone->ticks.fetch_add(t1 - m_t0, std::memory_order_relaxed);
    m_zone->calls.fetch_add(1, std::memory_order_relaxed);
    if (m_event >= 0) {
        ThreadData* td = t_data;
        if (m_event < (int)td->events.size()) td->events[m_event].t1 = t1;
        td->depth--;
    }
    if (m_nano) Perfmon::NanoProfiler::EndSample();
}

void end_frame() {
    uint64_t t = now();
    {
        std::lock_guard<std::mutex> lk(g_mutex);
        for (Zone* z : g_zones) {
            uint64_t ticks = z->ticks.exchange(0, std::memory_order_relaxed);
            uint32_t calls = z->calls.exchange(0, std::memory_order_relaxed);
            for (ZoneSlot& sl : z->slot) {
                ticks += sl.ticks.exchange(0, std::memory_order_relaxed);
                calls += sl.calls.exchange(0, std::memory_order_relaxed);
            }
            z->last_calls = calls;
            z->last_ms = ticks_to_ms(ticks);
            z->avg_ms = z->avg_ms * 0.95 + z->last_ms * 0.05;
        }
    }
    g_last_timeline.clear();
    for (auto& td : g_threads) {
        if (td.events.empty()) continue;
        ThreadTimeline tl;
        tl.thread = td.index;
        tl.events.swap(td.events);
        td.events.clear();
        td.events.reserve(tl.events.size());
        td.depth = 0;
        g_last_timeline.push_back(std::move(tl));
    }
    g_last_begin = g_frame_begin;
    g_last_end = t;
    g_frame_begin = t;
    Perfmon::NanoProfiler::Clear();
}

const std::vector<ThreadTimeline>& last_timeline() { return g_last_timeline; }
uint64_t last_frame_begin() { return g_last_begin; }
uint64_t last_frame_end() { return g_last_end; }

} // namespace bl::prof
