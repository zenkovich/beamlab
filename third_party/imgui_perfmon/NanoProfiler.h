#pragma once

#include <vector>

namespace Perfmon
{
    // Lightweight hierarchical profiler for measuring execution time
    class NanoProfiler
    {
    public:
        // Single profiling sample with timing information
        struct Sample
        {
            const char* name = nullptr; // Sample name
            int parent = -1;            // Parent sample index
            double beginTime = 0.0;     // Start time in microseconds
            double endTime = 0.0;       // End time in microseconds
        };

        // RAII helper for automatic sample scoping
        struct SampleScope
        {
            SampleScope(const char* name) { BeginSample(name); }
            ~SampleScope() { EndSample(); }
        };

    public:
        // Start timing a named sample, returns sample index
        static int BeginSample(const char* name);

        // Stop timing current sample
        static void EndSample();

        // (BeamLab) A child of the frame's last sample named parentName (the current one if there is none) of a known
        // duration (ms): time measured elsewhere, summed over many short intervals, shown as a part of its parent
        static void AddSample(const char* parentName, const char* name, double durationMs);

        // Move current samples to buffer and clear current list
        static void Clear();

        // Get current frame samples (being recorded)
        static const std::vector<Sample>& GetSamples();

        // Get previous frame samples (completed)
        static const std::vector<Sample>& GetBufferSamples();

    private:
        // Internal access to current samples
        static std::vector<Sample>& GetSamplesInternal();

        // Internal access to buffer samples
        static std::vector<Sample>& GetBufferSamplesInternal();

        // Get current time in microseconds
        static double GetTime();

    private:
        static int _top;  // Current top of call stack
    };
} // namespace Perfmon

// Convenience macros for profiling
#define NANO_PROFILE_BEGIN(NAME) Perfmon::NanoProfiler::BeginSample(NAME)
#define NANO_PROFILE_END() Perfmon::NanoProfiler::EndSample()
#define NANO_PROFILE_SCOPE(NAME) Perfmon::NanoProfiler::SampleScope _nano_prof_scope(NAME)

