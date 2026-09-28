#include "NanoProfiler.h"
#include <chrono>

namespace Perfmon
{
    int NanoProfiler::BeginSample(const char* name)
    {
        auto& samples = GetSamplesInternal();
        samples.emplace_back(Sample{ name, _top, GetTime(), 0.0 });
        _top = static_cast<int>(samples.size()) - 1;

        return _top;
    }

    void NanoProfiler::EndSample()
    {
        auto& samples = GetSamplesInternal();

        samples[_top].endTime = GetTime();
        _top = samples[_top].parent;  // Return to parent sample
    }

    void NanoProfiler::Clear()
    {
        // Move current samples to buffer for display
        GetBufferSamplesInternal() = GetSamplesInternal();
        GetSamplesInternal().clear();

        _top = -1;  // Reset call stack
    }

    const std::vector<NanoProfiler::Sample>& NanoProfiler::GetSamples()
    {
        return GetSamplesInternal();
    }

    std::vector<NanoProfiler::Sample>& NanoProfiler::GetSamplesInternal()
    {
        static std::vector<Sample> samples;
        return samples;
    }

    std::vector<NanoProfiler::Sample>& NanoProfiler::GetBufferSamplesInternal()
    {
        static std::vector<Sample> samples;
        return samples;
    }

    double NanoProfiler::GetTime()
    {
        // Convert nanoseconds to microseconds
        return std::chrono::high_resolution_clock::now().time_since_epoch().count() / 1000.0 / 1000.0;
    }

    const std::vector<NanoProfiler::Sample>& NanoProfiler::GetBufferSamples()
    {
        return GetBufferSamplesInternal();
    }

    int NanoProfiler::_top;

} // namespace Perfmon
