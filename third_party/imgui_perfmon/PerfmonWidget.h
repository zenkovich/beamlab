#pragma once

#include "TimeSeries.h"
#include "NanoProfiler.h"
#include "imgui.h"

#include <string>
#include <unordered_map>
#include <functional>

namespace Perfmon
{
    // Performance status levels
    enum class PerfStatus { Good, Normal, Bad };

    // Common settings for performance metrics
    struct PerfMetricSettings
    {
        double goodValue = 0.0;           // Threshold for good performance
        double badValue = 0.0;            // Threshold for bad performance
        float badValueWeight = 1.0f;      // Weight for bad status in overall calculation
        float normalValueWeight = 1.0f;   // Weight for normal status in overall calculation
        float goodValueWeight = 1.0f;     // Weight for good status in overall calculation
    };

    // Continuously sampled performance metric with time series data
    struct Metric : public PerfMetricSettings
    {
        std::string name;                         // Metric display name
        std::function<double()> getSampleFunc;    // Function to get current value
        double baselineValue = 0.0;               // Baseline value for comparison

    public:
        static constexpr int kNumSamples = 130;   // Number of samples in time series

    public:
        Metric() = default;
        Metric(const std::string& name, std::function<double()> getSampleFunc, PerfMetricSettings perfSettings, std::vector<double> targetValues);

        // Get current performance status based on median value
        PerfStatus GetStatus() const;

        // Update metric with new sample if needed
        void Update(float dt);

        // Get time series data for this metric
        const TimeSeries<double>& GetSamplesData() const { return _samplesData; }

        // Get current adaptive target value
        double GetTargetValue() const { return _targetValue; }

    private:
        float _timeSinceLastUpdate = 0.0f;        // Time since last sample
        float _updateInterval = 0.0f;             // Update interval in seconds
        double _targetValue = 0.0;                // Current adaptive target value
        std::vector<double> _targetValues;        // Possible target values
        TimeSeries<double> _samplesData = TimeSeries<double>(kNumSamples);  // Time series data
    };

    // Simple counter-based metric (entities, objects, etc.)
    struct EntityCountMetric : public PerfMetricSettings
    {
        std::string name;                     // Metric display name
        int count = 0;                        // Current count
        int baselineCount = 0;                // Baseline count for comparison
        std::function<int()> getEntityCount; // Function to get current count

    public:
        EntityCountMetric() = default;
        EntityCountMetric(const std::string& name, std::function<int()> getEntityCount, PerfMetricSettings perfSettings);
    };

    // Main performance monitoring widget for ImGui
    class PerfmonWidget
    {
    public:
        // Draw complete performance monitoring UI
        void DrawGUI();

        // Update all metrics and profiler data
        void Update(float dt);

        // Register a time-series performance metric
        void RegisterMetric(Metric metric);

        // Register a counter-based metric
        void RegisterCounterMetric(EntityCountMetric metric);

    private:
        static constexpr int kNanoProfilerNumSamples = 140;  // Number of profiler frames to keep

    private:
        // Internal profiler sample data
        struct ProfileSample
        {
            const char* name;      // Sample name
            size_t hash = 0;       // Hash for grouping
            double time = 0.0;     // Duration in milliseconds
            int parent = -1;       // Parent sample index
        };

        // Single frame of profiler data
        struct NanoProfileFrame
        {
            std::vector<ProfileSample> samples;  // Samples in this frame
            double totalTime = 0.0;              // Total frame time
        };

    private:
        // Current overall performance status
        PerfStatus _overallStatus = PerfStatus::Good;

        // Registered metrics
        std::vector<Metric> _metrics;
        std::vector<EntityCountMetric> _entityCounters;

        // Entity counter update timing
        float _timeSinceLastEntitiesCount = 0.0f;
        float _entitiesCountUpdateInterval = 0.3f;

        // Profiler visualization data
        std::vector<NanoProfileFrame> _nanoProfilerSamplesHistory;
        int _nanoProfilerDetailFrame = -1;  // Frame to show details for (-1 = none)
        std::unordered_map<const char*, ImColor> _nanoProfilerColors;  // Colors for samples
        int _lastNanoProfilerColorIndex = 0;

        // Baseline comparison mode
        bool _enableBaseline = false;

    private:
        // Draw overall performance status
        void DrawOverallStatus();

        // Draw profiler timeline visualization
        void DrawNanoProfiler();

        // Draw entity counters section
        void DrawEntitiesCounters();

        // Draw single performance metric
        void DrawMetric(Metric& metric);

        // Draw metric's current value with status color
        void DrawMetricRecentValue(Metric& metric);

        // Draw time series graph for metric
        void DrawGraphic(const TimeSeries<double>& data, double goodMetric, double badMetric, double targetMetric);

        // Draw single entity counter
        void DrawEntityCount(EntityCountMetric& metric);

        // Process profiler data into visualization format
        void GetNanoProfilerSamplesFrame();

        // Update entity counters periodically
        void UpdateEntitiesCount(float dt);

        // Count all registered entities
        void CountEntities();

        // Calculate overall performance status from all metrics
        PerfStatus GetOverallStatus() const;

        // Toggle baseline comparison mode
        void ToggleBaseline();
    };
} // namespace Perfmon
