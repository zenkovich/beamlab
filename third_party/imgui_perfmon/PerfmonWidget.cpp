#include "PerfmonWidget.h"

#include "imgui_internal.h"
#include <algorithm>

namespace Perfmon
{
    constexpr float kGraphHeight = 10.f;

    // ImVec2 operators for convenience
    ImVec2 operator+(const ImVec2& lhs, const ImVec2& rhs) noexcept { return ImVec2(lhs.x + rhs.x, lhs.y + rhs.y); }
    ImVec2 operator-(const ImVec2& lhs, const ImVec2& rhs) noexcept { return ImVec2(lhs.x - rhs.x, lhs.y - rhs.y); }

    namespace Detail
    {
        // Multiply alpha channel of color by factor
        ImColor MultiplyA(const ImColor& clr, float k) noexcept
        {
            ImColor r = clr;
            r.Value.w *= k;
            return r;
        }

        struct GraphParams
        {
            double min;
            double max;
            double target;
        };

        // Draw horizontal grid lines for graph background
        void DrawGrid(float width, float height, double gridStep, double min, double max)
        {
            if (gridStep <= 0)
                return;

            const int kMaxGridSteps = (int)height / 10;

            if ((max - min) / gridStep > (double)kMaxGridSteps)
                return;

            const auto first = (double)gridStep * (std::floor(min / (double)gridStep) + 1.0);
            const auto scale = (double)height / (max - min);

            ImGuiWindow* wnd = ImGui::GetCurrentWindow();
            ImVec2 cpos = wnd->DC.CursorPos;

            for (auto v = first; v < max; v += (double)gridStep)
            {
                const auto y = (v - min) * scale;
                wnd->DrawList->AddRectFilled(
                    cpos + ImVec2(0.0f, (float)y),
                    cpos + ImVec2((float)width, (float)(y + 1.0)),
                    ImColor(255, 255, 255, 64),
                    0.f);
            }
        }

        // Determine performance status based on metric value and thresholds
        PerfStatus GetMetricStatus(double metric, double bad, double good)
        {
            bool lessIsBetter = bad > good;

            if (lessIsBetter)
            {
                if (metric >= bad)
                    return PerfStatus::Bad;
                else if (metric <= good)
                    return PerfStatus::Good;
                else
                    return PerfStatus::Normal;
            }
            else
            {
                if (metric <= bad)
                    return PerfStatus::Bad;
                else if (metric >= good)
                    return PerfStatus::Good;
                else
                    return PerfStatus::Normal;
            }
        }

        // Get color for performance status
        ImColor GetStatusColor(PerfStatus status)
        {
            auto goodColor = ImColor(0.0f, 1.0f, 0.0f, 1.0f);
            auto normalColor = ImColor(1.0f, 1.0f, 0.0f, 1.0f);
            auto badColor = ImColor(1.0f, 0.0f, 0.0f, 1.0f);

            switch (status)
            {
            case PerfStatus::Good:
                return goodColor;
            case PerfStatus::Normal:
                return normalColor;
            case PerfStatus::Bad:
                return badColor;
            default:
                return ImColor(1.0f, 1.0f, 1.0f, 1.0f);
            }
        }

        float lerp(float a, float b, float t) noexcept
        {
            return a + (b - a) * t;
        }

        float clamp(float x, float min, float max) noexcept
        {
            if (x < min)
                return min;

            if (x > max)
                return max;

            return x;
        }
    } // namespace Detail

    EntityCountMetric::EntityCountMetric(const std::string& name, std::function<int()> getEntityCount, PerfMetricSettings perfSettings)
        : PerfMetricSettings(perfSettings), name(name), getEntityCount(getEntityCount)
    {
    }

    void PerfmonWidget::DrawGUI()
    {
        DrawOverallStatus();
        DrawNanoProfiler();

        for (auto& metric : _metrics)
            DrawMetric(metric);

        DrawEntitiesCounters();
    }

    // Draw time series graph with colored bars
    void PerfmonWidget::DrawGraphic(const TimeSeries<double>& data, double goodMetric, double badMetric, double targetMetric)
    {
        ImGuiWindow* wnd = ImGui::GetCurrentWindow();
        ImVec2 cpos = wnd->DC.CursorPos;

        float valueWidth = 1.0f;
        float valuesGap = 1.0f;

        auto scale = (double)kGraphHeight / targetMetric;
        auto head = data.GetHead();
        auto* samplesPtr = data.GetSamples();
        auto samplesCount = data.GetNumSamples();

        // Draw each sample as a colored bar
        for (size_t i = 0; i < samplesCount; ++i)
        {
            auto sample = samplesPtr[(i + head) % samplesCount];
            auto value = sample * scale;
            auto capValue = std::min((double)kGraphHeight, value);

            auto valueColor = Detail::GetStatusColor(Detail::GetMetricStatus(sample, badMetric, goodMetric));

            const auto x = (float)i * (valueWidth + valuesGap);
            wnd->DrawList->AddRectFilled(
                cpos + ImVec2((float)x, kGraphHeight - (float)capValue),
                cpos + ImVec2((float)(x + valueWidth), kGraphHeight),
                valueColor,
                0.f);
        }

        const ImVec2 graphSize((float)Metric::kNumSamples * (valueWidth + valuesGap), kGraphHeight);
        const ImRect graphBB(wnd->DC.CursorPos, wnd->DC.CursorPos + graphSize);
        ImGui::ItemSize(graphSize);
        ImGui::ItemAdd(graphBB, 0);
    }

    // Draw single performance metric with value and graph
    void PerfmonWidget::DrawMetric(Metric& metric)
    {
        DrawMetricRecentValue(metric);

        ImGuiStyle& style = ImGui::GetStyle();
        auto textColor = style.Colors[ImGuiCol_Text];
        auto prcTextColor = Detail::MultiplyA(textColor, .5f);

        // Draw percentile info on the right
        ImGui::SetWindowFontScale(0.9f);
        ImGui::SameLine();
        std::string prcData = "low:" + std::to_string((int)metric.GetSamplesData().P0()) + " max:" + std::to_string((int)metric.GetSamplesData().P100());

        const auto& textSize = ImGui::CalcTextSize(prcData.c_str());
        const auto& pos = ImGui::GetCursorPos();

        ImGui::SetCursorPos(ImVec2(ImGui::GetCursorPos().x + ImGui::GetContentRegionAvail().x - textSize.x, pos.y));
        ImGui::TextColored(prcTextColor, "%s", prcData.c_str());

        ImGui::SetWindowFontScale(1.0f);

        DrawGraphic(metric.GetSamplesData(), metric.goodValue, metric.badValue, metric.GetTargetValue());
    }

    void PerfmonWidget::Update(float dt)
    {
        for (auto& metric : _metrics)
            metric.Update(dt);

        UpdateEntitiesCount(dt);
        GetNanoProfilerSamplesFrame();

        _overallStatus = GetOverallStatus();
    }


    void PerfmonWidget::RegisterMetric(Metric metric)
    {
        _metrics.emplace_back(std::move(metric));
    }

    // Count entities using registered functions
    void PerfmonWidget::CountEntities()
    {
        for (auto& metric : _entityCounters)
        {
            if (metric.getEntityCount)
                metric.count = metric.getEntityCount();
        }
    }

    // Update entity counters periodically
    void PerfmonWidget::UpdateEntitiesCount(float dt)
    {
        _timeSinceLastEntitiesCount += dt;
        if (_timeSinceLastEntitiesCount > _entitiesCountUpdateInterval)
        {
            _timeSinceLastEntitiesCount = 0.0f;
            CountEntities();
        }
    }

    // Draw single entity counter with baseline comparison
    void PerfmonWidget::DrawEntityCount(EntityCountMetric& metric)
    {
        auto status = Detail::GetMetricStatus(metric.count, metric.badValue, metric.goodValue);
        ImGui::Text("%s", metric.name.c_str());
        ImGui::SameLine();
        ImGui::TextColored(Detail::GetStatusColor(status), "%i", metric.count);

        if (_enableBaseline)
        {
            auto deltaValue = metric.count - metric.baselineCount;

            ImGui::SetWindowFontScale(0.8f);

            ImGui::SameLine();
            ImGui::Text(deltaValue > 0 ? "(+%i)" : "(%i)", deltaValue);

            ImGui::SetWindowFontScale(1.0f);
        }
    }

    void PerfmonWidget::RegisterCounterMetric(EntityCountMetric metric)
    {
        _entityCounters.emplace_back(metric);
    }

    // Calculate overall performance status as weighted average
    PerfStatus PerfmonWidget::GetOverallStatus() const
    {
        float pointsSumm = 0.0f;
        float weightsSumm = 0.0f;

        auto addMetricWeight = [&](PerfStatus status, float badWeight, float normalWeight, float goodWeight) {
            float weight = status == PerfStatus::Bad ? badWeight : status == PerfStatus::Normal ? normalWeight :
                goodWeight;
            pointsSumm += (float)status * weight;
            weightsSumm += weight;
            };

        auto addMetric = [&](const Metric& metric) {
            addMetricWeight(metric.GetStatus(), metric.badValueWeight, metric.normalValueWeight, metric.goodValueWeight);
            };

        auto addCounterMetric = [&](const EntityCountMetric& metric) {
            addMetricWeight(Detail::GetMetricStatus(metric.count, metric.badValue, metric.goodValue), metric.badValueWeight, metric.normalValueWeight, metric.goodValueWeight);
            };

        for (auto& metric : _metrics)
            addMetric(metric);

        for (auto& metric : _entityCounters)
            addCounterMetric(metric);

        float overallStatus = pointsSumm / weightsSumm;
        if (overallStatus < 0.33f)
            return PerfStatus::Good;
        else if (overallStatus < 0.66f)
            return PerfStatus::Normal;
        else
            return PerfStatus::Bad;
    }

    // Draw entity counters in 3-column layout
    void PerfmonWidget::DrawEntitiesCounters()
    {
        if (_entityCounters.empty())
            return;

        ImGui::Text("Entities:");
        auto cursorX = ImGui::GetCursorPosX();

        auto width = 240.0f;
        for (size_t i = 0; i < _entityCounters.size(); i++)
        {
            auto column = i % 3;

            if (column == 1)
            {
                ImGui::SameLine();
                ImGui::SetCursorPos(ImVec2(cursorX + width / 3.0f * 1.1f, ImGui::GetCursorPos().y));
            }
            else if (column == 2)
            {
                ImGui::SameLine();
                ImGui::SetCursorPos(ImVec2(cursorX + width / 3.0f * 2.4f, ImGui::GetCursorPos().y));
            }

            DrawEntityCount(_entityCounters[i]);
        }
    }

    // Draw overall status with baseline toggle button
    void PerfmonWidget::DrawOverallStatus()
    {
        ImGui::Text("Status:");

        ImGui::SameLine();
        static const char* statusNames[] = { "Good", "Normal", "Bad" };

        ImGui::TextColored(Detail::GetStatusColor(_overallStatus), "%s", statusNames[(int)_overallStatus]);

        float baselineBtnSize = 40;
        ImGui::SameLine(ImGui::GetWindowWidth() - baselineBtnSize - 10, 0.0f);
        ImGui::Button("Base", ImVec2(baselineBtnSize, 0));

        // Check if button is clicked
        bool underCursor = ImGui::IsItemHovered(ImGuiHoveredFlags_RectOnly);

        if (underCursor && ImGui::IsMouseClicked(0))
            ToggleBaseline();
    }

    // Draw metric value with status color and baseline comparison
    void PerfmonWidget::DrawMetricRecentValue(Metric& metric)
    {
        ImGui::Text("%s:", metric.name.c_str());

        auto status = metric.GetStatus();
        auto value = metric.GetSamplesData().GetMostRecentSample();

        ImGui::SameLine();
        ImGui::TextColored(Detail::GetStatusColor(status), "%-6.1f", value);

        if (_enableBaseline)
        {
            auto deltaValue = value - metric.baselineValue;

            ImGui::SameLine();
            ImGui::Text(deltaValue > 0.0 ? "(+%-6.1f)" : "(%-6.1f)", deltaValue);
        }
    }

    // Draw profiler timeline with sample details
    void PerfmonWidget::DrawNanoProfiler()
    {
        ImGuiWindow* wnd = ImGui::GetCurrentWindow();
        ImVec2 cpos = wnd->DC.CursorPos;

        float valueWidth = 1.0f;
        float valuesGap = 0.0f;

        float targetMetricMin = 6.0f;
        float targetMetricMax = 50.0f;

        // Calculate adaptive scaling based on frame times
        for (auto& frame : _nanoProfilerSamplesHistory)
            targetMetricMin = Detail::clamp((float)frame.totalTime, targetMetricMin, targetMetricMax);

        float graphHeight = 70;
        float sampleBorder = 5.0f;
        float sampleGap = 30.0f;

        float graphWidth = (float)kNanoProfilerNumSamples * (valueWidth + valuesGap);
        float scale = (float)graphHeight / targetMetricMin;

        ImGui::SetWindowFontScale(0.8f);
        float labelHeight = ImGui::CalcTextSize("[]").y;

        // Draw each frame's samples
        for (size_t i = 0; i < _nanoProfilerSamplesHistory.size(); i++)
        {
            if (_nanoProfilerDetailFrame >= 0 && (int)i > _nanoProfilerDetailFrame)
                break;

            auto& samplesEntry = _nanoProfilerSamplesHistory[i].samples;
            bool isDetailed = i == _nanoProfilerSamplesHistory.size() - 1 || i == (size_t)_nanoProfilerDetailFrame;
            const auto x = cpos.x + (float)(i) * (valueWidth + valuesGap);

            float y = 0;
            int j = 0;

            // Draw stacked bars for each sample in frame
            for (auto& sample : samplesEntry)
            {
                float duration = (float)sample.time;
                float value = duration * (float)scale;
                float capValue = std::min(graphHeight, value);

                // Get or assign color for this sample
                ImColor valueColor;
                if (_nanoProfilerColors.find(sample.name) != _nanoProfilerColors.end())
                {
                    valueColor = _nanoProfilerColors[sample.name];
                }
                else
                {
                    valueColor = ImColor::HSV(std::fmodf((float)(_lastNanoProfilerColorIndex++ % 100) * 0.15f, 1.0f), 0.65f, 0.9f);
                    _nanoProfilerColors[sample.name] = valueColor;
                }

                float sampleY1 = std::floor(cpos.y + graphHeight - (float)capValue - y);
                float sampleY2 = std::floor(cpos.y + graphHeight - y);

                wnd->DrawList->AddRectFilled(
                    ImVec2((float)x, sampleY1),
                    ImVec2((float)(x + valueWidth), sampleY2),
                    valueColor,
                    0.f);

                // Draw details for hovered or last frame
                if (isDetailed)
                {
                    float labelY = std::floor(cpos.y + graphHeight - j * labelHeight);
                    float labelY2 = std::floor(labelY - labelHeight);

                    float labelX = cpos.x + graphWidth + sampleGap - sampleBorder;
                    float labelX2 = cpos.x + graphWidth + sampleGap;

                    float sampleX = x + valueWidth + 2;
                    float sampleX2 = x + valueWidth + 2 + sampleBorder;

                    char durationStr[64];
                    snprintf(durationStr, sizeof(durationStr), "%.2f", duration);
                    std::string text = "[" + std::string(durationStr) + "ms] " + sample.name;
                    wnd->DrawList->AddText(ImVec2(labelX2, labelY2), valueColor, text.c_str());

                    // Draw connection lines
                    ImDrawListFlags oldFlags = wnd->DrawList->Flags;
                    wnd->DrawList->Flags &= ~ImDrawListFlags_AntiAliasedLines;
                    wnd->DrawList->Flags &= ~ImDrawListFlags_AntiAliasedFill;

                    wnd->DrawList->AddQuadFilled(ImVec2(sampleX, sampleY2), ImVec2(sampleX, sampleY1), ImVec2(sampleX2, sampleY1), ImVec2(sampleX2, sampleY2), valueColor);
                    wnd->DrawList->AddQuadFilled(ImVec2(sampleX2, sampleY1), ImVec2(sampleX2, sampleY2), ImVec2(labelX, labelY), ImVec2(labelX, labelY2), valueColor);
                    wnd->DrawList->AddQuadFilled(ImVec2(labelX, labelY), ImVec2(labelX, labelY2), ImVec2(labelX2, labelY2), ImVec2(labelX2, labelY), valueColor);

                    wnd->DrawList->Flags = oldFlags;
                }

                y += capValue;
                j++;
            }
        }

        const ImVec2 graphSize(graphWidth, graphHeight);
        const ImRect graphBB(wnd->DC.CursorPos, wnd->DC.CursorPos + graphSize);
        ImGui::ItemSize(graphSize);
        ImGui::ItemAdd(graphBB, 0);

        // Handle mouse hover for frame details
        if (graphBB.Contains(ImGui::GetMousePos()))
            _nanoProfilerDetailFrame = (int)(ImGui::GetMousePos().x - cpos.x) / (int)(valueWidth + valuesGap);
        else
            _nanoProfilerDetailFrame = -1;

        ImGui::SetWindowFontScale(1.0f);
    }

    // Process and optimize profiler samples for visualization
    void PerfmonWidget::GetNanoProfilerSamplesFrame()
    {
        NANO_PROFILE_SCOPE("Nano prof");

        auto& samples = NanoProfiler::GetBufferSamples();
        NanoProfileFrame newFrame;

        // Copy samples and calculate durations
        newFrame.samples.resize(samples.size());
        for (size_t i = 0; i < samples.size(); i++)
        {
            newFrame.samples[i].name = samples[i].name;
            newFrame.samples[i].hash = std::hash<const char*>{}(samples[i].name);
            newFrame.samples[i].time = samples[i].endTime - samples[i].beginTime;
            newFrame.samples[i].parent = samples[i].parent;
        }

        // Group identical samples by name
        std::unordered_map<const char*, int> samplesMap;
        for (size_t i = 0; i < newFrame.samples.size(); i++)
        {
            auto& sample = newFrame.samples[i];
            auto it = samplesMap.find(sample.name);
            if (it != samplesMap.end())
            {
                newFrame.samples[it->second].time += sample.time;
                sample.time = 0.0;
            }
            else
                samplesMap[sample.name] = (int)i;
        }

        // Subtract child sample times from parent samples
        for (size_t i = 0; i < newFrame.samples.size(); i++)
        {
            auto* sample = &newFrame.samples[i];
            int parent = sample->parent;
            while (parent >= 0)
            {
                auto& parentSample = newFrame.samples[parent];

                parentSample.time -= sample->time;
                sample = &parentSample;
                parent = parentSample.parent;
                parentSample.parent = -1; // Prevent double subtraction
            }
        }

        // Remove samples with negligible time
        for (auto it = newFrame.samples.begin(); it != newFrame.samples.end();)
        {
            if (it->time < 0.01)
                it = newFrame.samples.erase(it);
            else
                it++;
        }

        // Extract samples starting with "_" to always show them
        std::vector<ProfileSample> alwaysVisibleSamples;
        for (auto it = newFrame.samples.begin(); it != newFrame.samples.end();)
        {
            if (it->name[0] == '_')
            {
                alwaysVisibleSamples.push_back(*it);
                it = newFrame.samples.erase(it);
            }
            else
            {
                it++;
            }
        }

        // Limit samples to keep display clean
        size_t maxSamplesDetail = 8;
        size_t avilableSamples = maxSamplesDetail - alwaysVisibleSamples.size();

        std::sort(newFrame.samples.begin(), newFrame.samples.end(), [](const ProfileSample& x, const ProfileSample& y) { return x.time > y.time; });

        if (newFrame.samples.size() > avilableSamples)
        {
            double otherTime = 0.0;
            for (size_t i = avilableSamples; i < newFrame.samples.size(); i++)
                otherTime += newFrame.samples[i].time;

            newFrame.samples.resize(avilableSamples);
            const char* otherName = "Other";
            newFrame.samples.emplace_back(ProfileSample{ otherName, std::hash<const char*> {}(otherName), otherTime, -1 });
        }

        std::sort(newFrame.samples.begin(), newFrame.samples.end(), [](const ProfileSample& x, const ProfileSample& y) { return x.hash < y.hash; });

        // Add back always visible samples
        newFrame.samples.insert(newFrame.samples.end(), alwaysVisibleSamples.begin(), alwaysVisibleSamples.end());

        // Calculate total frame time
        for (auto& sample : newFrame.samples)
            newFrame.totalTime += sample.time;

        // Add frame to history if not in detail mode
        if (_nanoProfilerDetailFrame < 0)
        {
            _nanoProfilerSamplesHistory.emplace_back(std::move(newFrame));
            if (_nanoProfilerSamplesHistory.size() > kNanoProfilerNumSamples)
                _nanoProfilerSamplesHistory.erase(_nanoProfilerSamplesHistory.begin());
        }
    }

    // Toggle baseline comparison mode and capture current values
    void PerfmonWidget::ToggleBaseline()
    {
        _enableBaseline = !_enableBaseline;

        if (_enableBaseline)
        {
            for (auto& metric : _metrics)
                metric.baselineValue = metric.GetSamplesData().GetMostRecentSample();

            for (auto& metric : _entityCounters)
                metric.baselineCount = metric.count;
        }
    }


    Metric::Metric(const std::string& name, std::function<double()> getSampleFunc, PerfMetricSettings perfSettings, std::vector<double> targetValues) :
        PerfMetricSettings(perfSettings), name(name), getSampleFunc(getSampleFunc), _targetValues(std::move(targetValues))
    {
    }

    PerfStatus Metric::GetStatus() const
    {
        return Detail::GetMetricStatus(_samplesData.P50(), badValue, goodValue);
    }

    // Update metric with adaptive target scaling
    void Metric::Update(float dt)
    {
        double current = _samplesData.P50();
        double newTarget = _targetValues[0];

        // Find appropriate target value based on current performance
        for (auto& val : _targetValues)
        {
            if (val > current)
            {
                newTarget = val;
                break;
            }
        }

        // Smoothly interpolate target value
        _targetValue = (double)Detail::lerp((float)_targetValue, (float)newTarget, dt * 5.0f);

        _timeSinceLastUpdate += dt;
        if (_timeSinceLastUpdate > _updateInterval)
        {
            _timeSinceLastUpdate = 0.0f;
            _samplesData.Push(getSampleFunc());
        }
    }
} // namespace Perfmon
