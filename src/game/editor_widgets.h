// The model editor's layout helpers (editor_ui.cpp, editor_gfx.cpp, editor_deform.cpp): a property grid (the labels
// in a column on the left, the controls filling the right), section titles, hints, a force field with "never".
#pragma once

#include "imgui.h"

#include <cfloat>

namespace bl::edit_ui {

inline const ImVec4 kAccent(0.96f, 0.62f, 0.16f, 1.0f);
inline const ImVec4 kDimText(0.58f, 0.61f, 0.67f, 1.0f);

// a property grid: props_begin(), then prop("Label") before each control (it fills the right column), props_end()
inline bool props_begin(const char* id, float label_share = 0.42f) {
    if (!ImGui::BeginTable(id, 2, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_NoSavedSettings | ImGuiTableFlags_PadOuterX)) return false;
    ImGui::TableSetupColumn("label", ImGuiTableColumnFlags_WidthStretch, label_share);
    ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthStretch, 1.0f - label_share);
    return true;
}
inline void prop(const char* label, const char* tip = nullptr) {
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::AlignTextToFramePadding();
    ImGui::PushStyleColor(ImGuiCol_Text, kDimText);
    ImGui::TextUnformatted(label);
    ImGui::PopStyleColor();
    if (tip && ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) ImGui::SetTooltip("%s", tip);
    ImGui::TableNextColumn();
    ImGui::SetNextItemWidth(-FLT_MIN);
}
inline void props_end() { ImGui::EndTable(); }

// a section: a thin title line with some air above
inline void section_title(const char* s) {
    ImGui::Dummy(ImVec2(0, 4));
    ImGui::PushStyleColor(ImGuiCol_Text, kDimText);
    ImGui::SeparatorText(s);
    ImGui::PopStyleColor();
}

// a quiet explanation that wraps
inline void hint(const char* s) {
    ImGui::PushStyleColor(ImGuiCol_Text, kDimText);
    ImGui::PushTextWrapPos(0);
    ImGui::TextUnformatted(s);
    ImGui::PopTextWrapPos();
    ImGui::PopStyleColor();
}

// a force slider (log scale) with a "never" box (RoR's never: a value beyond the float range), filling the cell
inline bool never_field(const char* id, float* v, float lo, float hi, const char* fmt) {
    bool never = *v >= 1e29f, ch = false;
    ImGui::PushID(id);
    if (ImGui::Checkbox("##never", &never)) *v = never ? 1.0e30f : hi * 0.1f, ch = true;
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("never");
    ImGui::SameLine();
    ImGui::BeginDisabled(never);
    float x = never ? hi : *v;
    ImGui::SetNextItemWidth(-FLT_MIN);
    if (ImGui::SliderFloat("##v", &x, lo, hi, never ? "never" : fmt, ImGuiSliderFlags_Logarithmic) && !never) *v = x, ch = true;
    ImGui::EndDisabled();
    ImGui::PopID();
    return ch;
}

} // namespace bl::edit_ui
