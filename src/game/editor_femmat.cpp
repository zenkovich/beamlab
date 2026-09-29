// The model editor's FEM shells: what the FEM triangles (phys::FrameTri, the truck's `fem_tris`) are made of. Each is
// a sheet of one of the frame members' materials (phys::FrameMaterial: stiffness, strength, ductility) of a real
// thickness: its membrane and bending stiffness, its mass, its yield and tearing all follow from the two. Written as
// `set_fem_shell material, thickness, r, g, b` before its triangles (editor.h).
#include "game/editor.h"

#include "core/util.h"
#include "game/editor_internal.h"
#include "game/editor_widgets.h"
#include "phys/frame_fem.h"

#include "imgui.h"

#include <algorithm>
#include <cmath>

namespace bl {

using namespace edit_detail;
using namespace edit_ui;

namespace {

// ready-made shells for the + button
struct Template {
    const char* name;
    const char* material;
    float mm;
    vec3 color;
};
const Template kTemplates[] = {
    {"Body steel 0.8 mm", "Steel", 0.8f, vec3(0.78f, 0.14f, 0.10f)},
    {"Body steel 1 mm", "Steel", 1.0f, vec3(0.78f, 0.14f, 0.10f)},
    {"Floor steel 1.5 mm", "Steel", 1.5f, vec3(0.35f, 0.36f, 0.38f)},
    {"Reinforcement 2.5 mm", "Steel", 2.5f, vec3(0.25f, 0.26f, 0.28f)},
    {"Aluminium 1.5 mm", "Aluminium", 1.5f, vec3(0.78f, 0.8f, 0.83f)},
    {"Carbon panel 2 mm", "Carbon", 2.0f, vec3(0.12f, 0.12f, 0.13f)},
    {"Plastic bumper 3 mm", "Plastic", 3.0f, vec3(0.15f, 0.15f, 0.16f)},
};

const char* material_note(const std::string& m) {
    const phys::FrameMaterial& f = phys::frame_material(m);
    static char buf[160];
    snprintf(buf, sizeof buf, "E %.0f GPa, yield %.0f MPa, tears at %.0f%% stretch, %.0f kg/m3", f.E * 1e-9f, f.yield * 1e-6f,
             f.elongation * 100, f.rho);
    return buf;
}

} // namespace

vec3 ModelEditor::fem_preset_color(int i) const {
    const edit::FemPreset p = m_model.fem_preset(i);
    return p.color.x >= 0 ? p.color : vec3(0.78f, 0.12f, 0.10f);
}

void ModelEditor::apply_fem_preset_to_selection(int preset) {
    // the selected triangles (and every triangle among selected nodes) become FEM triangles of this shell
    push_undo();
    int n = 0;
    for (int i = 0; i < (int)m_model.tris.size(); i++) {
        edit::Tri& t = m_model.tris[i];
        if (!(elem_selected(Elem::Tri, i) || (is_selected(t.a) && is_selected(t.b) && is_selected(t.c)))) continue;
        t.fem = true, t.fem_preset = preset, t.shell = false, t.collision = false;
        t.options.erase(std::remove(t.options.begin(), t.options.end(), 'h'), t.options.end());
        n++;
    }
    m_status = std::to_string(n) + " triangles are FEM triangles of " + m_model.fem_preset(preset).name;
}

void ModelEditor::ui_fem_presets() {
    edit::Model& M = m_model;
    const float row = ImGui::GetFrameHeight();
    const int n = std::max(1, (int)M.fem_presets.size());
    m_fem_preset = std::clamp(m_fem_preset, 0, n - 1);
    // the list: the look, the name; how many triangles
    ImGui::BeginChild("##fempresets", ImVec2(0, std::min(5.5f, (float)n + 0.3f) * (row + ImGui::GetStyle().ItemSpacing.y) + 6), ImGuiChildFlags_Borders);
    for (int i = 0; i < n; i++) {
        const edit::FemPreset p = M.fem_preset(i);
        int count = 0;
        float area = 0;
        for (const edit::Tri& t : M.tris)
            if (t.fem && std::clamp(t.fem_preset, 0, n - 1) == i) {
                count++;
                area += 0.5f * length(cross(M.nodes[t.b].p - M.nodes[t.a].p, M.nodes[t.c].p - M.nodes[t.a].p));
            }
        ImGui::PushID(i);
        const vec3 c = fem_preset_color(i);
        ImGui::ColorButton("##c", ImVec4(c.x, c.y, c.z, 1), ImGuiColorEditFlags_NoTooltip, ImVec2(10, row));
        ImGui::SameLine(0, 6);
        if (ImGui::Selectable(p.name.c_str(), m_fem_preset == i, ImGuiSelectableFlags_AllowDoubleClick, ImVec2(ImGui::GetContentRegionAvail().x - row - 40, row))) {
            m_fem_preset = i;
            if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) m_fem_window = true, m_fem_edit = i;
        }
        const float kg = area * p.thickness * phys::frame_material(p.material).rho;
        ImGui::SetItemTooltip("%s %.2f mm: %.2f m2, %.1f kg\n%s\nNew FEM triangles take the checked shell; double click: edit", p.material.c_str(),
                              p.thickness * 1000, area, kg, material_note(p.material));
        ImGui::SameLine();
        ImGui::TextDisabled("%4d", count);
        ImGui::SameLine();
        if (icon_button("##sel", Icon::Select, false, "Select its triangles", row)) {
            clear_selection();
            for (int t = 0; t < (int)M.tris.size(); t++)
                if (M.tris[t].fem && std::clamp(M.tris[t].fem_preset, 0, n - 1) == i && elem_pickable(Elem::Tri, t)) m_sel_tris.push_back(t);
        }
        ImGui::PopID();
    }
    ImGui::EndChild();
    const float bs = ImGui::GetFrameHeight();
    if (icon_button("##fnew", Icon::Plus, false, "A new FEM shell (from a ready-made one)", bs)) ImGui::OpenPopup("##femtpl");
    if (ImGui::BeginPopup("##femtpl")) {
        for (const Template& t : kTemplates) {
            if (!ImGui::Selectable(t.name)) continue;
            push_undo();
            M.ensure_fem_preset();
            edit::FemPreset p;
            p.name = t.name, p.material = t.material, p.thickness = t.mm * 0.001f, p.color = t.color;
            M.fem_presets.push_back(p);
            m_fem_preset = (int)M.fem_presets.size() - 1;
        }
        ImGui::Separator();
        if (ImGui::Selectable("A copy of the checked one")) {
            push_undo();
            M.ensure_fem_preset();
            edit::FemPreset p = M.fem_preset(m_fem_preset);
            p.name += " copy";
            M.fem_presets.push_back(p);
            m_fem_preset = (int)M.fem_presets.size() - 1;
            m_fem_window = true, m_fem_edit = m_fem_preset;
        }
        ImGui::EndPopup();
    }
    ImGui::SameLine();
    if (icon_button("##fedit", Icon::Edit, m_fem_window, "Edit the checked shell", bs)) m_fem_window = true, m_fem_edit = m_fem_preset;
    ImGui::SameLine();
    if (icon_button("##fdel", Icon::Trash, false, "Delete the checked shell (its triangles take the first)", bs, M.fem_presets.size() > 1)) {
        push_undo();
        M.remove_fem_preset(m_fem_preset);
        m_fem_preset = 0;
        m_fem_window = false;
    }
    ImGui::SameLine();
    if (ImGui::Button("Apply to selection##fem", ImVec2(-1, bs))) {
        M.ensure_fem_preset();
        apply_fem_preset_to_selection(m_fem_preset);
    }
    ImGui::SetItemTooltip("The selected triangles (and the triangles among selected nodes) become FEM triangles of the checked shell");
}

void ModelEditor::ui_fem_window() {
    edit::Model& M = m_model;
    M.ensure_fem_preset();
    if (m_fem_edit < 0 || m_fem_edit >= (int)M.fem_presets.size()) {
        m_fem_window = false;
        return;
    }
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + m_left_w + 12, vp->WorkPos.y + m_top_h + 60), ImGuiCond_Appearing);
    ImGui::SetNextWindowSize(ImVec2(420, 0), ImGuiCond_Appearing);
    edit::FemPreset& p = M.fem_presets[m_fem_edit];
    char title[128];
    snprintf(title, sizeof title, "FEM shell: %s###femwin", p.name.c_str());
    if (!ImGui::Begin(title, &m_fem_window, ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::End();
        return;
    }
    bool ch = false;
    int count = 0;
    const phys::FrameMaterial* mats = phys::frame_materials(count);
    if (props_begin("##femprops", 0.34f)) {
        prop("Name");
        char name[64];
        snprintf(name, sizeof name, "%s", p.name.c_str());
        if (ImGui::InputText("##name", name, sizeof name)) p.name = name, ch = true;
        prop("Material", "The frame members' materials: elastic, then plastic past the yield stress, torn past the elongation");
        if (ImGui::BeginCombo("##mat", p.material.c_str())) {
            for (int i = 0; i < count; i++) {
                if (ImGui::Selectable(mats[i].name, p.material == mats[i].name)) p.material = mats[i].name, ch = true;
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", material_note(mats[i].name));
            }
            ImGui::EndCombo();
        }
        prop("");
        hint(material_note(p.material));
        prop("Thickness", "The sheet's real thickness: its membrane stiffness goes as t, its bending stiffness as t^3, its mass as t");
        float mm = p.thickness * 1000.0f;
        if (ImGui::SliderFloat("##mm", &mm, 0.3f, 20.0f, "%.2f mm", ImGuiSliderFlags_Logarithmic)) p.thickness = std::max(0.1f, mm) * 0.001f, ch = true;
        prop("");
        char kg[32];
        snprintf(kg, sizeof kg, "%.1f kg/m2", p.thickness * phys::frame_material(p.material).rho);
        hint(kg);
        prop("Colour", "Off: the vehicle's paint");
        bool own = p.color.x >= 0;
        if (ImGui::Checkbox("##owncol", &own)) p.color = own ? vec3(0.78f, 0.14f, 0.10f) : vec3(-1), ch = true;
        if (own) {
            ImGui::SameLine();
            float c[3] = {p.color.x, p.color.y, p.color.z};
            if (ImGui::ColorEdit3("##col", c, ImGuiColorEditFlags_NoInputs)) p.color = vec3(c[0], c[1], c[2]), ch = true;
        }
        props_end();
    }
    if (ch) m_dirty = true;
    ImGui::Separator();
    if (ImGui::Button("Apply to selection")) apply_fem_preset_to_selection(m_fem_edit);
    ImGui::SameLine();
    if (ImGui::Button("Select its triangles")) {
        clear_selection();
        for (int t = 0; t < (int)M.tris.size(); t++)
            if (M.tris[t].fem && M.tris[t].fem_preset == m_fem_edit && elem_pickable(Elem::Tri, t)) m_sel_tris.push_back(t);
    }
    ImGui::SameLine();
    if (ImGui::Button("Close")) m_fem_window = false;
    ImGui::End();
}

} // namespace bl
