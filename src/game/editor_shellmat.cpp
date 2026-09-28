// The model editor's shell materials: what the shell triangles (the sheet body's elements) are made of. The first is
// the model's sheet material (the globals' `sheet/...`), the others are written as `set_shell_material` before their
// triangles in the shells section: a glass window, a plastic bumper, an aluminium bonnet on a steel body. Each has
// its physics (the material: stiffness, ductility, strength, bending), its areal mass, its drawn thickness and its look
// (the material's own, or a colour) (editor.h).
#include "game/editor.h"

#include "core/util.h"
#include "game/editor_internal.h"
#include "game/editor_widgets.h"

#include "imgui.h"

#include <algorithm>
#include <cfloat>
#include <cmath>

namespace bl {

using namespace edit_detail;
using namespace edit_ui;

namespace {

struct MaterialInfo {
    const char* name;
    float density;   // kg / m3 (the areal mass of a thickness)
    vec3 look;       // the colour it is drawn in the editor (and in the game unless a colour is given)
    const char* what;
};
const MaterialInfo kMaterials[] = {
    {"Steel", 7850, vec3(0.62f, 0.64f, 0.68f), "Ductile: it dents and keeps the dent, tears only after a long stretch (body panels)"},
    {"Aluminium", 2700, vec3(0.78f, 0.8f, 0.83f), "Lighter than steel, dents more easily and tears sooner (bonnets, boot lids)"},
    {"Lead", 11340, vec3(0.45f, 0.46f, 0.5f), "Heavy and soft: it folds and sags"},
    {"Glass", 2500, vec3(0.55f, 0.75f, 0.85f), "Brittle: it does not dent, it cracks and shatters into shards (windows)"},
    {"Acrylic", 1190, vec3(0.25f, 0.26f, 0.28f), "A brittle plastic, tougher than glass (bumpers, covers, light lenses)"},
    {"Plywood", 600, vec3(0.72f, 0.58f, 0.38f), "Stiff and light, splits when bent too far (floors, boxes)"},
    {"Rubber", 1100, vec3(0.12f, 0.12f, 0.13f), "Very soft and stretchy (mud flaps, seals)"},
    {"Cardboard", 700, vec3(0.66f, 0.55f, 0.4f), "Soft, creases and tears easily"},
    {"Fabric", 1500, vec3(0.3f, 0.32f, 0.35f), "Only pulls: slack when pushed, like a cloth (soft tops, covers)"},
};
constexpr int kMaterialCount = sizeof(kMaterials) / sizeof(kMaterials[0]);

int material_index(const std::string& name) {
    for (int i = 0; i < kMaterialCount; i++)
        if (name == kMaterials[i].name) return i;
    return 0;
}

// ready-made materials for the + button
struct Template {
    const char* name;
    const char* material;
    float mm;         // real thickness (the areal mass)
    float drawn;      // drawn thickness (m)
    vec3 color;       // -1: the material's own look
};
const Template kTemplates[] = {
    {"Steel panel 0.8 mm", "Steel", 0.8f, 0.004f, vec3(-1)},
    {"Aluminium panel 1 mm", "Aluminium", 1.0f, 0.004f, vec3(-1)},
    {"Window glass 4 mm", "Glass", 4.0f, 0.005f, vec3(0.55f, 0.72f, 0.8f)},
    {"Plastic bumper 3 mm", "Acrylic", 3.0f, 0.008f, vec3(0.13f, 0.13f, 0.14f)},
    {"Plywood 6 mm", "Plywood", 6.0f, 0.008f, vec3(-1)},
    {"Rubber 3 mm", "Rubber", 3.0f, 0.006f, vec3(-1)},
    {"Soft top (fabric)", "Fabric", 0.3f, 0.003f, vec3(0.15f, 0.15f, 0.17f)},
};

} // namespace

vec3 ModelEditor::shell_preset_color(int i) const {
    const edit::ShellPreset p = m_model.shell_preset(i);
    return p.color.x >= 0 ? p.color : kMaterials[material_index(p.material)].look;
}

void ModelEditor::apply_shell_preset_to_selection(int preset) {
    // the selected triangles (and every triangle among selected nodes) become shells of this material
    push_undo();
    int n = 0;
    for (int i = 0; i < (int)m_model.tris.size(); i++) {
        edit::Tri& t = m_model.tris[i];
        if (elem_selected(Elem::Tri, i) || (is_selected(t.a) && is_selected(t.b) && is_selected(t.c))) t.shell = true, t.shell_preset = preset, n++;
    }
    m_status = std::to_string(n) + " triangles are shells of " + m_model.shell_preset(preset).name;
}

void ModelEditor::ui_shell_presets() {
    edit::Model& M = m_model;
    const float row = ImGui::GetFrameHeight();
    const int n = M.shell_preset_count();
    // the list: the look, the name, the material and mass; how many triangles
    ImGui::BeginChild("##shellpresets", ImVec2(0, std::min(5.5f, (float)n + 0.3f) * (row + ImGui::GetStyle().ItemSpacing.y) + 6), ImGuiChildFlags_Borders);
    for (int i = 0; i < n; i++) {
        const edit::ShellPreset p = M.shell_preset(i);
        int count = 0;
        for (const edit::Tri& t : M.tris) count += t.shell && (t.shell_preset == i || (i == 0 && t.shell_preset > (int)M.shell_presets.size()));
        ImGui::PushID(i);
        const vec3 c = shell_preset_color(i);
        ImGui::ColorButton("##c", ImVec4(c.x, c.y, c.z, 1), ImGuiColorEditFlags_NoTooltip, ImVec2(10, row));
        ImGui::SameLine(0, 6);
        char label[160];
        snprintf(label, sizeof label, "%s%s", p.name.c_str(), i == 0 ? "  (default)" : "");
        if (ImGui::Selectable(label, m_shell_preset == i, ImGuiSelectableFlags_AllowDoubleClick, ImVec2(ImGui::GetContentRegionAvail().x - row - 40, row))) {
            m_shell_preset = i;
            if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) m_shell_window = true, m_shell_edit = i, m_shell_new = false;
        }
        ImGui::SetItemTooltip("%s, %.1f kg/m2, drawn %.1f mm\n%s\nNew shells take the checked material; double click: edit", p.material.c_str(), p.kg_m2, p.thickness * 1000,
                              kMaterials[material_index(p.material)].what);
        ImGui::SameLine();
        ImGui::TextDisabled("%4d", count);
        ImGui::SameLine();
        if (icon_button("##sel", Icon::Select, false, "Select its triangles", row)) {
            clear_selection();
            for (int t = 0; t < (int)M.tris.size(); t++)
                if (M.tris[t].shell && M.tris[t].shell_preset == i && elem_pickable(Elem::Tri, t)) m_sel_tris.push_back(t);
        }
        ImGui::PopID();
    }
    ImGui::EndChild();
    const float bs = ImGui::GetFrameHeight();
    if (icon_button("##snew", Icon::Plus, false, "A new material (from a ready-made one)", bs)) ImGui::OpenPopup("##shelltpl");
    if (ImGui::BeginPopup("##shelltpl")) {
        for (const Template& t : kTemplates) {
            if (!ImGui::Selectable(t.name)) continue;
            edit::ShellPreset p;
            p.name = t.name, p.material = t.material, p.thickness = t.drawn, p.color = t.color;
            p.kg_m2 = kMaterials[material_index(t.material)].density * t.mm * 0.001f;
            push_undo();
            M.shell_presets.push_back(p);
            m_shell_preset = M.shell_preset_count() - 1;
        }
        ImGui::Separator();
        if (ImGui::Selectable("A copy of the checked one")) {
            push_undo();
            edit::ShellPreset p = M.shell_preset(m_shell_preset);
            p.name += " copy";
            M.shell_presets.push_back(p);
            m_shell_preset = M.shell_preset_count() - 1;
            m_shell_window = true, m_shell_edit = m_shell_preset, m_shell_new = false;
        }
        ImGui::EndPopup();
    }
    ImGui::SameLine();
    if (icon_button("##sedit", Icon::Edit, m_shell_window, "Edit the checked material", bs)) m_shell_window = true, m_shell_edit = m_shell_preset, m_shell_new = false;
    ImGui::SameLine();
    if (icon_button("##sdel", Icon::Trash, false, "Delete the checked material (its triangles take the default)", bs, m_shell_preset > 0)) {
        push_undo();
        M.remove_shell_preset(m_shell_preset);
        m_shell_preset = 0;
        m_shell_window = false;
    }
    ImGui::SameLine();
    if (ImGui::Button("Apply to selection", ImVec2(-1, bs))) apply_shell_preset_to_selection(m_shell_preset);
    ImGui::SetItemTooltip("The selected triangles (and the triangles among selected nodes) become shells of the checked material");
}

void ModelEditor::ui_shell_window() {
    edit::Model& M = m_model;
    if (m_shell_edit < 0 || m_shell_edit >= M.shell_preset_count()) {
        m_shell_window = false;
        return;
    }
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + m_left_w + 12, vp->WorkPos.y + m_top_h + 60), ImGuiCond_Appearing);
    ImGui::SetNextWindowSize(ImVec2(420, 0), ImGuiCond_Appearing);
    edit::ShellPreset p = M.shell_preset(m_shell_edit);
    char title[128];
    snprintf(title, sizeof title, "Shell material: %s%s###shellwin", p.name.c_str(), m_shell_edit == 0 ? " (default)" : "");
    if (!ImGui::Begin(title, &m_shell_window, ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::End();
        return;
    }
    bool ch = false;
    const int mi = material_index(p.material);
    if (props_begin("##shellprops", 0.34f)) {
        if (m_shell_edit > 0) {
            prop("Name");
            char name[64];
            snprintf(name, sizeof name, "%s", p.name.c_str());
            if (ImGui::InputText("##name", name, sizeof name)) p.name = name, ch = true;
        }
        prop("Material", "What it is made of: how stiff, how ductile, how strong, how it bends and breaks");
        if (ImGui::BeginCombo("##mat", p.material.c_str())) {
            for (int i = 0; i < kMaterialCount; i++) {
                if (ImGui::Selectable(kMaterials[i].name, i == mi)) {
                    // (the areal mass keeps the thickness it had)
                    const float mm = p.kg_m2 / kMaterials[mi].density * 1000.0f;
                    p.material = kMaterials[i].name;
                    p.kg_m2 = kMaterials[i].density * mm * 0.001f;
                    ch = true;
                }
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", kMaterials[i].what);
            }
            ImGui::EndCombo();
        }
        prop("");
        hint(kMaterials[material_index(p.material)].what);
        prop("Real thickness", "The sheet's thickness: its areal mass (the density of the material times the thickness)");
        float mm = p.kg_m2 / kMaterials[material_index(p.material)].density * 1000.0f;
        if (ImGui::SliderFloat("##mm", &mm, 0.1f, 30.0f, "%.2f mm", ImGuiSliderFlags_Logarithmic)) p.kg_m2 = kMaterials[material_index(p.material)].density * mm * 0.001f, ch = true;
        prop("Areal mass");
        ch |= ImGui::SliderFloat("##kgm2", &p.kg_m2, 0.1f, 100.0f, "%.2f kg/m2", ImGuiSliderFlags_Logarithmic);
        prop("Drawn thickness", "How thick it is drawn (thin sheets are drawn thicker than they are)");
        float dmm = p.thickness * 1000.0f;
        if (ImGui::SliderFloat("##drawn", &dmm, 0.5f, 50.0f, "%.1f mm", ImGuiSliderFlags_Logarithmic)) p.thickness = dmm * 0.001f, ch = true;
        if (m_shell_edit > 0) {
            prop("Colour", "Off: the material's own look (the metals: the vehicle's paint)");
            bool own = p.color.x >= 0;
            if (ImGui::Checkbox("##owncol", &own)) p.color = own ? kMaterials[material_index(p.material)].look : vec3(-1), ch = true;
            if (own) {
                ImGui::SameLine();
                float c[3] = {p.color.x, p.color.y, p.color.z};
                if (ImGui::ColorEdit3("##col", c, ImGuiColorEditFlags_NoInputs)) p.color = vec3(c[0], c[1], c[2]), ch = true;
            }
        }
        prop("Refinement", "How finely a dent or a crack subdivides its triangles (-1: the default, 3)");
        ch |= ImGui::SliderInt("##lvl", &p.max_level, -1, 4, p.max_level < 0 ? "default" : "%d");
        props_end();
    }
    if (ch) {
        M.set_shell_preset(m_shell_edit, p);
        m_dirty = true;
    }
    ImGui::Separator();
    if (ImGui::Button("Apply to selection")) apply_shell_preset_to_selection(m_shell_edit);
    ImGui::SameLine();
    if (ImGui::Button("Select its triangles")) {
        clear_selection();
        for (int t = 0; t < (int)M.tris.size(); t++)
            if (M.tris[t].shell && M.tris[t].shell_preset == m_shell_edit && elem_pickable(Elem::Tri, t)) m_sel_tris.push_back(t);
    }
    ImGui::SameLine();
    if (ImGui::Button("Close")) m_shell_window = false;
    ImGui::End();
}

} // namespace bl
