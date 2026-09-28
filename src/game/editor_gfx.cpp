// The model editor's graphics, part by part: the meshes bound to the model's nodes (flexbodies, props, the wheels'
// tyres and rims, the cab's submeshes) shown or hidden in the editor, switched on or off in the vehicle, highlighted
// when selected or under the mouse, picked in the views (Alt + click), and the Graphics tab (editor.h).
//
// How a mesh follows the physical model (the Rigs of Rods scheme, vehicle/visual.cpp):
//   - a flexbody is placed in the frame of three nodes (ref: the origin, x and y: the axes, the offset measured in
//     multiples of ref->x and ref->y and along their normal, then turned by the rotation). Each of its vertices is then
//     bound to the three nearest well-spread nodes of its forset and keeps its coordinates in their frame: the mesh
//     bends, stretches and crumples with those nodes. A forset of fewer than 3 nodes: the mesh moves rigidly.
//   - a prop is rigid: it keeps its placement in the frame of its ref, x and y nodes (it moves and turns with them).
//   - a mesh wheel's rim turns with the wheel's axle, its tyre is drawn over the wheel's generated tyre nodes.
//   - a cab submesh is the cab triangles themselves, textured by their nodes' texture coordinates.
#include "game/editor.h"

#include "core/util.h"
#include "game/editor_internal.h"
#include "game/editor_widgets.h"
#include "game/game.h"
#include "vehicle/vehicle.h"
#include "vehicle/visual.h"

#include "imgui.h"

#include <algorithm>
#include <cfloat>
#include <cmath>

namespace bl {

using namespace edit_detail;
using namespace edit_ui;
using VV = VehicleVisual;

namespace {

const char* kKindNames[5] = {"", "Flexbodies", "Props", "Wheels", "Cab"};
const char* kKindTips[5] = {"", "Meshes skinned to the nodes of their forset: they bend and crumple with them", "Rigid meshes moving with their ref, x and y nodes",
                            "The wheels' tyres and rim meshes", "The cab triangles drawn with their texture coordinates (the imported submeshes, the skin)"};

} // namespace

// ------------------------------------------------------------------------------------------------ parts
int ModelEditor::gfx_code(int kind, int index) const {
    static const int map[5] = {-1, VV::PART_FLEX, VV::PART_PROP, VV::PART_WHEEL, VV::PART_CAB};
    return kind >= 1 && kind <= 4 && index >= 0 ? VV::part_code(map[kind], index) : -1;
}

void ModelEditor::gfx_of_code(int code, int& kind, int& index) {
    kind = 0, index = -1;
    if (code < 0) return;
    static const int map[4] = {4, 1, 2, 3}; // (the visual's cab, flex, prop, wheel)
    const int k = VV::part_kind(code);
    if (k < 0 || k > 3) return;
    kind = map[k], index = VV::part_index(code);
}

int ModelEditor::gfx_count(int kind) const {
    const edit::Model& M = m_model;
    switch (kind) {
    case 1: return (int)M.flexbodies.size();
    case 2: return (int)M.props.size();
    case 3: return (int)M.wheels.size();
    case 4: {
        // (the editor's own triangles are written as a submesh of their own, after the imported ones)
        bool own = false;
        for (const edit::Tri& t : M.tris) own |= t.submesh < 0;
        return (int)M.submeshes.size() + (own ? 1 : 0);
    }
    default: return 0;
    }
}

bool ModelEditor::gfx_hidden(int kind, int i) const {
    const edit::Model& M = m_model;
    switch (kind) {
    case 1: return i < (int)M.flexbodies.size() && M.flexbodies[i].hidden;
    case 2: return i < (int)M.props.size() && M.props[i].hidden;
    case 3: return i < (int)M.wheels.size() && M.wheels[i].gfx_hidden;
    case 4: return i < (int)M.submeshes.size() ? M.submeshes[i].hidden : M.skin_hidden;
    default: return false;
    }
}

void ModelEditor::gfx_set_hidden(int kind, int i, bool h) {
    edit::Model& M = m_model;
    switch (kind) {
    case 1:
        if (i < (int)M.flexbodies.size()) M.flexbodies[i].hidden = h, m_dirty = true;
        break;
    case 2:
        if (i < (int)M.props.size()) M.props[i].hidden = h, m_dirty = true;
        break;
    case 3:
        if (i < (int)M.wheels.size()) M.wheels[i].gfx_hidden = h;
        break;
    case 4:
        if (i < (int)M.submeshes.size()) M.submeshes[i].hidden = h;
        else M.skin_hidden = h;
        break;
    default: break;
    }
}

bool ModelEditor::gfx_enabled(int kind, int i) const {
    if (kind == 1) return i >= (int)m_model.flexbodies.size() || m_model.flexbodies[i].enabled;
    if (kind == 2) return i >= (int)m_model.props.size() || m_model.props[i].enabled;
    return true;
}

std::string ModelEditor::gfx_name(int kind, int i) const {
    const edit::Model& M = m_model;
    switch (kind) {
    case 1: return i < (int)M.flexbodies.size() ? M.flexbodies[i].mesh : std::string();
    case 2: return i < (int)M.props.size() ? M.props[i].mesh : std::string();
    case 3:
        if (i < (int)M.wheels.size()) {
            const edit::Wheel& w = M.wheels[i];
            return "wheel " + std::to_string(i) + (w.rim_mesh.empty() ? std::string() : "  " + w.rim_mesh);
        }
        return std::string();
    case 4: {
        if (i >= (int)M.submeshes.size()) return "skin (the editor's triangles)";
        int n = 0;
        for (const edit::Tri& t : M.tris) n += t.submesh == i;
        return "submesh " + std::to_string(i) + "  (" + std::to_string(n) + " triangles)";
    }
    default: return std::string();
    }
}

std::vector<int> ModelEditor::gfx_nodes(int kind, int i) const {
    const edit::Model& M = m_model;
    std::vector<int> out;
    switch (kind) {
    case 1:
        if (i < (int)M.flexbodies.size()) {
            const edit::Flexbody& f = M.flexbodies[i];
            out = f.forset;
            out.push_back(f.ref), out.push_back(f.x), out.push_back(f.y);
        }
        break;
    case 2:
        if (i < (int)M.props.size()) out = {M.props[i].ref, M.props[i].x, M.props[i].y};
        break;
    case 3:
        if (i < (int)M.wheels.size()) out = {M.wheels[i].n1, M.wheels[i].n2};
        break;
    case 4:
        for (const edit::Tri& t : M.tris)
            if (t.submesh == (i < (int)M.submeshes.size() ? i : -1)) out.push_back(t.a), out.push_back(t.b), out.push_back(t.c);
        break;
    default: break;
    }
    out.erase(std::remove_if(out.begin(), out.end(), [&](int n) { return !M.ref_valid(n); }), out.end());
    return out;
}

void ModelEditor::gfx_select(int kind, int index) {
    m_gfx_kind = kind, m_gfx_sel = index;
    m_gfx_pick = 0;
    m_gfx_scroll = true;
    if (kind == 3 && index < (int)m_model.wheels.size()) {
        select_elem(Elem::Wheel, index, false, false);
    } else if (kind == 4) {
        // the submesh's triangles selected (their properties in the Selection tab)
        clear_selection();
        const int sm = index < (int)m_model.submeshes.size() ? index : -1;
        for (int t = 0; t < (int)m_model.tris.size(); t++)
            if (m_model.tris[t].submesh == sm && elem_pickable(Elem::Tri, t)) m_sel_tris.push_back(t);
    }
    m_status = gfx_name(kind, index);
}

void ModelEditor::gfx_frame(int kind, int index) {
    const std::vector<int> nodes = gfx_nodes(kind, index);
    if (nodes.empty()) return;
    vec3 mn(1e30f), mx(-1e30f);
    for (int n : nodes) {
        const vec3 p = m_model.ref_position(n);
        mn = vmin(mn, p), mx = vmax(mx, p);
    }
    if (kind == 3 && index < (int)m_model.wheels.size()) mn -= vec3(m_model.wheels[index].radius), mx += vec3(m_model.wheels[index].radius);
    for (View& v : m_views) {
        v.target = (mn + mx) * 0.5f;
        v.dist = std::max(1.0f, length(mx - mn) * 1.4f);
    }
}

void ModelEditor::apply_part_states(Vehicle* v) {
    if (!v || !v->visual()) return;
    std::unordered_map<int, uint8_t> st;
    const int hover = m_gfx_ui_hover >= 0 ? m_gfx_ui_hover : m_gfx_view_hover;
    bool any_selected = false;
    for (int kind = 1; kind <= 4; kind++) {
        const int n = gfx_count(kind);
        for (int i = 0; i < n; i++) {
            const int code = gfx_code(kind, i);
            const bool sel = m_gfx_kind == kind && m_gfx_sel == i;
            uint8_t s = VV::PS_NORMAL;
            if (gfx_hidden(kind, i)) s = VV::PS_HIDDEN;
            else if (sel && m_gfx_highlight) s = VV::PS_SELECTED, any_selected = true;
            else if (code == hover) s = VV::PS_HOVER;
            else if (!gfx_enabled(kind, i)) s = VV::PS_FAINT;
            if (s != VV::PS_NORMAL) st[code] = s;
        }
    }
    v->visual()->set_part_states(std::move(st));
    v->visual()->others_alpha = any_selected ? m_gfx_others : 1.0f;
}

bool ModelEditor::gfx_view_input(bool click) {
    // Alt held (or the pick button): the mesh under the mouse is lit, a click selects it
    m_gfx_view_hover = -1;
    const bool want = (ImGui::GetIO().KeyAlt || m_gfx_pick == 6) && m_show_gfx && m_preview && alive(m_preview) && m_preview->visual();
    if (!want || ImGui::GetIO().WantCaptureMouse || view_at(m_mouse) < 0) return false;
    vec3 ro, rd;
    mouse_ray(m_active_view, ro, rd);
    const double t0 = time_seconds();
    m_gfx_view_hover = m_preview->visual()->pick(ro, rd);
    if (getenv("BL_EDITOR_LOG") && click) printf("editor: mesh pick %.2f ms\n", (time_seconds() - t0) * 1000.0);
    if (!click) return false;
    int kind, index;
    gfx_of_code(m_gfx_view_hover, kind, index);
    if (kind > 0) {
        gfx_select(kind, index);
        m_open_graphics_tab = true;
    } else {
        m_gfx_kind = 0, m_gfx_sel = -1;
        m_gfx_pick = 0;
    }
    return true;
}

// ------------------------------------------------------------------------------------------------ the graphics' nodes only
void ModelEditor::set_gfx_only(int scope) {
    m_gfx_only = std::clamp(scope, 0, 2);
    m_gfx_only_base = (int)m_model.nodes.size(); // (nodes made from now on stay shown)
    update_gfx_mask();
}

void ModelEditor::update_gfx_mask() {
    const edit::Model& M = m_model;
    const int N = (int)M.nodes.size();
    if (!m_gfx_only) {
        m_gfx_mask.clear();
        m_gfx_mask_count = N;
        return;
    }
    m_gfx_mask.assign(N, 0);
    auto add_part = [&](int kind, int i) {
        for (int n : gfx_nodes(kind, i)) {
            if (edit::is_wheel_ref(n)) {
                // (a mesh on a wheel's own nodes: the wheel's axle nodes stand for them)
                const int w = edit::wheel_of_ref(n);
                if (w >= 0 && w < (int)M.wheels.size()) {
                    if (M.wheels[w].n1 >= 0 && M.wheels[w].n1 < N) m_gfx_mask[M.wheels[w].n1] = 1;
                    if (M.wheels[w].n2 >= 0 && M.wheels[w].n2 < N) m_gfx_mask[M.wheels[w].n2] = 1;
                }
            } else if (n >= 0 && n < N) {
                m_gfx_mask[n] = 1;
            }
        }
    };
    if (m_gfx_only == 2 && m_gfx_kind > 0 && m_gfx_sel >= 0 && m_gfx_sel < gfx_count(m_gfx_kind)) {
        add_part(m_gfx_kind, m_gfx_sel);
    } else {
        for (int kind = 1; kind <= 4; kind++)
            for (int i = 0; i < gfx_count(kind); i++)
                if (!gfx_hidden(kind, i)) add_part(kind, i);
    }
    // and their neighbours, beam by beam
    for (int g = 0; g < m_gfx_only_grow; g++) {
        std::vector<char> next = m_gfx_mask;
        for (const edit::Beam& b : M.beams)
            if (m_gfx_mask[b.a] || m_gfx_mask[b.b]) next[b.a] = next[b.b] = 1;
        m_gfx_mask.swap(next);
    }
    // what is being worked on stays: the selection, the nodes made since the filter was set
    for (int n : m_sel)
        if (n < N) m_gfx_mask[n] = 1;
    for (int n : m_picks)
        if (n >= 0 && n < N) m_gfx_mask[n] = 1;
    if (m_chain >= 0 && m_chain < N) m_gfx_mask[m_chain] = 1;
    m_gfx_only_base = std::min(m_gfx_only_base, N);
    for (int n = m_gfx_only_base; n < N; n++) m_gfx_mask[n] = 1;
    m_gfx_mask_count = 0;
    for (char c : m_gfx_mask) m_gfx_mask_count += c;
}

// ------------------------------------------------------------------------------------------------ the Graphics tab
void ModelEditor::ui_graphics() {
    edit::Model& M = m_model;
    m_show_binding = true;
    // what is shown
    if (props_begin("##gfxtop")) {
        prop("Meshes", "The model's meshes over its nodes, see-through (the opacity)");
        ImGui::Checkbox("##show", &m_show_gfx);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(-FLT_MIN);
        ImGui::SliderFloat("##alpha", &m_gfx_alpha, 0.05f, 1.0f, "%.2f");
        prop("Selected part", "Tinted and opaque; the others at this opacity");
        ImGui::Checkbox("##hl", &m_gfx_highlight);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(-FLT_MIN);
        ImGui::SliderFloat("##others", &m_gfx_others, 0.0f, 1.0f, "others %.2f");
        prop("Beams and nodes", "The node-beam model's opacity (0: only the selected and the hovered)");
        ImGui::SliderFloat("##skel", &m_skel_alpha, 0.0f, 1.0f, m_skel_alpha < 0.02f ? "hidden" : "%.2f");
        prop("Which nodes", "Only the nodes the shown meshes are bound to (forsets, ref / x / y, axles, submeshes) and the elements among them; Alt+G");
        static const char* scopes[] = {"all", "the graphics' nodes", "the selected part's"};
        int sc = m_gfx_only;
        if (ImGui::Combo("##gfxonly", &sc, scopes, 3)) set_gfx_only(sc);
        if (m_gfx_only) {
            prop("  and around", "Also the nodes this many beams away");
            ImGui::SliderInt("##grow", &m_gfx_only_grow, 0, 4, m_gfx_only_grow ? "+%d beams" : "no more");
        }
        props_end();
    }
    if (!m_preview_error.empty()) ImGui::TextColored(ImVec4(1, 0.6f, 0.3f, 1), "Preview: %s", m_preview_error.c_str());
    ui_gfx_parts();
    ui_gfx_selected();
    // a new binding: a mesh of the model's folder on the selected nodes
    if (ImGui::CollapsingHeader("Bind a new mesh")) {
        const std::vector<std::string> meshes = home_meshes();
        std::vector<const char*> names;
        for (const auto& s : meshes) names.push_back(s.c_str());
        static int pick = 0;
        pick = std::clamp(pick, 0, std::max(0, (int)names.size() - 1));
        if (names.empty()) {
            hint(("No .mesh files in assets/vehicles/" + M.home).c_str());
        } else {
            if (props_begin("##newmesh")) {
                prop("Mesh");
                ImGui::Combo("##newmeshc", &pick, names.data(), (int)names.size());
                props_end();
            }
            const bool ok = m_sel.size() >= 3;
            if (!ok) hint("Select three nodes or more: the first three are its ref, x and y, all of them its forset.");
            ImGui::BeginDisabled(!ok);
            const float w = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
            if (ImGui::Button("As a flexbody", ImVec2(w, 0))) {
                push_undo();
                edit::Flexbody f;
                f.ref = m_sel[0], f.x = m_sel[1], f.y = m_sel[2], f.mesh = meshes[pick], f.forset = m_sel, f.layer = m_layer;
                M.flexbodies.push_back(f);
                gfx_select(1, (int)M.flexbodies.size() - 1);
            }
            ImGui::SetItemTooltip("Skinned to the selected nodes: it deforms with them");
            ImGui::SameLine();
            if (ImGui::Button("As a prop", ImVec2(w, 0))) {
                push_undo();
                edit::Prop p;
                p.ref = m_sel[0], p.x = m_sel[1], p.y = m_sel[2], p.mesh = meshes[pick], p.layer = m_layer;
                M.props.push_back(p);
                gfx_select(2, (int)M.props.size() - 1);
            }
            ImGui::SetItemTooltip("Rigid, moving with the first three selected nodes");
            ImGui::EndDisabled();
        }
    }
}

void ModelEditor::ui_gfx_parts() {
    edit::Model& M = m_model;
    section_title("Parts");
    // the filter, the pick button, the help
    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - 118);
    ImGui::InputTextWithHint("##gfxfilter", "filter by name", m_gfx_filter, sizeof m_gfx_filter);
    ImGui::SameLine();
    if (ImGui::Button(m_gfx_pick == 6 ? "Click one..." : "Pick in view", ImVec2(90, 0))) m_gfx_pick = m_gfx_pick == 6 ? 0 : 6;
    ImGui::SetItemTooltip("Click a mesh in a view to select it (or Alt + click at any time; Alt + the mouse over a mesh names it)");
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::BeginItemTooltip()) {
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 30);
        ImGui::TextUnformatted("How a mesh follows the nodes");
        ImGui::Separator();
        ImGui::TextUnformatted("Flexbody: placed in the frame of three nodes (ref: the origin, x and y: the axes; the offset along ref->x and ref->y and "
                               "their normal, then the rotation). Each vertex is tied to the three nearest well-spread nodes of its forset and keeps its "
                               "place in their frame: the mesh bends and crumples with them (fewer than three: it moves rigidly).");
        ImGui::Spacing();
        ImGui::TextUnformatted("Prop: rigid in the frame of its ref, x and y nodes. Wheel: the rim turns with the axle, the tyre is drawn over the wheel's "
                               "own nodes. Cab submesh: the cab triangles, textured through their nodes.");
        ImGui::Spacing();
        ImGui::TextDisabled("The eye: shown in the editor. The box: part of the vehicle (off: kept as a comment, drawn faint).");
        ImGui::TextDisabled("Click: select; double click: the views on it; Alt+H hides the selected, Alt+Shift+H shows all.");
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
    const std::string filter = to_lower(m_gfx_filter);
    auto visible_row = [&](int kind, int i) { return filter.empty() || to_lower(gfx_name(kind, i)).find(filter) != std::string::npos; };
    auto all = [&](auto fn) {
        for (int k = 1; k <= 4; k++)
            for (int i = 0; i < gfx_count(k); i++) fn(k, i);
    };
    if (ImGui::SmallButton("Show all")) all([&](int k, int i) { gfx_set_hidden(k, i, false); });
    ImGui::SameLine();
    if (ImGui::SmallButton("Hide all")) all([&](int k, int i) { gfx_set_hidden(k, i, true); });
    ImGui::SameLine();
    ImGui::BeginDisabled(m_gfx_kind <= 0 || m_gfx_sel < 0);
    if (ImGui::SmallButton("Only the selected")) all([&](int k, int i) { gfx_set_hidden(k, i, !(k == m_gfx_kind && i == m_gfx_sel)); });
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::SmallButton("Invert")) all([&](int k, int i) { gfx_set_hidden(k, i, !gfx_hidden(k, i)); });
    const float lh = ImGui::GetFrameHeightWithSpacing();
    ImGui::BeginChild("##gfxparts", ImVec2(0, lh * 10 + 8), ImGuiChildFlags_Borders | ImGuiChildFlags_ResizeY);
    const float bs = ImGui::GetFrameHeight();
    int total = 0;
    for (int kind = 1; kind <= 4; kind++) {
        const int n = gfx_count(kind);
        total += n;
        if (!n) continue;
        ImGui::PushID(kind);
        bool all_hidden = true;
        for (int i = 0; i < n; i++) all_hidden &= gfx_hidden(kind, i);
        if (icon_button("##keye", all_hidden ? Icon::EyeOff : Icon::Eye, false, all_hidden ? "Show all of them" : "Hide all of them", bs))
            for (int i = 0; i < n; i++) gfx_set_hidden(kind, i, !all_hidden);
        ImGui::SameLine();
        char hdr[64];
        snprintf(hdr, sizeof hdr, "%s  %d###cat", kKindNames[kind], n);
        const bool open = ImGui::TreeNodeEx(hdr, ImGuiTreeNodeFlags_DefaultOpen | ImGuiTreeNodeFlags_SpanAvailWidth);
        ImGui::SetItemTooltip("%s", kKindTips[kind]);
        if (open) {
            for (int i = 0; i < n; i++) {
                if (!visible_row(kind, i)) continue;
                ImGui::PushID(i);
                const bool hid = gfx_hidden(kind, i), on = gfx_enabled(kind, i);
                const bool sel = m_gfx_kind == kind && m_gfx_sel == i;
                if (icon_button("##eye", hid ? Icon::EyeOff : Icon::Eye, false, hid ? "Hidden in the editor (click: show)" : "Shown (click: hide)", bs)) gfx_set_hidden(kind, i, !hid);
                ImGui::SameLine();
                if (kind == 1 || kind == 2) {
                    bool v = on;
                    if (ImGui::Checkbox("##on", &v)) {
                        push_undo();
                        if (kind == 1) M.flexbodies[i].enabled = v;
                        else M.props[i].enabled = v;
                    }
                    ImGui::SetItemTooltip(v ? "Part of the vehicle (click: switch off)" : "Switched off: not in the vehicle, kept in the file as a comment");
                    ImGui::SameLine();
                }
                if (!on || hid) ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.5f, 0.52f, 0.56f, 1));
                if (ImGui::Selectable(gfx_name(kind, i).c_str(), sel, ImGuiSelectableFlags_AllowDoubleClick)) {
                    gfx_select(kind, i);
                    m_gfx_scroll = false;
                    if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) gfx_frame(kind, i);
                }
                if (!on || hid) ImGui::PopStyleColor();
                if (ImGui::IsItemHovered()) m_gfx_ui_hover = gfx_code(kind, i);
                if (kind == 1) {
                    ImGui::SameLine();
                    ImGui::TextDisabled("%d", (int)M.flexbodies[i].forset.size());
                    ImGui::SetItemTooltip("Nodes in its forset");
                }
                if (sel && m_gfx_scroll) ImGui::SetScrollHereY(0.4f);
                ImGui::PopID();
            }
            ImGui::TreePop();
        }
        ImGui::PopID();
    }
    if (!total) hint("No meshes. Copy a vehicle (File > Copy a vehicle), or bind a mesh below.");
    ImGui::EndChild();
    m_gfx_scroll = false;
}

void ModelEditor::ui_gfx_selected() {
    edit::Model& M = m_model;
    if (m_gfx_kind <= 0 || m_gfx_sel < 0 || m_gfx_sel >= gfx_count(m_gfx_kind)) return;
    const int kind = m_gfx_kind, sel = m_gfx_sel;
    section_title((std::string(kind == 1 ? "Flexbody: " : kind == 2 ? "Prop: " : kind == 3 ? "" : "Cab ") + gfx_name(kind, sel)).c_str());
    if (kind == 3 || kind == 4) {
        if (props_begin("##partprops")) {
            prop("Hidden");
            bool hid = gfx_hidden(kind, sel);
            if (ImGui::Checkbox("##hid", &hid)) gfx_set_hidden(kind, sel, hid);
            if (kind == 3) {
                edit::Wheel& w = M.wheels[sel];
                prop("Axle nodes");
                ImGui::Text("%d - %d", w.n1, w.n2);
                if (w.type >= 2) {
                    char b[128];
                    snprintf(b, sizeof b, "%s", w.rim_mesh.c_str());
                    prop(w.type == 4 ? "Rim mesh" : "Mesh");
                    if (ImGui::InputText("##rim", b, sizeof b, ImGuiInputTextFlags_EnterReturnsTrue)) push_undo(), w.rim_mesh = b;
                    snprintf(b, sizeof b, "%s", w.tyre_material.c_str());
                    prop(w.type == 4 ? "Tyre mesh" : "Tyre material");
                    if (ImGui::InputText("##tyre", b, sizeof b, ImGuiInputTextFlags_EnterReturnsTrue)) push_undo(), w.tyre_material = b;
                } else {
                    prop("Materials");
                    ImGui::TextDisabled("%s, %s", w.face_material.c_str(), w.band_material.c_str());
                }
            } else {
                const bool own = sel >= (int)M.submeshes.size();
                int n = 0;
                for (const edit::Tri& t : M.tris) n += t.submesh == (own ? -1 : sel);
                prop("Triangles");
                ImGui::Text("%d", n);
                prop("Material");
                ImGui::TextDisabled("%s", own ? (M.skin ? "the skin colour" : M.cab_material.c_str()) : M.cab_material.c_str());
                if (!own) prop("Texture coords"), ImGui::Text("%d", (int)M.submeshes[sel].texcoords.size());
            }
            props_end();
        }
        if (ImGui::SmallButton("View it")) gfx_frame(kind, sel);
        ImGui::SameLine();
        if (kind == 4 && ImGui::SmallButton("Select its triangles")) gfx_select(kind, sel);
        if (kind == 3) hint("Its sizes and springs: the Properties tab.");
        return;
    }
    // a flexbody or a prop: its binding to the nodes
    int *ref, *x, *y;
    vec3 *off, *rot;
    std::string* mesh;
    std::vector<int>* forset = nullptr;
    std::string* extra = nullptr;
    bool* enabled;
    if (kind == 1) {
        edit::Flexbody& f = M.flexbodies[sel];
        ref = &f.ref, x = &f.x, y = &f.y, off = &f.offset, rot = &f.rot, mesh = &f.mesh, forset = &f.forset, enabled = &f.enabled;
    } else {
        edit::Prop& p = M.props[sel];
        ref = &p.ref, x = &p.x, y = &p.y, off = &p.offset, rot = &p.rot, mesh = &p.mesh, extra = &p.extra, enabled = &p.enabled;
    }
    const std::vector<std::string> meshes = home_meshes();
    auto node_row = [&](const char* label, const char* tip, int* v, int pick_code) {
        ImGui::PushID(label);
        prop(label, tip);
        if (edit::is_wheel_ref(*v)) {
            ImGui::Text("wheel %d, node %d", edit::wheel_of_ref(*v), edit::wheel_node_of_ref(*v));
        } else {
            int n = *v;
            ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - 128);
            if (ImGui::InputInt("##n", &n, 1, 10) && n >= 0 && n < (int)M.nodes.size()) {
                const int keep_pick = m_gfx_pick;
                m_gfx_pick = pick_code;
                gfx_pick_node(n);
                m_gfx_pick = keep_pick == pick_code ? 0 : keep_pick;
            }
        }
        ImGui::SameLine();
        if (ImGui::Button(m_gfx_pick == pick_code ? "click..." : "pick", ImVec2(56, 0))) m_gfx_pick = m_gfx_pick == pick_code ? 0 : pick_code;
        ImGui::SetItemTooltip("Click a node in a view (Esc cancels)");
        ImGui::SameLine();
        ImGui::BeginDisabled(m_sel.size() != 1);
        if (ImGui::Button("sel", ImVec2(-FLT_MIN, 0))) {
            m_gfx_pick = pick_code;
            gfx_pick_node(m_sel[0]);
        }
        ImGui::SetItemTooltip("The selected node");
        ImGui::EndDisabled();
        ImGui::PopID();
    };
    if (props_begin("##bindprops", 0.3f)) {
        prop("In the vehicle", "Off: not in the vehicle, kept in the file as a comment (drawn faint here)");
        bool v = *enabled;
        if (ImGui::Checkbox("##on", &v)) push_undo(), *enabled = v;
        ImGui::SameLine(0, 18);
        bool hid = gfx_hidden(kind, sel);
        if (ImGui::Checkbox("hidden here", &hid)) gfx_set_hidden(kind, sel, hid);
        prop("Mesh");
        {
            char b[160];
            snprintf(b, sizeof b, "%s", mesh->c_str());
            ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - ImGui::GetFrameHeight() - 4);
            if (ImGui::InputText("##mesh", b, sizeof b, ImGuiInputTextFlags_EnterReturnsTrue)) push_undo(), *mesh = b;
            ImGui::SameLine(0, 4);
            if (ImGui::BeginCombo("##meshes", nullptr, ImGuiComboFlags_NoPreview | ImGuiComboFlags_PopupAlignLeft)) {
                for (const auto& s : meshes)
                    if (ImGui::Selectable(s.c_str(), s == *mesh)) push_undo(), *mesh = s;
                if (meshes.empty()) ImGui::TextDisabled("no meshes in the folder");
                ImGui::EndCombo();
            }
            ImGui::SetItemTooltip("A mesh of the model's folder");
        }
        node_row("Ref", "The origin of its frame (yellow in the views)", ref, 1);
        node_row("X", "The first axis: ref -> x (red)", x, 2);
        node_row("Y", "The second axis: ref -> y (green)", y, 3);
        prop("Keep in place", "A new ref / x / y node changes the offset and rotation so the mesh does not move");
        ImGui::Checkbox("##keep", &m_gfx_keep);
        bool ch = false;
        prop("Offset", "Along ref->x, along ref->y (multiples of those distances), along the normal (m)");
        ch |= ImGui::DragFloat3("##off", &off->x, 0.002f, -20, 20, "%.3f");
        prop("Rotation");
        ch |= ImGui::DragFloat3("##rot", &rot->x, 0.25f, -360, 360, "%.1f");
        if (ch) {
            if (!m_drag_pushed_ui) push_undo(), m_drag_pushed_ui = true;
        } else if (!ImGui::IsAnyItemActive()) {
            m_drag_pushed_ui = false;
        }
        if (extra) {
            char b[200];
            snprintf(b, sizeof b, "%s", extra->c_str());
            prop("Extra", "What follows the mesh in the file (a dashboard's wheel mesh, a beacon's colour...)");
            if (ImGui::InputText("##extra", b, sizeof b, ImGuiInputTextFlags_EnterReturnsTrue)) push_undo(), *extra = b;
        }
        props_end();
    }
    if (forset) {
        char hdr[64];
        snprintf(hdr, sizeof hdr, "Forset: %d nodes it deforms with###forset", (int)forset->size());
        if (ImGui::TreeNodeEx(hdr, ImGuiTreeNodeFlags_DefaultOpen)) {
            char b[512];
            snprintf(b, sizeof b, "%s", edit::forset_string(*forset).c_str());
            ImGui::SetNextItemWidth(-FLT_MIN);
            if (ImGui::InputText("##forset", b, sizeof b, ImGuiInputTextFlags_EnterReturnsTrue)) {
                push_undo();
                std::vector<int> v = edit::parse_forset(b);
                v.erase(std::remove_if(v.begin(), v.end(), [&](int n) { return !M.ref_valid(n); }), v.end());
                std::sort(v.begin(), v.end());
                *forset = v;
            }
            ImGui::SetItemTooltip("Ranges like 1-5, 7, 9-12 (Enter applies); white squares in the views");
            const float w = (ImGui::GetContentRegionAvail().x - 3 * ImGui::GetStyle().ItemSpacing.x) / 4;
            if (ImGui::Button("= sel", ImVec2(w, 0))) push_undo(), *forset = m_sel;
            ImGui::SetItemTooltip("The forset becomes the selected nodes");
            ImGui::SameLine();
            if (ImGui::Button("+ sel", ImVec2(w, 0))) {
                push_undo();
                for (int n : m_sel)
                    if (!std::binary_search(forset->begin(), forset->end(), n)) forset->insert(std::lower_bound(forset->begin(), forset->end(), n), n);
            }
            ImGui::SetItemTooltip("The selected nodes added");
            ImGui::SameLine();
            if (ImGui::Button("- sel", ImVec2(w, 0))) {
                push_undo();
                forset->erase(std::remove_if(forset->begin(), forset->end(), [&](int n) { return is_selected(n); }), forset->end());
            }
            ImGui::SetItemTooltip("The selected nodes taken out");
            ImGui::SameLine();
            if (ImGui::Button("select", ImVec2(w, 0))) {
                clear_selection();
                for (int n : *forset)
                    if (n < (int)M.nodes.size()) m_sel.push_back(n);
            }
            ImGui::SetItemTooltip("Select the forset's nodes");
            const float w2 = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
            if (ImGui::Button(m_gfx_pick == 4 ? "adding: click nodes..." : "add by clicking", ImVec2(w2, 0))) m_gfx_pick = m_gfx_pick == 4 ? 0 : 4;
            ImGui::SameLine();
            if (ImGui::Button(m_gfx_pick == 5 ? "removing: click nodes..." : "remove by clicking", ImVec2(w2, 0))) m_gfx_pick = m_gfx_pick == 5 ? 0 : 5;
            ImGui::TreePop();
        }
    }
    ImGui::Spacing();
    const float w = (ImGui::GetContentRegionAvail().x - 3 * ImGui::GetStyle().ItemSpacing.x) / 4;
    if (ImGui::Button("View it", ImVec2(w, 0))) gfx_frame(kind, sel);
    ImGui::SameLine();
    if (ImGui::Button("Duplicate", ImVec2(w, 0))) {
        push_undo();
        if (kind == 1) M.flexbodies.push_back(M.flexbodies[sel]), gfx_select(1, (int)M.flexbodies.size() - 1);
        else M.props.push_back(M.props[sel]), gfx_select(2, (int)M.props.size() - 1);
    }
    ImGui::SameLine();
    if (ImGui::Button("Mirror", ImVec2(w, 0))) {
        // bound to the twin nodes on the other side, the offset's normal part turned
        auto tw = [&](int n) {
            const int t = edit::is_wheel_ref(n) ? -1 : twin_or_self(n);
            return t >= 0 ? t : n;
        };
        push_undo();
        if (kind == 1) {
            edit::Flexbody f = M.flexbodies[sel];
            f.ref = tw(f.ref), f.x = tw(f.x), f.y = tw(f.y), f.offset.z = -f.offset.z;
            for (int& n : f.forset) n = tw(n);
            std::sort(f.forset.begin(), f.forset.end());
            M.flexbodies.push_back(f);
            gfx_select(1, (int)M.flexbodies.size() - 1);
        } else {
            edit::Prop p = M.props[sel];
            p.ref = tw(p.ref), p.x = tw(p.x), p.y = tw(p.y), p.offset.z = -p.offset.z;
            M.props.push_back(p);
            gfx_select(2, (int)M.props.size() - 1);
        }
    }
    ImGui::SetItemTooltip("A copy on the twin nodes of the other side (the mesh itself is not mirrored)");
    ImGui::SameLine();
    if (ImGui::Button("Delete", ImVec2(w, 0))) {
        push_undo();
        if (kind == 1) M.flexbodies.erase(M.flexbodies.begin() + sel);
        else M.props.erase(M.props.begin() + sel);
        m_gfx_kind = 0, m_gfx_sel = -1;
    }
}

} // namespace bl
