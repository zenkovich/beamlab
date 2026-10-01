// The model editor's collision volumes (Model::volumes, the truck's `collision_volumes`, phys::CollisionVolume): convex
// hulls riding on anchor nodes of the frame - a car's engine, its cabin, its trunk's load. The Volume tool (editor.h):
// a volume made from the selected nodes (its anchors) with a box inside them; its points dragged in the views (with
// symmetry their mirrors too), added with Ctrl+click, deleted with Delete; Shift+click on a node makes it an anchor
// or takes it off. The panel: the list, the break rms, the anchors and the hull from the selection, grow / shrink, the
// mirror points, the points' coordinates; what the hull holds (the nodes inside it are lit in the views).
#include "game/editor.h"

#include "game/editor_internal.h"
#include "game/editor_widgets.h"
#include "phys/softbody.h"

#include "imgui.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace bl {

using namespace edit_detail;
using namespace edit_ui;

namespace {
constexpr int kMaxPoints = 64; // (phys: SoftBody::add_volume takes up to 64)
}

const ModelEditor::VolumeHull& ModelEditor::volume_hull(int v) const {
    static const VolumeHull none;
    if (v < 0 || v >= (int)m_model.volumes.size()) return none;
    if ((int)m_vol_hulls.size() != (int)m_model.volumes.size()) m_vol_hulls.assign(m_model.volumes.size(), VolumeHull());
    VolumeHull& h = m_vol_hulls[v];
    const std::vector<vec3>& pts = m_model.volumes[v].verts;
    bool same = h.pts.size() == pts.size();
    for (size_t k = 0; same && k < pts.size(); k++) same = h.pts[k].x == pts[k].x && h.pts[k].y == pts[k].y && h.pts[k].z == pts[k].z;
    if (!same) {
        h.pts = pts;
        h.ok = (int)pts.size() <= kMaxPoints && phys::convex_hull(pts, h.planes, h.faces);
    }
    return h;
}

int ModelEditor::volume_point_twin(int v, int k) const {
    if (v < 0 || v >= (int)m_model.volumes.size()) return -1;
    const std::vector<vec3>& P = m_model.volumes[v].verts;
    if (k < 0 || k >= (int)P.size() || std::fabs(P[k].z) < 1e-3f) return -1;
    for (int j = 0; j < (int)P.size(); j++)
        if (j != k && std::fabs(P[j].x - P[k].x) < 1e-3f && std::fabs(P[j].y - P[k].y) < 1e-3f && std::fabs(P[j].z + P[k].z) < 1e-3f) return j;
    return -1;
}

std::vector<int> ModelEditor::volume_nodes_inside(int v) const {
    std::vector<int> out;
    const VolumeHull& h = volume_hull(v);
    if (!h.ok) return out;
    for (int i = 0; i < (int)m_model.nodes.size(); i++) {
        const vec3 p = m_model.nodes[i].p;
        bool in = true;
        for (const vec4& pl : h.planes) in &= dot(pl.xyz(), p) - pl.w < 0.0f;
        if (in) out.push_back(i);
    }
    return out;
}

namespace {
// the box of these nodes' points, inset by d where it is wide enough (at most a third of its size a side)
std::vector<vec3> inset_box(const edit::Model& M, const std::vector<int>& ids, float d) {
    vec3 mn(1e30f), mx(-1e30f);
    for (int n : ids) mn = vmin(mn, M.nodes[n].p), mx = vmax(mx, M.nodes[n].p);
    for (int a = 0; a < 3; a++) {
        const float in = std::min(d, (mx[a] - mn[a]) / 3.0f);
        mn[a] += in, mx[a] -= in;
        if (mx[a] - mn[a] < 0.02f) { // (flat: a slab of 2 cm about its middle)
            const float c = 0.5f * (mn[a] + mx[a]);
            mn[a] = c - 0.01f, mx[a] = c + 0.01f;
        }
    }
    std::vector<vec3> out;
    for (int k = 0; k < 8; k++) out.push_back(vec3(k & 1 ? mx.x : mn.x, k & 2 ? mx.y : mn.y, k & 4 ? mx.z : mn.z));
    return out;
}
} // namespace

void ModelEditor::volume_from_selection() {
    std::vector<int> ids = selection_nodes_all();
    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
    if (ids.size() < 3) {
        m_status = "Select three nodes of the frame or more: the new volume's anchors (a box inside them is its hull)";
        return;
    }
    push_undo();
    edit::Volume v;
    for (int k = 1;; k++) {
        char name[32];
        snprintf(name, sizeof name, "volume%d", k);
        if (std::none_of(m_model.volumes.begin(), m_model.volumes.end(), [&](const edit::Volume& o) { return o.name == name; })) {
            v.name = name;
            break;
        }
    }
    v.anchors = ids;
    v.verts = inset_box(m_model, ids, m_vol_inset);
    m_model.volumes.push_back(v);
    m_vol = (int)m_model.volumes.size() - 1, m_vol_point = -1;
    m_status = "Volume '" + v.name + "': " + std::to_string(ids.size()) + " anchors, a box " + std::to_string((int)std::lround(m_vol_inset * 100)) +
               " cm inside them; drag its points, Shift+click nodes to add or take off anchors";
}

void ModelEditor::volume_box_from_selection(int v) {
    const std::vector<int> ids = selection_nodes_all();
    if (v < 0 || ids.size() < 2) {
        m_status = "Select the nodes the box goes round (it is inset from their box)";
        return;
    }
    push_undo();
    m_model.volumes[v].verts = inset_box(m_model, ids, m_vol_inset);
    m_vol_point = -1;
}

void ModelEditor::volume_points_from_selection(int v) {
    std::vector<int> ids = selection_nodes_all();
    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
    if (v < 0 || ids.size() < 4) {
        m_status = "Select four nodes or more off one plane: their points become the hull's (those inside it are dropped)";
        return;
    }
    push_undo();
    std::vector<vec3>& P = m_model.volumes[v].verts;
    P.clear();
    for (int n : ids) P.push_back(m_model.nodes[n].p);
    volume_prune(v);
    m_vol_point = -1;
}

void ModelEditor::volume_anchors_from_selection(int v, int how) {
    std::vector<int> ids = selection_nodes_all();
    if (v < 0 || ids.empty()) {
        m_status = "Select the frame nodes the volume rides on";
        return;
    }
    push_undo();
    std::vector<int>& an = m_model.volumes[v].anchors;
    if (how == 0) an.clear();
    for (int n : ids) {
        const auto it = std::find(an.begin(), an.end(), n);
        if (how == 2) {
            if (it != an.end()) an.erase(it);
        } else if (it == an.end()) {
            an.push_back(n);
        }
    }
    std::sort(an.begin(), an.end());
}

void ModelEditor::volume_toggle_anchor(int v, int node) {
    if (v < 0 || node < 0) return;
    push_undo();
    std::vector<int>& an = m_model.volumes[v].anchors;
    const bool has = std::find(an.begin(), an.end(), node) != an.end();
    for (int n : {node, m_symmetry ? twin_or_self(node) : -1}) {
        if (n < 0) continue;
        const auto it = std::find(an.begin(), an.end(), n);
        if (has && it != an.end()) an.erase(it);
        else if (!has && it == an.end()) an.push_back(n);
    }
    std::sort(an.begin(), an.end());
    m_status = "Node " + std::to_string(node) + (has ? " is no anchor now" : " is an anchor now") + " (" + std::to_string(an.size()) + " anchors)";
}

void ModelEditor::volume_add_point(int v, vec3 p) {
    if (v < 0) return;
    std::vector<vec3>& P = m_model.volumes[v].verts;
    const bool twin = m_symmetry && std::fabs(p.z) >= 1e-3f;
    if ((int)P.size() + (twin ? 2 : 1) > kMaxPoints) {
        m_status = "A volume has at most 64 points";
        return;
    }
    push_undo();
    P.push_back(p);
    m_vol_point = (int)P.size() - 1;
    if (twin) P.push_back(vec3(p.x, p.y, -p.z));
}

void ModelEditor::volume_remove_point(int v, int k) {
    if (v < 0 || k < 0 || k >= (int)m_model.volumes[v].verts.size()) return;
    push_undo();
    const int t = m_symmetry ? volume_point_twin(v, k) : -1;
    std::vector<vec3>& P = m_model.volumes[v].verts;
    if (t > k) P.erase(P.begin() + t);
    P.erase(P.begin() + k);
    if (t >= 0 && t < k) P.erase(P.begin() + t);
    m_vol_point = -1;
}

void ModelEditor::volume_offset(int v, float d) {
    if (v < 0 || m_model.volumes[v].verts.empty()) return;
    push_undo();
    std::vector<vec3>& P = m_model.volumes[v].verts;
    vec3 c(0);
    for (const vec3& p : P) c += p / (float)P.size();
    for (vec3& p : P)
        if (length(p - c) > 1e-4f) p += normalize(p - c) * d;
}

void ModelEditor::volume_mirror(int v) {
    if (v < 0) return;
    push_undo();
    std::vector<vec3>& P = m_model.volumes[v].verts;
    const size_t n = P.size();
    for (size_t k = 0; k < n; k++)
        if (std::fabs(P[k].z) >= 1e-3f && volume_point_twin(v, (int)k) < 0) P.push_back(vec3(P[k].x, P[k].y, -P[k].z));
    volume_prune(v);
}

void ModelEditor::volume_prune(int v) {
    // (the points no face of the hull lies on: inside it, of no use)
    std::vector<vec3>& P = m_model.volumes[v].verts;
    std::vector<vec4> planes;
    std::vector<std::vector<uint8_t>> faces;
    if (P.size() > 255 || !phys::convex_hull(P, planes, faces)) return;
    std::vector<char> on(P.size(), 0);
    for (const auto& f : faces)
        for (uint8_t k : f) on[k] = 1;
    std::vector<vec3> kept;
    for (size_t k = 0; k < P.size(); k++)
        if (on[k]) kept.push_back(P[k]);
    if (kept.size() > (size_t)kMaxPoints) kept.resize(kMaxPoints);
    P.swap(kept);
}

void ModelEditor::delete_volume(int v) {
    if (v < 0 || v >= (int)m_model.volumes.size()) return;
    push_undo();
    m_model.volumes.erase(m_model.volumes.begin() + v);
    m_vol = std::min(v, (int)m_model.volumes.size() - 1), m_vol_point = -1;
}

void ModelEditor::ui_volumes() {
    edit::Model& M = m_model;
    const float row = ImGui::GetFrameHeight();
    if (m_vol >= (int)M.volumes.size()) m_vol = (int)M.volumes.size() - 1;
    hint("What fills a car - the engine, the seats and the crew, the load: convex hulls riding on frame nodes (anchors). Other bodies and the car's "
         "own parts on mounts are kept out of them; crushed past their break rms they are off");
    // the list: the name, anchors / points; valid or not
    ImGui::BeginChild("##volumes", ImVec2(0, std::min(5.5f, (float)M.volumes.size() + 0.3f) * (row + ImGui::GetStyle().ItemSpacing.y) + 6), ImGuiChildFlags_Borders);
    for (int i = 0; i < (int)M.volumes.size(); i++) {
        const edit::Volume& v = M.volumes[i];
        const bool ok = v.anchors.size() >= 3 && volume_hull(i).ok;
        ImGui::PushID(i);
        ImGui::ColorButton("##c", ok ? ImVec4(1.0f, 0.25f, 0.85f, 1) : ImVec4(0.9f, 0.2f, 0.15f, 1), ImGuiColorEditFlags_NoTooltip, ImVec2(10, row));
        ImGui::SameLine(0, 6);
        if (ImGui::Selectable(v.name.empty() ? "(unnamed)" : v.name.c_str(), m_vol == i, 0, ImVec2(ImGui::GetContentRegionAvail().x - 70, row))) m_vol = i, m_vol_point = -1;
        ImGui::SetItemTooltip("%zu anchors, %zu points%s", v.anchors.size(), v.verts.size(), ok ? "" : "\nnot valid: 3 anchors and 4 points off one plane at least");
        ImGui::SameLine();
        ImGui::TextDisabled("%3zu / %zu", v.anchors.size(), v.verts.size());
        ImGui::PopID();
    }
    if (M.volumes.empty()) ImGui::TextDisabled("none: select the frame nodes round a space, then New");
    ImGui::EndChild();
    const float bs = ImGui::GetFrameHeight();
    if (icon_button("##vnew", Icon::Plus, false, "A new volume: the selected nodes its anchors, a box inside them its hull", bs)) volume_from_selection();
    ImGui::SameLine();
    const int v = active_volume();
    if (icon_button("##vdel", Icon::Trash, false, "Delete the volume", bs, v >= 0)) delete_volume(v);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(-FLT_MIN);
    float cm = m_vol_inset * 100.0f;
    if (ImGui::SliderFloat("##inset", &cm, 0.0f, 30.0f, "box inset %.0f cm")) m_vol_inset = cm * 0.01f;
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("How far a new box (New, Box round selection) stands inside the selected nodes' box");
    if (v < 0) return;
    edit::Volume& V = M.volumes[v];
    const VolumeHull& H = volume_hull(v);
    if (!props_begin("##volprops")) return;
    prop("Name", "The volume's name in the file and in the game's status line");
    char buf[64];
    snprintf(buf, sizeof buf, "%s", V.name.c_str());
    if (ImGui::InputText("##vname", buf, sizeof buf)) {
        if (!m_drag_pushed_ui) push_undo(), m_drag_pushed_ui = true;
        std::string s(buf);
        std::replace(s.begin(), s.end(), ' ', '_'); // (a word in the file)
        std::replace(s.begin(), s.end(), ',', '_');
        V.name = s;
    }
    if (ImGui::IsItemDeactivated()) m_drag_pushed_ui = false;
    prop("Break rms", "How far out of their shape its anchors may be crushed or torn (the rms of their best fit) before the volume is off for good");
    float rms = V.break_rms * 100.0f;
    if (ImGui::IsItemActivated()) {}
    if (ImGui::SliderFloat("##vrms", &rms, 2.0f, 50.0f, "%.0f cm")) {
        if (!m_drag_pushed_ui) push_undo(), m_drag_pushed_ui = true;
        V.break_rms = rms * 0.01f;
    }
    if (ImGui::IsItemDeactivated()) m_drag_pushed_ui = false;
    prop("Crush force", "The force of its contacts (all of them) it is off past, held 3 ms: a bumper's reinforcement crushed, the beam under it takes over (0: never)");
    float kn = V.break_force * 0.001f;
    if (ImGui::SliderFloat("##vcrush", &kn, 0.0f, 1000.0f, kn > 0 ? "%.0f kN" : "never", ImGuiSliderFlags_Logarithmic)) {
        if (!m_drag_pushed_ui) push_undo(), m_drag_pushed_ui = true;
        V.break_force = kn * 1000.0f;
    }
    if (ImGui::IsItemDeactivated()) m_drag_pushed_ui = false;
    prop("Drawn", "Drawn in the game as a solid of this colour (an engine block, a gearbox: the car's simplified machinery)");
    bool drawn = V.color.x >= 0;
    if (ImGui::Checkbox("##vdrawn", &drawn)) push_undo(), V.color = drawn ? vec3(0.32f, 0.33f, 0.35f) : vec3(-1);
    if (drawn) {
        ImGui::SameLine();
        ImGui::SetNextItemWidth(-FLT_MIN);
        vec3 c = V.color;
        if (ImGui::ColorEdit3("##vcol", &c.x, ImGuiColorEditFlags_NoInputs)) {
            if (!m_drag_pushed_ui) push_undo(), m_drag_pushed_ui = true;
            V.color = c;
        }
        if (ImGui::IsItemDeactivated()) m_drag_pushed_ui = false;
    }
    // what it is: faces, size, the nodes in it
    const std::vector<int> inside = volume_nodes_inside(v);
    int in_anchors = 0;
    for (int n : inside) in_anchors += std::find(V.anchors.begin(), V.anchors.end(), n) != V.anchors.end();
    prop("Hull");
    if (!H.ok) {
        ImGui::TextColored(ImVec4(0.95f, 0.35f, 0.3f, 1), V.verts.size() > (size_t)kMaxPoints ? "over 64 points" : "its points span no volume");
    } else {
        vec3 mn(1e30f), mx(-1e30f);
        for (const vec3& p : V.verts) mn = vmin(mn, p), mx = vmax(mx, p);
        ImGui::Text("%zu faces, %.2f x %.2f x %.2f m", H.faces.size(), mx.x - mn.x, mx.y - mn.y, mx.z - mn.z);
    }
    prop("Anchors", "The frame nodes it rides on (placed every step by their best fit): three at least, spread round it (floor, pillars, rails)");
    if (V.anchors.size() < 3) ImGui::TextColored(ImVec4(0.95f, 0.35f, 0.3f, 1), "%zu: three at least", V.anchors.size());
    else ImGui::Text("%zu", V.anchors.size());
    prop("Nodes inside", "The model's nodes in the hull (ringed orange in the views): a part's nodes in it are not held off (the game warns); the frame's may be");
    ImGui::Text("%zu (%d anchors)", inside.size(), in_anchors);
    prop("Show them");
    ImGui::Checkbox("##vin", &m_vol_show_inside);
    props_end();
    // the anchors and the hull from the selection
    section_title("Anchors");
    const float w3 = (ImGui::GetContentRegionAvail().x - 2 * ImGui::GetStyle().ItemSpacing.x) / 3.0f;
    if (ImGui::Button("= selection", ImVec2(w3, 0))) volume_anchors_from_selection(v, 0);
    ImGui::SetItemTooltip("The selected nodes are its anchors");
    ImGui::SameLine();
    if (ImGui::Button("+ selection", ImVec2(w3, 0))) volume_anchors_from_selection(v, 1);
    ImGui::SetItemTooltip("The selected nodes are anchors too");
    ImGui::SameLine();
    if (ImGui::Button("- selection", ImVec2(w3, 0))) volume_anchors_from_selection(v, 2);
    ImGui::SetItemTooltip("The selected nodes are no anchors");
    if (ImGui::Button("Select the anchors", ImVec2(-FLT_MIN, 0))) {
        clear_selection();
        for (int n : V.anchors)
            if (n >= 0 && n < (int)M.nodes.size()) sorted_insert(m_sel, n);
    }
    section_title("Hull");
    const float w2 = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) / 2.0f;
    if (ImGui::Button("Box round selection", ImVec2(w2, 0))) volume_box_from_selection(v);
    ImGui::SetItemTooltip("Its points: the box of the selected nodes, inset (the slider above)");
    ImGui::SameLine();
    if (ImGui::Button("Points of selection", ImVec2(w2, 0))) volume_points_from_selection(v);
    ImGui::SetItemTooltip("Its points: the selected nodes' (those inside their hull dropped)");
    if (ImGui::Button("Shrink 1 cm", ImVec2(w3, 0))) volume_offset(v, -0.01f);
    ImGui::SameLine();
    if (ImGui::Button("Grow 1 cm", ImVec2(w3, 0))) volume_offset(v, 0.01f);
    ImGui::SameLine();
    if (ImGui::Button("Mirror", ImVec2(w3, 0))) volume_mirror(v);
    ImGui::SetItemTooltip("The points' mirrors across the middle (z = 0) added where missing: a symmetric hull");
    if (ImGui::Button("Drop the points inside", ImVec2(-FLT_MIN, 0))) push_undo(), volume_prune(v), m_vol_point = -1;
    // the points
    char head[48];
    snprintf(head, sizeof head, "Points (%zu)###vpts", V.verts.size());
    if (ImGui::TreeNode(head)) {
        for (int k = 0; k < (int)V.verts.size(); k++) {
            ImGui::PushID(k);
            if (ImGui::Selectable("##pt", m_vol_point == k, 0, ImVec2(18, 0))) m_vol_point = k;
            ImGui::SameLine();
            ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - row - 4);
            vec3 p = V.verts[k];
            if (ImGui::DragFloat3("##p", &p.x, 0.005f, -10.0f, 10.0f, "%.3f")) {
                if (!m_drag_pushed_ui) push_undo(), m_drag_pushed_ui = true;
                const int t = m_symmetry ? volume_point_twin(v, k) : -1;
                V.verts[k] = p;
                if (t >= 0) V.verts[t] = vec3(p.x, p.y, -p.z);
            }
            if (ImGui::IsItemDeactivated()) m_drag_pushed_ui = false;
            ImGui::SameLine();
            if (icon_button("##x", Icon::Trash, false, "Delete the point (with symmetry its mirror too)", row)) volume_remove_point(v, k);
            ImGui::PopID();
        }
        ImGui::TreePop();
    }
}

} // namespace bl
