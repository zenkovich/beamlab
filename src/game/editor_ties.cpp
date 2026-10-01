// The model editor's ties: what holds the parts on the body - the mounts (Model::mounts, the truck's `mounts`,
// phys::FrameMount: a door's latch, its hinges, a bumper's clamped bolts, a hood's buffers, a lid's stays) and the
// sheet's welds to the frame (Model::welds, the truck's `welds`). The Ties tool (editor.h): click the body's node, then
// the part's (a hinge: then the part's second node on its line; a weld: the frame's node, then the sheet's) for a new
// tie of the kind chosen in the panel (with symmetry the twin too); click a tie's line to make it the active one,
// Delete takes it off (and its twin), Shift+click a node gives the active hinge its second node. The panel: the kinds,
// the list (by kind), the active one's settings (with symmetry its twin's too).
#include "game/editor.h"

#include "game/editor_internal.h"
#include "game/editor_widgets.h"

#include "imgui.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace bl {

using namespace edit_detail;
using namespace edit_ui;

namespace {
// the kinds of a new tie: the mounts' (edit::Mount::kind), then a weld
constexpr int kWeld = 5;
const char kMountKinds[] = {'p', 'c', 'h', 's', 'r'};
const char* kTieNames[] = {"Latch", "Bolt", "Hinge", "Buffer", "Stay", "Weld"};
const char* kTieTips[] = {
    "A latch (a point): the part's node held at its place on the body's node's frame, turning freely; it lets go past its break force (a door's, a hood's, a lid's lock)",
    "A clamped bolt: the part's node and its ring of nodes held at their angle too; it lets go past its break force, or twisted past its break moment (a bumper's brackets, a lamp's clips)",
    "A hinge: the part's two nodes on its line held, the part turns about it only (a door's, a hood's, a lid's hinges); it tears past its break force",
    "A buffer (a stop): pushes the part's node off the body's node only, never pulls (a door's, a hood's rubber stops: with the latch gone the part rests on them)",
    "A stay (a strap): pulls the part's node back only once it is farther than its length times the distance at rest (a lid's gas strut, a door's check strap)",
    "A weld: the sheet's node held on a frame node, its pull spread over the sheet's nodes round it within the radius (the sheet body on the frame)",
};
const ImVec4 kTieCols[] = {{1.0f, 0.45f, 0.15f, 1}, {1.0f, 0.22f, 0.25f, 1}, {0.35f, 0.7f, 1.0f, 1}, {0.62f, 0.64f, 0.7f, 1}, {0.45f, 1.0f, 0.45f, 1}, {1.0f, 0.85f, 0.25f, 1}};

int mount_kind(char k) {
    for (int i = 0; i < 5; i++)
        if (kMountKinds[i] == k) return i;
    return 0;
}
} // namespace

int ModelEditor::tie_nodes(int t, int n[3]) const {
    const edit::Model& M = m_model;
    if (t < 0 || t >= tie_count()) return 0;
    if (t < (int)M.mounts.size()) {
        const edit::Mount& m = M.mounts[t];
        n[0] = m.a, n[1] = m.b, n[2] = m.b2;
        return m.kind == 'h' && m.b2 >= 0 ? 3 : 2;
    }
    const edit::Weld& w = M.welds[t - M.mounts.size()];
    n[0] = w.anchor, n[1] = w.node;
    return 2;
}

int ModelEditor::tie_at(vec2 mouse) const {
    // the nearest line within 8 px (a hinge: its line to the second node too), the active one first
    const int N = (int)m_model.nodes.size();
    int best = -1;
    float bd = 8.0f * 8.0f;
    for (int t = 0; t < tie_count(); t++) {
        int n[3];
        const int c = tie_nodes(t, n);
        if (t < (int)m_model.mounts.size() && m_tie_filter >= 0 && m_tie_filter != mount_kind(m_model.mounts[t].kind)) continue;
        if (t >= (int)m_model.mounts.size() && m_tie_filter >= 0 && m_tie_filter != kWeld) continue;
        vec2 s[3];
        bool ok = true;
        for (int k = 0; k < c; k++) ok &= n[k] >= 0 && n[k] < N && node_shown(n[k]) && project(m_active_view, to_world(m_model.nodes[n[k]].p), s[k]);
        if (!ok) continue;
        for (int k = 0; k + 1 < c; k++) {
            const float d = seg_dist2(mouse, s[k], s[k + 1]) - (t == m_tie ? 9.0f : 0.0f);
            if (d < bd) bd = d, best = t;
        }
    }
    return best;
}

int ModelEditor::tie_twin(int t) const {
    const edit::Model& M = m_model;
    int n[3], w[3];
    const int c = tie_nodes(t, n);
    if (!c) return -1;
    for (int k = 0; k < c; k++)
        if ((w[k] = twin_or_self(n[k])) < 0) return -1;
    bool self = true;
    for (int k = 0; k < c; k++) self &= w[k] == n[k];
    if (self) return -1; // (on the plane: its own mirror)
    const bool mount = t < (int)M.mounts.size();
    const int lo = mount ? 0 : (int)M.mounts.size(), hi = mount ? (int)M.mounts.size() : tie_count();
    for (int u = lo; u < hi; u++) {
        int q[3];
        if (u == t || tie_nodes(u, q) != c) continue;
        if (mount && M.mounts[u].kind != M.mounts[t].kind) continue;
        bool same = true;
        for (int k = 0; k < c; k++) same &= q[k] == w[k];
        if (same) return u;
    }
    return -1;
}

void ModelEditor::add_tie(int kind, const std::vector<int>& nodes) {
    edit::Model& M = m_model;
    if (nodes.size() < 2) return;
    push_undo();
    std::vector<std::vector<int>> sets = {nodes};
    if (m_symmetry) {
        std::vector<int> w;
        bool self = true;
        for (int n : nodes) w.push_back(twin_or_self(n)), self &= w.back() == n;
        if (!self && std::find(w.begin(), w.end(), -1) == w.end()) sets.push_back(w);
    }
    for (const std::vector<int>& s : sets) {
        if (kind == kWeld) {
            // (the last weld's settings, or the Frame Car's)
            edit::Weld w;
            if (!M.welds.empty()) w = M.welds.back();
            else w.radius = 0.17f, w.brk = 2500.0f, w.k = 10000.0f;
            w.anchor = s[0], w.node = s[1], w.anchor2 = -1, w.t = 0;
            M.welds.push_back(w);
            m_tie = tie_count() - 1;
        } else {
            // the last one's of the kind, or the Shell Car's
            const char k = kMountKinds[std::clamp(kind, 0, 4)];
            edit::Mount m;
            m.kind = k;
            auto last = std::find_if(M.mounts.rbegin(), M.mounts.rend(), [&](const edit::Mount& x) { return x.kind == k; });
            if (last != M.mounts.rend()) m = *last;
            else if (k == 'p') m.brk = 16000.0f;
            else if (k == 'c') m.brk = 6000.0f, m.param = 150.0f;
            else if (k == 'h') m.brk = 40000.0f, m.damp = 2.0f;
            else if (k == 'r') m.brk = 20000.0f, m.param = 1.2f;
            m.a = s[0], m.b = s[1], m.b2 = k == 'h' && s.size() > 2 ? s[2] : -1;
            M.mounts.insert(M.mounts.end(), m);
            m_tie = (int)M.mounts.size() - 1;
        }
    }
    if (sets.size() > 1) m_tie = kind == kWeld ? tie_count() - 2 : (int)M.mounts.size() - 2; // (the one clicked)
    m_status = std::string(kTieNames[std::clamp(kind, 0, kWeld)]) + " added" + (sets.size() > 1 ? " (and its twin)" : "");
}

void ModelEditor::delete_tie(int t) {
    edit::Model& M = m_model;
    if (t < 0 || t >= tie_count()) return;
    push_undo();
    const int w = tie_twin(t);
    // the higher one first: the other keeps its index
    for (int u : {std::max(t, w), std::min(t, w)}) {
        if (u < 0) continue;
        if (u < (int)M.mounts.size()) M.mounts.erase(M.mounts.begin() + u);
        else M.welds.erase(M.welds.begin() + (u - M.mounts.size()));
    }
    m_tie = std::min(t, tie_count() - 1);
    m_status = w >= 0 ? "Tie deleted (and its twin)" : "Tie deleted";
}

void ModelEditor::ui_ties() {
    edit::Model& M = m_model;
    const float row = ImGui::GetFrameHeight();
    hint("What holds the parts on the body: latches, bolts, hinges, buffers and stays (mounts: the body's node, then the part's), "
         "and the sheet's welds to the frame. Click two nodes for a new one of the kind below; click a line to edit it");
    // the kind of a new one (a swatch of its colour in the views)
    const float w3 = (ImGui::GetContentRegionAvail().x - 2 * ImGui::GetStyle().ItemSpacing.x) / 3.0f;
    for (int k = 0; k <= kWeld; k++) {
        if (k % 3) ImGui::SameLine();
        ImGui::PushID(k);
        const bool on = m_tie_kind == k;
        ImGui::PushStyleColor(ImGuiCol_Button, on ? ImVec4(kTieCols[k].x * 0.55f, kTieCols[k].y * 0.55f, kTieCols[k].z * 0.55f, 1) : ImGui::GetStyleColorVec4(ImGuiCol_Button));
        if (ImGui::Button(kTieNames[k], ImVec2(w3, 0))) m_tie_kind = k, m_picks.clear();
        ImGui::PopStyleColor();
        ImGui::SetItemTooltip("%s", kTieTips[k]);
        ImGui::PopID();
    }
    hint(m_tie_kind == kWeld ? "New: the frame's node, then the sheet's" : m_tie_kind == 2 ? "New: the body's node, the part's, then the part's second node on the hinge's line"
                                                                                         : "New: the body's node, then the part's");
    // the list, of one kind or all
    char head[48];
    snprintf(head, sizeof head, "Ties (%d)", tie_count());
    section_title(head);
    {
        static const char* filters[] = {"all kinds", "latches", "clamped bolts", "hinges", "buffers", "stays", "welds"};
        int f = m_tie_filter + 1;
        ImGui::SetNextItemWidth(-FLT_MIN);
        if (ImGui::Combo("##tfilter", &f, filters, 7)) m_tie_filter = f - 1;
    }
    std::vector<int> list;
    for (int t = 0; t < tie_count(); t++) {
        const int k = t < (int)M.mounts.size() ? mount_kind(M.mounts[t].kind) : kWeld;
        if (m_tie_filter < 0 || m_tie_filter == k) list.push_back(t);
    }
    ImGui::BeginChild("##ties", ImVec2(0, std::min(8.5f, (float)list.size() + 0.3f) * (row + ImGui::GetStyle().ItemSpacing.y) + 6), ImGuiChildFlags_Borders);
    ImGuiListClipper clip;
    clip.Begin((int)list.size());
    while (clip.Step())
        for (int r = clip.DisplayStart; r < clip.DisplayEnd; r++) {
            const int t = list[r];
            const bool mount = t < (int)M.mounts.size();
            const int k = mount ? mount_kind(M.mounts[t].kind) : kWeld;
            int n[3];
            const int c = tie_nodes(t, n);
            char label[96];
            const float brk = mount ? M.mounts[t].brk : M.welds[t - M.mounts.size()].brk;
            if (c == 3) snprintf(label, sizeof label, "%-7s %d > %d, %d", kTieNames[k], n[0], n[1], n[2]);
            else snprintf(label, sizeof label, "%-7s %d > %d", kTieNames[k], n[0], n[1]);
            ImGui::PushID(t);
            ImGui::ColorButton("##c", kTieCols[k], ImGuiColorEditFlags_NoTooltip, ImVec2(10, row));
            ImGui::SameLine(0, 6);
            if (ImGui::Selectable(label, m_tie == t, 0, ImVec2(ImGui::GetContentRegionAvail().x - 64, row))) m_tie = t;
            ImGui::SameLine();
            if (brk > 0) ImGui::TextDisabled("%6.1f kN", brk * 0.001f);
            else ImGui::TextDisabled("  never");
            ImGui::PopID();
        }
    if (list.empty()) ImGui::TextDisabled("none: click two nodes in a view");
    ImGui::EndChild();
    const int t = active_tie();
    const float bs = ImGui::GetFrameHeight();
    if (icon_button("##tdel", Icon::Trash, false, "Delete the tie (Delete; with symmetry its twin too)", bs, t >= 0)) delete_tie(t);
    ImGui::SameLine();
    ImGui::BeginDisabled(t < 0);
    if (ImGui::Button("Select its nodes", ImVec2(-FLT_MIN, 0))) {
        int n[3];
        const int c = tie_nodes(t, n);
        clear_selection();
        for (int k = 0; k < c; k++)
            if (n[k] >= 0 && n[k] < (int)M.nodes.size()) sorted_insert(m_sel, n[k]);
        frame_selection();
    }
    ImGui::EndDisabled();
    if (t < 0) return;

    // the active one's settings; with symmetry its twin gets them too
    const int tw = m_symmetry ? tie_twin(t) : -1;
    const bool mount = t < (int)M.mounts.size();
    auto edited = [&]() {
        if (!m_drag_pushed_ui) push_undo(), m_drag_pushed_ui = true;
    };
    auto done = [&]() {
        if (ImGui::IsItemDeactivated()) m_drag_pushed_ui = false;
    };
    auto force = [&](const char* id, float& v, float hi) {
        float kn = v * 0.001f;
        if (ImGui::SliderFloat(id, &kn, 0.0f, hi, kn > 0 ? "%.1f kN" : "never", ImGuiSliderFlags_Logarithmic)) edited(), v = kn * 1000.0f;
        done();
    };
    auto stiff = [&](const char* id, float& v) {
        float kn = v * 0.001f;
        if (ImGui::SliderFloat(id, &kn, 0.0f, 1000.0f, kn > 0 ? "%.0f kN/m" : "the step's", ImGuiSliderFlags_Logarithmic)) edited(), v = kn * 1000.0f;
        done();
    };
    section_title(mount ? "Mount" : "Weld");
    if (!props_begin("##tieprops")) return;
    if (mount) {
        edit::Mount& m = M.mounts[t];
        prop("Kind", kTieTips[mount_kind(m.kind)]);
        int k = mount_kind(m.kind);
        if (ImGui::Combo("##tkind", &k, kTieNames, 5)) {
            push_undo();
            m.kind = kMountKinds[k];
            if (m.kind == 'c' && m.param <= 0) m.param = 150.0f;
            if (m.kind == 'r' && m.param <= 1.0f) m.param = 1.2f;
        }
        prop("Nodes", "The body's node, then the part's (a hinge: and the part's second node on its line)");
        if (m.kind == 'h') ImGui::Text("%d > %d, %d", m.a, m.b, m.b2);
        else ImGui::Text("%d > %d", m.a, m.b);
        if (m.kind == 'h' && m.b2 < 0) {
            prop("");
            ImGui::TextColored(ImVec4(0.95f, 0.35f, 0.3f, 1), "no second node: Shift+click one");
        }
        prop("Break force", "It lets go past this force (a buffer: none), 0: never");
        force("##tbrk", m.brk, 200.0f);
        prop("Stiffness", "Of the hold (0: the most the step allows)");
        stiff("##tk", m.k);
        if (m.kind != 's' && m.kind != 'r') {
            prop("Turn damping", "N m s/rad: the part's turning about the node damped (a hinge's friction)");
            if (ImGui::SliderFloat("##tdamp", &m.damp, 0.0f, 50.0f, "%.1f", ImGuiSliderFlags_Logarithmic)) edited();
            done();
        }
        if (m.kind == 'c') {
            prop("Break moment", "N m: twisted past it (held 0.1 s) the bolt lets go");
            if (ImGui::SliderFloat("##tmom", &m.param, 1.0f, 5000.0f, "%.0f N m", ImGuiSliderFlags_Logarithmic)) edited();
            done();
        }
        if (m.kind == 'r') {
            prop("Length", "Times the distance at rest: it pulls the part's node back past it");
            if (ImGui::SliderFloat("##tlen", &m.param, 1.0f, 3.0f, "%.2f x")) edited();
            done();
        }
        if (tw >= 0 && tw < (int)M.mounts.size()) {
            edit::Mount& o = M.mounts[tw];
            o.kind = m.kind, o.brk = m.brk, o.k = m.k, o.damp = m.damp, o.param = m.param;
        }
    } else {
        edit::Weld& w = M.welds[t - M.mounts.size()];
        prop("Nodes", "The frame's node, then the sheet's");
        if (w.anchor2 >= 0) ImGui::Text("%d > %d (at %.2f of %d > %d)", w.anchor, w.node, w.t, w.anchor, w.anchor2);
        else ImGui::Text("%d > %d", w.anchor, w.node);
        prop("Radius", "The sheet's nodes within it round the weld's node share its pull");
        float cm = w.radius * 100.0f;
        if (ImGui::SliderFloat("##wrad", &cm, 1.0f, 50.0f, "%.0f cm")) edited(), w.radius = cm * 0.01f;
        done();
        prop("Break force", "It tears past this force, 0: never");
        force("##wbrk", w.brk, 100.0f);
        prop("Stiffness", "Of the hold (0: the most the step allows)");
        stiff("##wk", w.k);
        const int wt = tw - (int)M.mounts.size();
        if (wt >= 0 && wt < (int)M.welds.size()) {
            edit::Weld& o = M.welds[wt];
            o.radius = w.radius, o.brk = w.brk, o.k = w.k;
        }
    }
    prop("Twin", "With symmetry the twin (the same kind on the mirrored nodes) gets these settings too");
    if (tw >= 0) ImGui::Text("%d%s", tw, m_symmetry ? " (in step)" : "");
    else ImGui::TextDisabled(m_symmetry ? "none" : "symmetry off");
    props_end();
}

} // namespace bl
