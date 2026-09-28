// The model editor's deformation demo (Mode::Deform): the preview's nodes displaced by hand to see how the meshes bound
// to them follow (a flexbody's vertices ride on their forset nodes, a prop on its ref / x / y nodes, a wheel's tyre
// and rim on its axle). Nodes or beams are dragged with a soft falloff, a brush pushes or pulls a patch, presets
// twist, bend, crush or wave the whole model. The model itself is never changed: Reset or leaving the mode puts
// everything back (editor.h).
#include "game/editor.h"

#include "core/util.h"
#include "game/editor_internal.h"
#include "game/editor_widgets.h"
#include "game/game.h"
#include "vehicle/vehicle.h"
#include "vehicle/visual.h"

#include "imgui.h"

#include <algorithm>
#include <cmath>

namespace bl {

using namespace edit_detail;
using namespace edit_ui;

namespace {
float smooth_falloff(float d, float r) {
    if (r <= 1e-5f) return d <= 1e-5f ? 1.0f : 0.0f;
    const float x = clampf(1.0f - d / r, 0, 1);
    return x * x * (3 - 2 * x);
}
} // namespace

vec3 ModelEditor::demo_pos(int n) const {
    return to_world(m_model.nodes[n].p) + (n < (int)m_demo_off.size() ? m_demo_off[n] : vec3(0));
}

void ModelEditor::enter_deform() {
    if (m_mode == Mode::Deform) return;
    if (m_mode != Mode::Edit) end_mode();
    tool_cancel();
    m_show_gfx = true;
    // the meshes are the point here: opaque, the skeleton faint (both put back on leaving)
    m_demo_saved_alpha[0] = m_gfx_alpha, m_demo_saved_alpha[1] = m_skel_alpha;
    m_gfx_alpha = 1.0f, m_skel_alpha = 0.3f;
    m_demo_off.assign(m_model.nodes.size(), vec3(0));
    m_demo_undo.clear();
    m_demo_wave = false;
    m_demo_drag = false;
    m_mode = Mode::Deform;
    m_status = "Deformation demo: drag nodes or beams, the meshes follow (the model is not changed)";
}

void ModelEditor::demo_push_undo() {
    m_demo_undo.push_back(m_demo_off);
    if (m_demo_undo.size() > 64) m_demo_undo.erase(m_demo_undo.begin());
}

void ModelEditor::demo_reset() {
    demo_push_undo();
    m_demo_off.assign(m_model.nodes.size(), vec3(0));
    m_demo_wave = false;
}

void ModelEditor::apply_demo_to_preview() {
    Vehicle* v = m_preview;
    if (!v || !alive(v)) return;
    phys::SoftBody& b = *v->body;
    const int ne = std::min((int)m_model.nodes.size(), (int)b.nodes.size());
    m_demo_off.resize(m_model.nodes.size(), vec3(0));
    bool moved = false;
    for (int i = 0; i < ne; i++) {
        const vec3 p = demo_pos(i);
        if (length2(b.nodes[i].p - p) > 1e-10f) v->set_node_position(i, p), moved = true;
    }
    // a wheel's own nodes (its tyre, its rim) follow its axle
    if (m_preview_rest.size() == b.nodes.size())
        for (const phys::Wheel& w : b.wheels) {
            if ((int)w.axle0 >= ne || (int)w.axle1 >= ne) continue;
            const vec3 o = (m_demo_off[w.axle0] + m_demo_off[w.axle1]) * 0.5f;
            auto put = [&](uint32_t n) {
                if (n >= b.nodes.size()) return;
                const vec3 p = m_preview_rest[n] + o;
                if (length2(b.nodes[n].p - p) > 1e-10f) v->set_node_position((int)n, p), moved = true;
            };
            for (uint32_t n : w.nodes) put(n);
            for (uint32_t n : w.rim) put(n);
        }
    if (moved) b.compute_aabb();
}

void ModelEditor::deform_preset(int kind) {
    // from the rest shape, along the model's length (x) and height (y); the amount 0..1
    const edit::Model& M = m_model;
    const int N = (int)M.nodes.size();
    if (!N) return;
    demo_push_undo();
    m_demo_off.assign(N, vec3(0));
    vec3 mn(1e30f), mx(-1e30f);
    for (const edit::Node& n : M.nodes) mn = vmin(mn, n.p), mx = vmax(mx, n.p);
    const vec3 c = (mn + mx) * 0.5f, ext = vmax(mx - mn, vec3(1e-3f));
    const float a = m_demo_amount;
    switch (kind) {
    case 0: // twist about the length axis: the front turned one way, the back the other
        for (int i = 0; i < N; i++) {
            const vec3 p = M.nodes[i].p;
            const float t = (p.x - c.x) / ext.x; // -0.5 .. 0.5
            const float ang = t * a * 1.2f;
            const float y = p.y - c.y, z = p.z - c.z;
            const vec3 q(p.x, c.y + y * std::cos(ang) - z * std::sin(ang), c.z + y * std::sin(ang) + z * std::cos(ang));
            m_demo_off[i] = q - p;
        }
        break;
    case 1: // bend: the ends up, the middle down (a sagging frame)
        for (int i = 0; i < N; i++) {
            const float t = (M.nodes[i].p.x - c.x) / (ext.x * 0.5f);
            m_demo_off[i] = vec3(0, (t * t - 0.33f) * a * ext.x * 0.12f, 0);
        }
        break;
    case 2: { // a frontal crash: the front third pushed back, the most at the bumper, bulging out a little
        const float depth = ext.x * 0.35f, push = a * ext.x * 0.18f;
        for (int i = 0; i < N; i++) {
            const vec3 p = M.nodes[i].p;
            const float f = clampf(1.0f - (p.x - mn.x) / depth, 0, 1); // (the front is -x)
            if (f <= 0) continue;
            const float s = f * f;
            m_demo_off[i] = vec3(push * s, (p.y - c.y) / ext.y * push * 0.25f * s, (p.z - c.z) / ext.z * push * 0.35f * s);
        }
        break;
    }
    case 3: { // dents: a few pushes toward the inside at random places on the outside
        uint32_t seed = 0x9e3779b9u + (uint32_t)m_demo_undo.size() * 7919u;
        auto rnd = [&]() {
            seed = seed * 1664525u + 1013904223u;
            return (float)(seed >> 8) / 16777216.0f;
        };
        for (int k = 0; k < 5; k++) {
            const int n0 = std::min(N - 1, (int)(rnd() * N));
            const vec3 p0 = M.nodes[n0].p, in = normalize_or(c - p0, vec3(0, -1, 0));
            const float r = std::max(0.15f, ext.x * 0.12f), depth = a * r * 0.8f;
            for (int i = 0; i < N; i++) m_demo_off[i] += in * (depth * smooth_falloff(length(M.nodes[i].p - p0), r));
        }
        break;
    }
    default: break;
    }
}

void ModelEditor::deform_input(float dt) {
    ImGuiIO& io = ImGui::GetIO();
    const edit::Model& M = m_model;
    const int N = (int)M.nodes.size();
    m_demo_off.resize(N, vec3(0));
    m_demo_time += dt;
    // the wave: animated, from the rest shape
    if (m_demo_wave) {
        vec3 mn(1e30f), mx(-1e30f);
        for (const edit::Node& n : M.nodes) mn = vmin(mn, n.p), mx = vmax(mx, n.p);
        const float L = std::max(0.5f, mx.x - mn.x), A = m_demo_amount * L * 0.05f;
        for (int i = 0; i < N; i++) {
            const vec3 p = M.nodes[i].p;
            const float ph = (p.x - mn.x) / L * 2 * kPi * 1.5f - m_demo_time * 4.0f;
            m_demo_off[i] = vec3(0, A * std::sin(ph), A * 0.5f * std::sin(ph * 0.5f + p.y));
        }
    }
    // keys: Esc back to editing, R resets, Ctrl+Z steps back
    if (!io.WantTextInput) {
        if (ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
            end_mode();
            return;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_R, false) && !(io.KeyCtrl || io.KeySuper)) demo_reset();
        if (ImGui::IsKeyPressed(ImGuiKey_Z, false) && (io.KeyCtrl || io.KeySuper) && !m_demo_undo.empty()) {
            m_demo_off = m_demo_undo.back();
            m_demo_undo.pop_back();
            m_demo_wave = false;
        }
    }
    const int vi = m_active_view;
    const bool in_view = !io.WantCaptureMouse && view_at(m_mouse) >= 0 && !m_split_hover;
    vec3 ro, rd;
    mouse_ray(vi, ro, rd);
    // what is under the mouse: a node, else a beam (their displaced places)
    m_demo_hover_node = m_demo_hover_beam = -1;
    m_demo_brush_ok = false;
    if (in_view && !m_demo_drag) {
        // (a node hidden behind the meshes is passed over for one in front: the surface under the mouse)
        float mesh_t = 1e30f;
        if (m_show_gfx && m_gfx_alpha > 0.5f && m_preview && alive(m_preview) && m_preview->visual()) m_preview->visual()->pick(ro, rd, &mesh_t);
        float bd = 12.0f * 12.0f, bd_hidden = bd;
        int hidden_best = -1;
        for (int i = 0; i < N; i++) {
            if (!node_shown(i)) continue;
            vec2 s;
            const vec3 p = demo_pos(i);
            if (!project(vi, p, s)) continue;
            const float d = (s.x - m_mouse.x) * (s.x - m_mouse.x) + (s.y - m_mouse.y) * (s.y - m_mouse.y);
            if (dot(p - ro, rd) > mesh_t + 0.12f) {
                if (d < bd_hidden) bd_hidden = d, hidden_best = i;
                continue;
            }
            if (d < bd) bd = d, m_demo_hover_node = i;
        }
        if (m_demo_hover_node < 0 && mesh_t >= 1e29f) m_demo_hover_node = hidden_best;
        if (m_demo_hover_node < 0 && m_demo_tool == 0) {
            bd = 6.0f * 6.0f;
            for (int i = 0; i < (int)M.beams.size(); i++) {
                if (!elem_shown(Elem::Beam, i)) continue;
                vec2 a, b;
                if (!project(vi, demo_pos(M.beams[i].a), a) || !project(vi, demo_pos(M.beams[i].b), b)) continue;
                const float d = seg_dist2(m_mouse, a, b);
                if (d < bd) bd = d, m_demo_hover_beam = i;
            }
        }
        // the brush: where the mouse ray meets the meshes (else the nearest node's depth)
        if (m_demo_tool != 0) {
            float t = 1e30f;
            if (m_preview && alive(m_preview) && m_preview->visual() && m_preview->visual()->pick(ro, rd, &t) >= 0 && t < 1e29f) {
                m_demo_brush = ro + rd * t, m_demo_brush_ok = true;
            } else if (m_demo_hover_node >= 0) {
                m_demo_brush = demo_pos(m_demo_hover_node), m_demo_brush_ok = true;
            }
        }
    }
    const bool click = in_view && ImGui::IsMouseClicked(ImGuiMouseButton_Left);
    const bool down = ImGui::IsMouseDown(ImGuiMouseButton_Left);
    if (m_demo_tool == 0) {
        // ---- drag: the grabbed node (or a beam's two) follows the mouse in the view plane, the nodes around with
        // the falloff (measured on the rest shape)
        if (click && (m_demo_hover_node >= 0 || m_demo_hover_beam >= 0)) {
            demo_push_undo();
            m_demo_wave = false;
            m_demo_grab.clear();
            if (m_demo_hover_node >= 0) m_demo_grab = {m_demo_hover_node};
            else m_demo_grab = {M.beams[m_demo_hover_beam].a, M.beams[m_demo_hover_beam].b};
            vec3 a(0);
            for (int n : m_demo_grab) a += demo_pos(n);
            m_demo_anchor = a * (1.0f / (float)m_demo_grab.size());
            m_demo_plane_n = m_views[vi].cam.forward();
            m_demo_off0 = m_demo_off;
            m_demo_w.assign(N, 0.0f);
            for (int i = 0; i < N; i++) {
                float d = 1e30f;
                for (int g : m_demo_grab) d = std::min(d, length(M.nodes[i].p - M.nodes[g].p));
                m_demo_w[i] = smooth_falloff(d, m_demo_radius);
            }
            for (int g : m_demo_grab) m_demo_w[g] = 1.0f;
            m_demo_drag = true;
            m_demo_drag_view = vi;
            if (getenv("BL_EDITOR_LOG")) printf("editor: demo drag of %zu node(s) from %d, %d within the falloff\n", m_demo_grab.size(), m_demo_grab[0],
                                                (int)std::count_if(m_demo_w.begin(), m_demo_w.end(), [](float w) { return w > 0; }));
        }
        if (m_demo_drag) {
            vec3 dro, drd, hit;
            mouse_ray(m_demo_drag_view, dro, drd);
            if (ray_plane(dro, drd, m_demo_plane_n, dot(m_demo_plane_n, m_demo_anchor), hit)) {
                const vec3 delta = hit - m_demo_anchor;
                for (int i = 0; i < N; i++)
                    if (m_demo_w[i] > 0) m_demo_off[i] = m_demo_off0[i] + delta * m_demo_w[i];
            }
            if (!down) m_demo_drag = false;
        }
    } else {
        // ---- the brush: held down, it pushes the nodes under it away from the view (1) or pulls them toward it (2);
        // Shift the other way
        if (click) demo_push_undo(), m_demo_wave = false;
        if (down && in_view && m_demo_brush_ok) {
            const bool pull = (m_demo_tool == 2) != io.KeyShift;
            const vec3 dir = m_views[vi].cam.forward() * (pull ? -1.0f : 1.0f);
            const float r = std::max(0.02f, m_demo_radius), speed = m_demo_strength * r * 1.5f;
            for (int i = 0; i < N; i++) {
                const float w = smooth_falloff(length(demo_pos(i) - m_demo_brush), r);
                if (w > 0) m_demo_off[i] += dir * (speed * w * dt);
            }
        }
    }
}

void ModelEditor::draw_deform(Renderer& r, int view) {
    const View& vw = m_views[view];
    const ViewFrame vf = view_frame(vw.cam, vw.rh);
    const edit::Model& M = m_model;
    const float skel = m_skel_alpha;
    // the rest shape, faint; the displaced one; the nodes coloured by how far they moved
    if (m_demo_rest_shape)
        for (int i = 0; i < (int)M.beams.size(); i++)
            if (elem_shown(Elem::Beam, i)) r.line(to_world(M.nodes[M.beams[i].a].p), to_world(M.nodes[M.beams[i].b].p), Renderer::rgba(1, 1, 1, 0.12f));
    float maxd = 1e-4f;
    for (const vec3& o : m_demo_off) maxd = std::max(maxd, length(o));
    for (int i = 0; i < (int)M.beams.size(); i++) {
        if (!elem_shown(Elem::Beam, i)) continue;
        const bool hi = i == m_demo_hover_beam || (m_demo_drag && m_demo_grab.size() == 2 && ((m_demo_grab[0] == M.beams[i].a && m_demo_grab[1] == M.beams[i].b)));
        if (!hi && skel < 0.02f) continue;
        const uint32_t c = hi ? kHoverCol : Renderer::rgba(0.85f, 0.87f, 0.9f, 0.8f * skel);
        r.thick_line(demo_pos(M.beams[i].a), demo_pos(M.beams[i].b), c, hi ? m_beam_px + 2.0f : m_beam_px);
    }
    for (int i = 0; i < (int)M.nodes.size(); i++) {
        if (!node_shown(i)) continue;
        const float d = i < (int)m_demo_off.size() ? length(m_demo_off[i]) / maxd : 0.0f;
        // (still: grey; moved: toward orange)
        if (d <= 0.01f && skel < 0.02f && i != m_demo_hover_node) continue;
        const uint32_t c = d > 0.01f ? Renderer::rgba(0.7f + 0.3f * d, 0.7f - 0.1f * d, 0.7f - 0.55f * d, 0.35f + 0.65f * d) : Renderer::rgba(0.7f, 0.72f, 0.75f, skel);
        r.point(demo_pos(i), c);
    }
    // the falloff: a ring of the radius around the node under the mouse (or the brush)
    auto sphere_ring = [&](vec3 p, float rad, uint32_t c) {
        const int n = 48;
        for (int k = 0; k < n; k++) {
            const float t0 = 2 * kPi * k / n, t1 = 2 * kPi * (k + 1) / n;
            r.thick_line(p + (vf.right * std::cos(t0) + vf.up * std::sin(t0)) * rad, p + (vf.right * std::cos(t1) + vf.up * std::sin(t1)) * rad, c, 1.5f);
        }
    };
    if (m_demo_tool == 0) {
        const int n = m_demo_drag ? (m_demo_grab.empty() ? -1 : m_demo_grab[0]) : m_demo_hover_node;
        if (n >= 0) {
            ring(r, vf, demo_pos(n), kHoverCol, m_node_px * 0.5f + 4.0f, 14, 2.0f);
            if (m_demo_radius > 0.001f) sphere_ring(demo_pos(n), m_demo_radius, Renderer::rgba(1, 0.75f, 0.3f, 0.7f));
        }
        if (m_demo_drag) thick_line(r, vf, m_demo_anchor, demo_pos(m_demo_grab[0]), Renderer::rgba(1, 0.8f, 0.3f, 0.9f), 1.5f);
    } else if (m_demo_brush_ok) {
        sphere_ring(m_demo_brush, std::max(0.02f, m_demo_radius), m_demo_tool == 2 ? Renderer::rgba(0.4f, 0.8f, 1, 0.9f) : Renderer::rgba(1, 0.55f, 0.3f, 0.9f));
    }
}

void ModelEditor::ui_deform_panel() {
    // (the left panel in the deformation demo)
    ImGui::PushTextWrapPos(0);
    ImGui::TextDisabled("Move nodes or beams and watch the meshes follow them. The model is not changed.");
    ImGui::PopTextWrapPos();
    ImGui::Spacing();
    const float bs = 34.0f;
    if (icon_button("##dtool0", Icon::Move, m_demo_tool == 0, "Drag: a node or a beam follows the mouse in the view plane, the nodes around it with the falloff", bs)) m_demo_tool = 0;
    ImGui::SameLine();
    if (icon_button("##dtool1", Icon::PushPull, m_demo_tool == 1, "Push: hold the button over the meshes to press a dent in (Shift: pull)", bs)) m_demo_tool = 1;
    ImGui::SameLine();
    if (icon_button("##dtool2", Icon::Grow, m_demo_tool == 2, "Pull: hold the button to pull a bump out (Shift: push)", bs)) m_demo_tool = 2;
    ImGui::SameLine();
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted(m_demo_tool == 0 ? "Drag" : m_demo_tool == 1 ? "Push" : "Pull");
    if (props_begin("##deformprops")) {
        prop("Falloff", "How far around the moved node the others follow (0: that node alone)");
        ImGui::SliderFloat("##radius", &m_demo_radius, 0.0f, 2.0f, m_demo_radius < 0.005f ? "only the node" : "%.2f m");
        if (m_demo_tool != 0) {
            prop("Strength");
            ImGui::SliderFloat("##strength", &m_demo_strength, 0.05f, 2.0f, "%.2f");
        }
        props_end();
    }
    section_title("Presets");
    if (props_begin("##presetprops")) {
        prop("Amount");
        ImGui::SliderFloat("##amount", &m_demo_amount, 0.0f, 1.0f, "%.2f");
        props_end();
    }
    const float w = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
    if (ImGui::Button("Twist", ImVec2(w, 0))) deform_preset(0);
    ImGui::SameLine();
    if (ImGui::Button("Bend", ImVec2(w, 0))) deform_preset(1);
    if (ImGui::Button("Front crash", ImVec2(w, 0))) deform_preset(2);
    ImGui::SameLine();
    if (ImGui::Button("Dents", ImVec2(w, 0))) deform_preset(3);
    if (ImGui::Button(m_demo_wave ? "Stop the wave" : "Wave (animated)", ImVec2(-1, 0))) {
        if (!m_demo_wave) demo_push_undo();
        m_demo_wave = !m_demo_wave;
        if (!m_demo_wave) m_demo_off.assign(m_model.nodes.size(), vec3(0));
    }
    section_title("Show");
    ImGui::Checkbox("the rest shape", &m_demo_rest_shape);
    ImGui::SetNextItemWidth(-1);
    ImGui::SliderFloat("##gfxalpha", &m_gfx_alpha, 0.05f, 1.0f, "meshes %.2f");
    ImGui::SetNextItemWidth(-1);
    ImGui::SliderFloat("##skelalpha", &m_skel_alpha, 0.0f, 1.0f, "beams %.2f");
    ImGui::Spacing();
    ImGui::BeginDisabled(m_demo_undo.empty());
    if (ImGui::Button("Undo (Ctrl+Z)", ImVec2(w, 0)) && !m_demo_undo.empty()) {
        m_demo_off = m_demo_undo.back();
        m_demo_undo.pop_back();
        m_demo_wave = false;
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Reset (R)", ImVec2(w, 0))) demo_reset();
    ImGui::Spacing();
    if (ImGui::Button("Back to editing (Esc)", ImVec2(-1, 0))) end_mode();
}

} // namespace bl
