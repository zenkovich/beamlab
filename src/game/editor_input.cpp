// The model editor's input: the views and their cameras, picking with inference, the tools, the typed values,
// the hotkeys and the physics test's mouse (editor.h).
#include "game/editor.h"

#include "core/util.h"
#include "game/editor_internal.h"
#include "game/game.h"
#include "vehicle/vehicle.h"

#include "imgui.h"

#include <GLFW/glfw3.h>
#include <algorithm>
#include <cmath>
#include <cstring>

namespace bl {

using namespace edit_detail;

namespace {

constexpr float kNodePx = 10.0f, kMidPx = 8.0f, kEdgePx = 6.0f, kAxisPx = 9.0f;
const vec3 kAxes[3] = {vec3(1, 0, 0), vec3(0, 1, 0), vec3(0, 0, 1)};
// quad: the top view top left, the front view top right, the side view bottom left, the perspective bottom right
const int kQuadSlot[4] = {3, 1, 2, 0};

float d2(vec2 a, vec2 b) { return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y); }
bool key(ImGuiKey k) { return ImGui::IsKeyDown(k); }

} // namespace

// ------------------------------------------------------------------------------------------------ views
void ModelEditor::layout(float W, float H) {
    // the work area: the window below the main menu, between the panels
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    const float x0 = vp->WorkPos.x, y0 = vp->WorkPos.y;
    W = vp->WorkSize.x, H = vp->WorkSize.y;
    m_left_w = clampf(m_left_w, 220.0f, std::max(220.0f, W * 0.4f));
    m_right_w = clampf(m_right_w, 260.0f, std::max(260.0f, W * 0.45f));
    const float x = x0 + m_left_w, y = y0 + m_top_h, w = std::max(80.0f, W - m_left_w - m_right_w), h = std::max(80.0f, H - m_top_h - m_status_h);
    m_area[0] = x, m_area[1] = y, m_area[2] = w, m_area[3] = h;
    for (View& v : m_views) v.shown = false;
    auto put = [&](int i, float rx, float ry, float rw, float rh) {
        View& v = m_views[i];
        v.rx = rx, v.ry = ry, v.rw = std::max(20.0f, rw), v.rh = std::max(20.0f, rh);
        v.shown = true;
    };
    if (!m_quad) {
        put(m_single, x, y, w, h);
        return;
    }
    if (m_maximized >= 0) {
        put(m_maximized, x, y, w, h);
        return;
    }
    const float sx = x + w * m_split_x, sy = y + h * m_split_y, g = 1.0f;
    put(kQuadSlot[0], x, y, sx - x - g, sy - y - g);
    put(kQuadSlot[1], sx + g, y, x + w - sx - g, sy - y - g);
    put(kQuadSlot[2], x, sy + g, sx - x - g, y + h - sy - g);
    put(kQuadSlot[3], sx + g, sy + g, x + w - sx - g, y + h - sy - g);
}

void ModelEditor::update_cameras() {
    for (View& v : m_views) {
        Camera& c = v.cam;
        const vec3 t = to_world(v.target);
        c.target = t;
        if (v.ortho == 0) {
            v.pitch = clampf(v.pitch, -1.55f, 1.55f);
            const vec3 dir(std::cos(v.pitch) * std::sin(v.yaw), std::sin(v.pitch), std::cos(v.pitch) * std::cos(v.yaw));
            c.pos = t + dir * v.dist;
            c.up = vec3(0, 1, 0);
            c.fov_deg = 50.0f;
            c.ortho_half = 0;
            c.znear = clampf(v.dist * 0.01f, 0.02f, 0.5f);
            c.zfar = 3000.0f;
        } else {
            const vec3 fwd = v.ortho == 1 ? vec3(1, 0, 0) : v.ortho == 2 ? vec3(0, 0, -1) : vec3(0, -1, 0);
            c.up = v.ortho == 3 ? vec3(-1, 0, 0) : vec3(0, 1, 0);
            c.pos = t - fwd * 300.0f;
            c.fov_deg = 50.0f;
            c.ortho_half = std::max(0.05f, v.dist * 0.5f);
            c.znear = 1.0f;
            c.zfar = 1200.0f;
        }
        c.update(v.rw / std::max(1.0f, v.rh));
    }
}

int ModelEditor::view_at(vec2 m) const {
    for (int i = 0; i < 4; i++) {
        const View& v = m_views[i];
        if (v.shown && m.x >= v.rx && m.x < v.rx + v.rw && m.y >= v.ry && m.y < v.ry + v.rh) return i;
    }
    return -1;
}

bool ModelEditor::project(int view, vec3 world, vec2& s) const {
    const View& v = m_views[view];
    const vec4 c = v.cam.viewproj * vec4(world, 1.0f);
    if (c.w <= 1e-4f) return false;
    s = vec2(v.rx + (c.x / c.w * 0.5f + 0.5f) * v.rw, v.ry + (0.5f - c.y / c.w * 0.5f) * v.rh);
    return true;
}

void ModelEditor::mouse_ray(int view, vec3& ro, vec3& rd) const {
    const View& v = m_views[view];
    const vec2 ndc((m_mouse.x - v.rx) / std::max(1.0f, v.rw) * 2 - 1, 1 - (m_mouse.y - v.ry) / std::max(1.0f, v.rh) * 2);
    const mat4 inv = inverse(v.cam.viewproj);
    const vec4 a = inv * vec4(ndc.x, ndc.y, -1, 1), b = inv * vec4(ndc.x, ndc.y, 1, 1);
    ro = a.xyz() / a.w;
    rd = normalize(b.xyz() / b.w - ro);
}

void ModelEditor::pan_view(View& v, vec2 md) {
    const float px = v.ortho ? 2.0f * v.cam.ortho_half / std::max(1.0f, v.rh) : 2.0f * std::tan(v.cam.fov_deg * kDeg2Rad * 0.5f) * v.dist / std::max(1.0f, v.rh);
    const vec3 fwd = v.cam.forward(), right = normalize_or(cross(fwd, v.cam.up), vec3(1, 0, 0)), up = cross(right, fwd);
    v.target = v.target - right * (md.x * px) + up * (md.y * px);
}

void ModelEditor::camera_input(float dt) {
    ImGuiIO& io = ImGui::GetIO();
    const bool over_ui = io.WantCaptureMouse;
    const int hv = over_ui ? -1 : view_at(m_mouse);
    const vec2 md(m_mouse.x - m_prev_mouse.x, m_mouse.y - m_prev_mouse.y);
    m_rclick = false;
    // the splitters of the quad views: drag to resize, double click to even them out
    const bool splitting = m_drag == Drag::SplitX || m_drag == Drag::SplitY || m_drag == Drag::SplitXY;
    m_split_hover = 0;
    if (m_quad && m_maximized < 0 && (m_drag == Drag::None || splitting) && !over_ui) {
        const float sx = m_area[0] + m_area[2] * m_split_x, sy = m_area[1] + m_area[3] * m_split_y;
        const bool in_area = m_mouse.x >= m_area[0] && m_mouse.x < m_area[0] + m_area[2] && m_mouse.y >= m_area[1] && m_mouse.y < m_area[1] + m_area[3];
        const bool nx = in_area && std::fabs(m_mouse.x - sx) < 5, ny = in_area && std::fabs(m_mouse.y - sy) < 5;
        m_split_hover = (nx ? 1 : 0) | (ny ? 2 : 0);
        if (m_split_hover && m_drag == Drag::None) {
            if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) m_split_x = m_split_y = 0.5f;
            else if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) m_drag = nx && ny ? Drag::SplitXY : nx ? Drag::SplitX : Drag::SplitY;
        }
    }
    if (m_drag == Drag::SplitX || m_drag == Drag::SplitY || m_drag == Drag::SplitXY) {
        if (m_drag != Drag::SplitY) m_split_x = clampf((m_mouse.x - m_area[0]) / m_area[2], 0.12f, 0.88f);
        if (m_drag != Drag::SplitX) m_split_y = clampf((m_mouse.y - m_area[1]) / m_area[3], 0.12f, 0.88f);
        m_split_hover = m_drag == Drag::SplitXY ? 3 : m_drag == Drag::SplitX ? 1 : 2;
        if (!ImGui::IsMouseDown(ImGuiMouseButton_Left)) m_drag = Drag::None;
    }
    if (m_split_hover) ImGui::SetMouseCursor(m_split_hover == 3 ? ImGuiMouseCursor_ResizeAll : m_split_hover == 1 ? ImGuiMouseCursor_ResizeEW : ImGuiMouseCursor_ResizeNS);

    // right drag orbits (the 3D view) or pans (the flat views), middle drag pans
    if (m_drag == Drag::None && hv >= 0) {
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Right)) m_drag = Drag::Orbit, m_drag_view = hv;
        else if (ImGui::IsMouseClicked(ImGuiMouseButton_Middle)) m_drag = Drag::Pan, m_drag_view = hv;
    }
    if (m_drag == Drag::Orbit) {
        View& v = m_views[m_drag_view];
        if (v.ortho == 0) {
            v.yaw -= md.x * 0.006f;
            v.pitch = clampf(v.pitch + md.y * 0.006f, -1.55f, 1.55f);
        } else {
            pan_view(v, md);
        }
        if (!ImGui::IsMouseDown(ImGuiMouseButton_Right)) {
            if (io.MouseDragMaxDistanceSqr[1] < 16.0f) m_rclick = true; // (a click, not a drag)
            m_drag = Drag::None;
        }
    } else if (m_drag == Drag::Pan) {
        pan_view(m_views[m_drag_view], md);
        if (!ImGui::IsMouseDown(ImGuiMouseButton_Middle)) m_drag = Drag::None;
    }
    // the wheel zooms toward the point under the mouse (while a node is pulled in the physics test: the pull's strength)
    if (hv >= 0 && io.MouseWheel != 0 && !(m_mode == Mode::Physics && m_drag == Drag::Grab)) {
        View& z = m_views[hv];
        const float f = std::pow(0.87f, io.MouseWheel);
        vec3 ro, rd, q;
        mouse_ray(hv, ro, rd);
        const vec3 fwd = z.cam.forward();
        if (ray_plane(ro, rd, fwd, dot(fwd, to_world(z.target)), q)) z.target = z.target + (to_model(q) - z.target) * (1.0f - f);
        z.dist = clampf(z.dist * f, 0.05f, 3000.0f);
    }
    // WASD: fly (the 3D view; Q / E down / up while the right button is held) or pan (the flat views)
    if (!io.WantTextInput && !(io.KeyCtrl || io.KeySuper) && !io.KeyAlt) {
        const int vi = m_drag == Drag::Orbit || m_drag == Drag::Pan ? m_drag_view : hv >= 0 ? hv : m_active_view;
        const float fx = (float)key(ImGuiKey_D) - (float)key(ImGuiKey_A), fz = (float)key(ImGuiKey_W) - (float)key(ImGuiKey_S);
        const float fy = ImGui::IsMouseDown(ImGuiMouseButton_Right) ? (float)key(ImGuiKey_E) - (float)key(ImGuiKey_Q) : 0.0f;
        if (vi >= 0 && (fx != 0 || fy != 0 || fz != 0)) {
            View& w = m_views[vi];
            const float speed = (w.ortho ? w.dist * 0.8f : std::max(1.5f, w.dist * 0.8f)) * (io.KeyShift ? 3.0f : 1.0f);
            const vec3 fwd = w.cam.forward(), right = normalize_or(cross(fwd, w.cam.up), vec3(1, 0, 0)), up = cross(right, fwd);
            if (w.ortho) w.target += (right * fx + up * fz) * (speed * dt);
            else w.target += (fwd * fz + right * fx + vec3(0, 1, 0) * fy) * (speed * dt);
        }
    }
}

// ------------------------------------------------------------------------------------------------ picking
int ModelEditor::gizmo_axis_hit(vec2 m) const {
    if (m_tool != Tool::Select || m_mode != Mode::Edit) return -1;
    const std::vector<int> nodes = selection_nodes_all();
    if (nodes.empty()) return -1;
    const vec3 c = gizmo_center();
    const View& v = m_views[m_active_view];
    const ViewFrame vf = view_frame(v.cam, v.rh);
    const float L = vf.px_at(to_world(c)) * 70.0f;
    vec2 o;
    if (!project(m_active_view, to_world(c), o)) return -1;
    int best = -1;
    float bd = 8.0f * 8.0f;
    for (int a = 0; a < 3; a++) {
        vec2 s;
        if (!project(m_active_view, to_world(c + kAxes[a] * L), s) || d2(o, s) < 64.0f) continue;
        const float d = seg_dist2(m, o, s);
        if (d < bd) bd = d, best = a;
    }
    return best;
}

vec3 ModelEditor::gizmo_center() const {
    const std::vector<int> nodes = selection_nodes_all();
    vec3 c(0);
    for (int n : nodes) c += m_model.nodes[n].p;
    return nodes.empty() ? c : c * (1.0f / (float)nodes.size());
}

bool ModelEditor::joint_hover(int& beam, int& end) const {
    // the frame element under the mouse (within 14 px of it on screen) and its end nearer to the mouse
    beam = end = -1;
    if (ImGui::GetIO().WantCaptureMouse || view_at(m_mouse) < 0) return false;
    const edit::Model& M = m_model;
    float best = 14.0f * 14.0f;
    for (int i = 0; i < (int)M.beams.size(); i++) {
        const edit::Beam& b = M.beams[i];
        if (!M.groups[std::clamp(b.group, 0, (int)M.groups.size() - 1)].is_frame() || !elem_pickable(Elem::Beam, i)) continue;
        vec2 sa, sb;
        if (!project(m_active_view, to_world(M.nodes[b.a].p), sa) || !project(m_active_view, to_world(M.nodes[b.b].p), sb)) continue;
        const vec2 d = sb - sa;
        const float l2 = dot(d, d);
        const float t = l2 > 1e-6f ? std::clamp(dot(m_mouse - sa, d) / l2, 0.0f, 1.0f) : 0.0f;
        const vec2 q = sa + d * t - m_mouse;
        if (dot(q, q) < best) best = dot(q, q), beam = i, end = t < 0.5f ? 0 : 1;
    }
    return beam >= 0;
}

void ModelEditor::update_hover() {
    m_hover_node = -1;
    m_hover_kind = Elem::None;
    m_hover_elem = -1;
    m_hover_axis = -1;
    m_hover_tri = -1;
    if (ImGui::GetIO().WantCaptureMouse || view_at(m_mouse) < 0 || m_split_hover) return;
    if (m_drag != Drag::None && m_drag != Drag::Erase) return;
    const int vi = m_active_view;
    const edit::Model& M = m_model;
    m_hover_axis = gizmo_axis_hit(m_mouse);
    if (m_hover_axis >= 0) return;
    const bool pick_move = m_tool == Tool::Move && !m_op;
    const bool elems = m_tool == Tool::Select || m_tool == Tool::Erase || pick_move;
    const bool node_pick = m_gfx_pick >= 1 && m_gfx_pick <= 5;
    const bool nodes = elems || node_pick;
    if (nodes) {
        float bd = kNodePx * kNodePx;
        for (int i = 0; i < (int)M.nodes.size(); i++) {
            if (!node_pickable(i)) continue;
            vec2 s;
            if (!project(vi, to_world(M.nodes[i].p), s)) continue;
            const float d = d2(s, m_mouse);
            if (d < bd) bd = d, m_hover_node = i;
        }
        if (m_hover_node >= 0 || node_pick) return;
    }
    if (elems) {
        float bd = kEdgePx * kEdgePx;
        auto seg = [&](int a, int b, Elem kind, int idx) {
            if (!elem_pickable(kind, idx)) return;
            vec2 s0, s1;
            if (!project(vi, to_world(M.nodes[a].p), s0) || !project(vi, to_world(M.nodes[b].p), s1)) return;
            const float d = seg_dist2(m_mouse, s0, s1);
            if (d < bd) bd = d, m_hover_kind = kind, m_hover_elem = idx;
        };
        for (int i = 0; i < (int)M.beams.size(); i++) seg(M.beams[i].a, M.beams[i].b, Elem::Beam, i);
        for (int i = 0; i < (int)M.shocks.size(); i++) seg(M.shocks[i].a, M.shocks[i].b, Elem::Shock, i);
        for (int i = 0; i < (int)M.hydros.size(); i++) seg(M.hydros[i].a, M.hydros[i].b, Elem::Hydro, i);
        for (int i = 0; i < (int)M.joints.size(); i++) seg(M.joints[i].parent, M.joints[i].child, Elem::Joint, i);
        for (int i = 0; i < (int)M.wheels.size(); i++) seg(M.wheels[i].n1, M.wheels[i].n2, Elem::Wheel, i);
        if (m_hover_kind != Elem::None) return;
    }
    if (elems || m_tool == Tool::PushPull) {
        vec3 ro, rd;
        mouse_ray(vi, ro, rd);
        float bt = 1e30f;
        for (int i = 0; i < (int)M.tris.size(); i++) {
            if (!elem_pickable(Elem::Tri, i)) continue;
            const edit::Tri& t = M.tris[i];
            float tt;
            if (ray_tri(ro, rd, to_world(M.nodes[t.a].p), to_world(M.nodes[t.b].p), to_world(M.nodes[t.c].p), tt) && tt < bt) bt = tt, m_hover_tri = i;
        }
        if (m_hover_tri >= 0 && elems) m_hover_kind = Elem::Tri, m_hover_elem = m_hover_tri;
    }
}

vec3 ModelEditor::snap(vec3 p) const {
    if (!m_snap || m_snap_size <= 0) return p;
    return vec3(std::round(p.x / m_snap_size), std::round(p.y / m_snap_size), std::round(p.z / m_snap_size)) * m_snap_size;
}

vec3 ModelEditor::plane_point(int view, bool has_anchor, vec3 anchor) const {
    // what the drawing plane of a view goes through: the anchor; else the 3D view's work height, the flat views'
    // depth of the selected node (or the work height for the top view, the view's centre for the others)
    if (has_anchor) return anchor;
    const View& v = m_views[view];
    if (!m_sel.empty() && v.ortho) return m_model.nodes[m_sel.back()].p;
    if (v.ortho == 1 || v.ortho == 2) return v.target;
    return vec3(0, m_work_y, 0);
}

ModelEditor::Pick ModelEditor::pick_point(bool has_anchor, vec3 anchor, bool nodes) {
    Pick pk;
    const int vi = m_active_view;
    const View& v = m_views[vi];
    if (!v.shown) return pk;
    vec3 ro, rd;
    mouse_ray(vi, ro, rd);
    const vec3 mro = to_model(ro);
    const edit::Model& M = m_model;
    const bool op_moving = (m_op && (m_tool == Tool::Move || m_tool == Tool::Rotate || m_tool == Tool::Scale)) || m_drag == Drag::MoveFree || m_drag == Drag::MoveAxis;
    auto moving = [&](int n) { return op_moving && std::binary_search(m_op_nodes.begin(), m_op_nodes.end(), n); };
    // 1. a node
    if (nodes) {
        float bd = kNodePx * kNodePx;
        for (int i = 0; i < (int)M.nodes.size(); i++) {
            if (!node_shown(i) || moving(i)) continue;
            vec2 s;
            if (!project(vi, to_world(M.nodes[i].p), s)) continue;
            const float d = d2(s, m_mouse);
            if (d < bd) bd = d, pk.node = i;
        }
        if (pk.node >= 0) {
            pk.kind = PK_Node;
            pk.p = M.nodes[pk.node].p;
            return pk;
        }
    }
    // 2. the middle of a beam, a point on a beam
    {
        float bm = kMidPx * kMidPx, be = kEdgePx * kEdgePx;
        int mid = -1, edge = -1;
        for (int i = 0; i < (int)M.beams.size(); i++) {
            const edit::Beam& b = M.beams[i];
            if (!elem_shown(Elem::Beam, i) || moving(b.a) || moving(b.b)) continue;
            vec2 sa, sb;
            if (!project(vi, to_world(M.nodes[b.a].p), sa) || !project(vi, to_world(M.nodes[b.b].p), sb)) continue;
            if (d2(sa, sb) < 24.0f * 24.0f) continue; // (too short on screen)
            float d = d2((sa + sb) * 0.5f, m_mouse);
            if (d < bm) bm = d, mid = i;
            d = seg_dist2(m_mouse, sa, sb);
            if (d < be) be = d, edge = i;
        }
        if (mid >= 0) {
            pk.kind = PK_Mid;
            pk.beam = mid;
            pk.t = 0.5f;
            pk.p = (M.nodes[M.beams[mid].a].p + M.nodes[M.beams[mid].b].p) * 0.5f;
            return pk;
        }
        if (edge >= 0) {
            const vec3 a = M.nodes[M.beams[edge].a].p, b = M.nodes[M.beams[edge].b].p;
            float t = clampf(line_param_near_ray(a, b - a, mro, rd), 0.02f, 0.98f);
            const float L = length(b - a);
            if (m_snap && L > 1e-4f) t = clampf(std::round(t * L / m_snap_size) * m_snap_size / L, 0.02f, 0.98f);
            pk.kind = PK_Edge;
            pk.beam = edge;
            pk.t = t;
            pk.p = a + (b - a) * t;
            return pk;
        }
    }
    // 3. an axis from the anchor (red x, green y, blue z); an arrow key locks one
    if (has_anchor) {
        int ax = m_axis_lock;
        vec2 sa;
        if (ax < 0 && project(vi, to_world(anchor), sa)) {
            float best = kAxisPx * kAxisPx;
            for (int a = 0; a < 3; a++) {
                vec2 sb;
                if (!project(vi, to_world(anchor + kAxes[a]), sb)) continue;
                const vec2 dv = sb - sa;
                const float l = std::sqrt(dv.x * dv.x + dv.y * dv.y);
                if (l < 1.0f) continue; // (along the view direction)
                if ((m_mouse.x - sa.x) * dv.x + (m_mouse.y - sa.y) * dv.y < -l * 1e3f) continue;
                const float d = std::fabs((m_mouse.x - sa.x) * dv.y - (m_mouse.y - sa.y) * dv.x) / l;
                if (d * d < best && d2(m_mouse, sa) > 12.0f * 12.0f) best = d * d, ax = a;
            }
        }
        if (ax >= 0) {
            float t = line_param_near_ray(anchor, kAxes[ax], mro, rd);
            if (m_snap) t = std::round(t / m_snap_size) * m_snap_size;
            pk.kind = PK_Axis;
            pk.axis = ax;
            pk.p = anchor + kAxes[ax] * t;
            return pk;
        }
    }
    // 4. the reference mesh
    if (m_ref_snap && m_ref_show && !m_ref_idx.empty()) {
        vec3 h;
        if (ref_hit(ro, rd, h)) {
            pk.kind = PK_Ref;
            pk.p = to_model(h);
            return pk;
        }
    }
    // 5. the view's drawing plane
    const vec3 n = plane_normal(vi);
    const vec3 through = plane_point(vi, has_anchor, anchor);
    vec3 out;
    if (!ray_plane(mro, rd, n, dot(n, through), out) || length2(out - mro) > 600.0f * 600.0f) return pk;
    vec3 q = snap(out);
    q = q - n * dot(q - out, n); // (the depth stays on the plane)
    pk.kind = PK_Plane;
    pk.p = q;
    return pk;
}

// ------------------------------------------------------------------------------------------------ tools
void ModelEditor::set_tool(Tool t) {
    tool_cancel();
    m_tool = t;
    if (t == Tool::Volume && active_volume() < 0 && !m_model.volumes.empty()) m_vol = 0, m_vol_point = -1; // (the first one to work on)
    m_gfx_pick = 0;
    m_status.clear();
}

void ModelEditor::op_revert() {
    // back to the snapshot the operation took when it started
    if (m_op_pushed && !m_undo.empty()) {
        m_model = m_undo.back();
        m_undo.pop_back();
        m_hidden.resize(m_model.nodes.size(), 0);
        auto trim_sel = [](std::vector<int>& v, int n) { v.erase(std::remove_if(v.begin(), v.end(), [n](int i) { return i >= n; }), v.end()); };
        trim_sel(m_sel, (int)m_model.nodes.size());
    }
    m_op_pushed = false;
}

void ModelEditor::tool_cancel() {
    if (m_op && (m_tool == Tool::Move || m_tool == Tool::Rotate || m_tool == Tool::Scale)) op_revert();
    if (m_drag == Drag::VolumePoint) { // (the dragged point back where it was)
        const int v = active_volume();
        if (v >= 0 && m_vol_point >= 0 && m_vol_point < (int)m_model.volumes[v].verts.size()) {
            m_model.volumes[v].verts[m_vol_point] = m_vol_p0;
            if (m_vol_drag_twin >= 0 && m_vol_drag_twin < (int)m_model.volumes[v].verts.size()) m_model.volumes[v].verts[m_vol_drag_twin] = m_vol_twin_p0;
        }
        if (m_drag_pushed && !m_undo.empty()) m_undo.pop_back();
        m_drag = Drag::None;
    }
    if (m_drag == Drag::MoveFree || m_drag == Drag::MoveAxis) {
        if (m_drag_pushed) op_restore(), m_undo.pop_back();
        m_drag = Drag::None;
    }
    m_op = false;
    m_op_pushed = false;
    m_op_stage = 0;
    m_pp_region.clear();
    m_chain = m_chain_start = -1;
    m_picks.clear();
    m_vcb.clear();
    m_axis_lock = -1;
}

void ModelEditor::op_move(vec3 delta) {
    op_apply([&](vec3 p) { return p + delta; });
}

void ModelEditor::op_rotate(float ang) {
    const vec3 axis = plane_normal(m_op_view), c = m_op_a;
    const quat q = quat::axis_angle(axis, ang);
    op_apply([&](vec3 p) { return c + q.rotate(p - c); });
}

void ModelEditor::op_scale(vec3 f) {
    const vec3 c = m_op_a;
    op_apply([&](vec3 p) { return c + vec3((p.x - c.x) * f.x, (p.y - c.y) * f.y, (p.z - c.z) * f.z); });
}

float ModelEditor::rotate_angle(vec3 p) const {
    const vec3 n = plane_normal(m_op_view);
    vec3 r = m_op_b - m_op_a, q = p - m_op_a;
    r -= n * dot(r, n);
    q -= n * dot(q, n);
    if (length2(r) < 1e-10f || length2(q) < 1e-10f) return 0;
    float a = std::atan2(dot(cross(r, q), n), dot(r, q));
    if (m_snap) {
        const float s = 15.0f * kDeg2Rad;
        const float k = std::round(a / s) * s;
        if (std::fabs(a - k) < 4.0f * kDeg2Rad) a = k;
    }
    return a;
}

void ModelEditor::erase_hovered() {
    edit::Model& M = m_model;
    if (m_hover_node >= 0) {
        std::vector<int> ids{m_hover_node};
        const int t = m_symmetry ? M.twin(m_hover_node) : -1;
        if (t >= 0 && t != m_hover_node) ids.push_back(t);
        std::sort(ids.begin(), ids.end());
        M.remove_nodes(ids);
        m_hidden.assign(M.nodes.size(), 0);
        clear_selection();
        return;
    }
    if (m_hover_kind == Elem::None || m_hover_elem < 0) return;
    const int i = m_hover_elem;
    std::vector<int> idx{i};
    auto twin_pair = [&](int a, int b, auto find) {
        if (!m_symmetry) return;
        const int x = twin_or_self(a), y = twin_or_self(b);
        if (x < 0 || y < 0 || (x == a && y == b)) return;
        const int j = find(x, y);
        if (j >= 0 && j != i) idx.push_back(j);
    };
    switch (m_hover_kind) {
    case Elem::Beam:
        twin_pair(M.beams[i].a, M.beams[i].b, [&](int x, int y) { return M.find_beam(x, y); });
        std::sort(idx.rbegin(), idx.rend());
        for (int k : idx) M.remove_beam(k);
        break;
    case Elem::Shock:
        twin_pair(M.shocks[i].a, M.shocks[i].b, [&](int x, int y) {
            for (int k = 0; k < (int)M.shocks.size(); k++)
                if ((M.shocks[k].a == x && M.shocks[k].b == y) || (M.shocks[k].a == y && M.shocks[k].b == x)) return k;
            return -1;
        });
        std::sort(idx.rbegin(), idx.rend());
        for (int k : idx) M.remove_shock(k);
        break;
    case Elem::Hydro:
        twin_pair(M.hydros[i].a, M.hydros[i].b, [&](int x, int y) {
            for (int k = 0; k < (int)M.hydros.size(); k++)
                if ((M.hydros[k].a == x && M.hydros[k].b == y) || (M.hydros[k].a == y && M.hydros[k].b == x)) return k;
            return -1;
        });
        std::sort(idx.rbegin(), idx.rend());
        for (int k : idx) M.remove_hydro(k);
        break;
    case Elem::Tri: M.remove_tri(i); break;
    case Elem::Wheel: M.remove_wheel(i); break;
    case Elem::Joint: M.remove_joint(i); break;
    default: break;
    }
    clear_selection();
}

void ModelEditor::tool_input() {
    ImGuiIO& io = ImGui::GetIO();
    const bool camera_drag = m_drag == Drag::Orbit || m_drag == Drag::Pan || m_drag == Drag::SplitX || m_drag == Drag::SplitY || m_drag == Drag::SplitXY;
    if (camera_drag) return;
    const bool in_view = !io.WantCaptureMouse && view_at(m_mouse) >= 0 && !m_split_hover;
    const bool click = in_view && ImGui::IsMouseClicked(ImGuiMouseButton_Left);
    const bool dbl = in_view && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left);
    const bool down = ImGui::IsMouseDown(ImGuiMouseButton_Left);
    const bool release = ImGui::IsMouseReleased(ImGuiMouseButton_Left);
    const bool shift = io.KeyShift, ctrl = io.KeyCtrl || io.KeySuper;
    edit::Model& M = m_model;
    if (m_rclick) {
        if (m_op || m_chain >= 0 || !m_picks.empty()) tool_cancel();
        else if (m_gfx_pick) m_gfx_pick = 0;
    }

    // a node for the graphics binding (the Graphics tab's pick buttons)
    if (m_gfx_pick >= 1 && m_gfx_pick <= 5) {
        if (click && m_hover_node >= 0) gfx_pick_node(m_hover_node);
        return;
    }
    // a mesh (Alt, or the Graphics tab's pick button)
    if (m_drag == Drag::None && !m_op && gfx_view_input(click)) return;

    // the snapped point of the drawing tools, from the anchor of the operation in progress
    bool has_anchor = false;
    vec3 anchor(0);
    switch (m_tool) {
    case Tool::Line:
        if (m_chain >= 0) has_anchor = true, anchor = M.nodes[m_chain].p;
        break;
    case Tool::Rect: case Tool::Circle: case Tool::Tape: case Tool::Move: case Tool::Scale:
        if (m_op && !(m_tool == Tool::Tape && m_tape_done)) has_anchor = true, anchor = m_op_a;
        break;
    case Tool::Rotate:
        if (m_op_stage >= 1) has_anchor = true, anchor = m_op_a;
        break;
    case Tool::Tri: case Tool::Shell: case Tool::FemTri: case Tool::Shock: case Tool::Rod: case Tool::Wheel:
        if (!m_picks.empty()) has_anchor = true, anchor = M.nodes[m_picks.back()].p;
        break;
    default: break;
    }
    const bool draws = m_tool != Tool::Select && m_tool != Tool::Erase && m_tool != Tool::PushPull && !(m_tool == Tool::Move && !m_op && selection_empty());
    m_pick = draws && (in_view || m_op) ? pick_point(has_anchor, anchor, true) : Pick();
    // rectangles and circles stay in the plane they were started in
    if ((m_tool == Tool::Rect || m_tool == Tool::Circle) && m_op && m_pick.valid()) {
        const vec3 n = plane_normal(m_op_view);
        m_pick.p -= n * dot(m_pick.p - m_op_a, n);
    }

    switch (m_tool) {
    // ---- select: click, shift adds, ctrl toggles, a box (left to right: inside, right to left: touching), drag a
    // selected node or a gizmo arrow to move
    case Tool::Select: {
        if (click) {
            m_drag_start = m_mouse;
            m_drag_pushed = false;
            if (m_hover_axis >= 0) {
                op_begin_nodes();
                vec3 ro, rd;
                mouse_ray(m_active_view, ro, rd);
                m_drag_anchor = gizmo_center();
                m_drag_axis = m_hover_axis;
                m_drag_t0 = line_param_near_ray(to_world(m_drag_anchor), kAxes[m_drag_axis], ro, rd);
                m_drag = Drag::MoveAxis;
            } else if (m_hover_node >= 0) {
                if (dbl) {
                    select_node(m_hover_node, shift, false);
                    select_connected();
                } else {
                    if (!(is_selected(m_hover_node) && !shift && !ctrl)) select_node(m_hover_node, shift, ctrl);
                    if (is_selected(m_hover_node) && !shift && !ctrl) {
                        op_begin_nodes();
                        m_drag_anchor = M.nodes[m_hover_node].p;
                        m_drag = Drag::MoveFree;
                    }
                }
            } else if (m_hover_kind != Elem::None) {
                if (dbl && m_hover_kind == Elem::Tri) {
                    // the face: the coplanar triangles around it, with their nodes
                    std::vector<int> region;
                    vec3 n;
                    pushpull_region(m_hover_elem, region, n);
                    if (!shift) clear_selection();
                    for (int t : region) {
                        sorted_insert(m_sel_tris, t);
                        for (int k : {M.tris[t].a, M.tris[t].b, M.tris[t].c}) sorted_insert(m_sel, k);
                    }
                } else {
                    const bool was = elem_selected(m_hover_kind, m_hover_elem);
                    if (!(was && !shift && !ctrl)) select_elem(m_hover_kind, m_hover_elem, shift, ctrl);
                    if (elem_selected(m_hover_kind, m_hover_elem) && !shift && !ctrl) {
                        op_begin_nodes();
                        vec3 ro, rd;
                        mouse_ray(m_active_view, ro, rd);
                        const Pick p = pick_point(false, vec3(0), false);
                        m_drag_anchor = p.valid() ? p.p : gizmo_center();
                        m_drag = Drag::MoveFree;
                    }
                }
            } else {
                m_drag = Drag::Box;
            }
        }
        if ((m_drag == Drag::MoveFree || m_drag == Drag::MoveAxis) && down && d2(m_mouse, m_drag_start) > 9.0f) {
            vec3 delta(0);
            if (m_drag == Drag::MoveAxis) {
                vec3 ro, rd;
                mouse_ray(m_active_view, ro, rd);
                float t = line_param_near_ray(to_world(m_drag_anchor), kAxes[m_drag_axis], ro, rd) - m_drag_t0;
                if (m_snap) t = std::round(t / m_snap_size) * m_snap_size;
                delta = kAxes[m_drag_axis] * t;
            } else {
                m_pick = pick_point(true, m_drag_anchor, true);
                if (m_pick.valid()) delta = m_pick.p - m_drag_anchor;
            }
            if (!m_drag_pushed) {
                push_undo();
                m_drag_pushed = true;
            }
            op_move(delta);
            m_op_value = length(delta);
            m_status = "Move " + fmt_len(m_op_value);
        }
        if (release) {
            if (m_drag == Drag::Box) {
                if (d2(m_mouse, m_drag_start) > 16.0f) box_select(shift, ctrl);
                else if (!shift && !ctrl) clear_selection();
            }
            if (m_drag == Drag::MoveFree || m_drag == Drag::MoveAxis || m_drag == Drag::Box) m_drag = Drag::None;
        }
        break;
    }
    // ---- line: beams from click to click; a click off a node makes one (on a beam: splits it); the start ends a loop
    case Tool::Line: {
        if (click && m_pick.valid()) {
            if (m_pick.kind != PK_Node) push_undo();
            const int n = node_from_pick(m_pick);
            if (m_chain < 0) {
                m_chain = m_chain_start = n;
            } else if (n != m_chain) {
                if (m_pick.kind == PK_Node) push_undo();
                add_beam_sym(m_chain, n);
                if (n == m_chain_start) m_chain = m_chain_start = -1; // (a closed loop)
                else m_chain = n;
            }
            m_axis_lock = -1;
        }
        break;
    }
    // ---- node: a node where the click lands (on a beam: the beam is split); the new node is selected so the flat views
    // put the next one at its depth
    case Tool::Node: {
        if (click && m_pick.valid()) {
            if (m_pick.kind == PK_Node) {
                select_node(m_pick.node, shift, ctrl);
            } else {
                push_undo();
                const int n = node_from_pick(m_pick);
                clear_selection();
                m_sel = {n};
                m_status = "Node " + std::to_string(n) + " at " + fmt_vec(M.nodes[n].p);
            }
        }
        break;
    }
    // ---- rectangle / circle: two clicks in the plane of the view (the 3D view: the plane chosen in the options)
    case Tool::Rect:
    case Tool::Circle: {
        if (click && m_pick.valid()) {
            if (!m_op) {
                m_op = true;
                m_op_a = m_pick.p;
                m_op_view = m_active_view;
            } else {
                if (m_tool == Tool::Rect) make_rect(m_op_a, m_pick.p, m_op_view);
                else make_circle(m_op_a, m_pick.p, m_op_view);
                m_op = false;
            }
        }
        if (m_op && m_pick.valid()) {
            if (m_tool == Tool::Rect) {
                vec3 u, v;
                plane_axes(plane_normal(m_op_view), u, v);
                m_status = "Rectangle " + fmt_len(std::fabs(dot(m_pick.p - m_op_a, u))) + " x " + fmt_len(std::fabs(dot(m_pick.p - m_op_a, v))) + "  (type w,h Enter)";
            } else {
                m_status = "Circle radius " + fmt_len(length(m_pick.p - m_op_a)) + "  (type the radius, Enter)";
            }
        }
        break;
    }
    // ---- push / pull: click a face, move, click; the face's triangles become a box's cap
    case Tool::PushPull: {
        vec3 ro, rd;
        mouse_ray(m_active_view, ro, rd);
        if (!m_op && click && m_hover_tri >= 0) {
            pushpull_region(m_hover_tri, m_pp_region, m_pp_n);
            const edit::Tri& t = M.tris[m_hover_tri];
            float tt = 0;
            ray_tri(ro, rd, to_world(M.nodes[t.a].p), to_world(M.nodes[t.b].p), to_world(M.nodes[t.c].p), tt);
            m_op_a = to_model(ro + rd * tt);
            m_op = true;
            m_op_view = m_active_view;
            m_op_value = 0;
        } else if (m_op) {
            float d = line_param_near_ray(m_op_a, m_pp_n, to_model(ro), rd);
            if (m_snap) d = std::round(d / m_snap_size) * m_snap_size;
            m_op_value = d;
            m_status = "Push / pull " + fmt_len(d) + "  (type a distance, Enter)";
            if (click) {
                pushpull_apply(d);
                m_op = false;
                m_pp_region.clear();
            }
        }
        break;
    }
    // ---- move / rotate / scale the selection (move with nothing selected: the clicked node or element is taken);
    // Ctrl at the first click of move moves a copy
    case Tool::Move: {
        if (!m_op && click) {
            if (selection_empty()) {
                if (m_hover_node >= 0) select_node(m_hover_node, false, false);
                else if (m_hover_kind != Elem::None) select_elem(m_hover_kind, m_hover_elem, false, false);
                m_pick = pick_point(false, vec3(0), true);
            }
            if (!selection_empty() && m_pick.valid()) {
                if (ctrl) duplicate_selection(vec3(0));
                else push_undo();
                m_op_pushed = true;
                op_begin_nodes();
                m_op_a = m_pick.p;
                m_op = true;
                m_op_view = m_active_view;
            }
        } else if (m_op) {
            if (m_pick.valid()) {
                const vec3 delta = m_pick.p - m_op_a;
                op_move(delta);
                m_op_value = length(delta);
                m_status = "Move " + fmt_len(m_op_value) + "  (type a distance or dx,dy,dz, Enter)";
            }
            if (click) m_op = false, m_op_pushed = false;
        }
        break;
    }
    case Tool::Rotate: {
        if (click && m_pick.valid()) {
            if (m_op_stage == 0) {
                if (selection_empty()) {
                    m_status = "Select what to rotate first";
                } else {
                    m_op_a = m_pick.p;
                    m_op_stage = 1;
                    m_op = true;
                    m_op_view = m_active_view;
                }
            } else if (m_op_stage == 1) {
                m_op_b = m_pick.p;
                if (length2(m_op_b - m_op_a) > 1e-8f) {
                    push_undo();
                    m_op_pushed = true;
                    op_begin_nodes();
                    m_op_stage = 2;
                }
            } else {
                m_op_stage = 0;
                m_op = false;
                m_op_pushed = false;
            }
        } else if (m_op_stage == 2 && m_pick.valid()) {
            const float a = rotate_angle(m_pick.p);
            op_rotate(a);
            m_op_value = a * kRad2Deg;
            char b[64];
            snprintf(b, sizeof b, "Rotate %.1f deg  (type an angle, Enter)", m_op_value);
            m_status = b;
        }
        break;
    }
    case Tool::Scale: {
        if (!m_op && click && m_pick.valid()) {
            if (selection_empty()) {
                m_status = "Select what to scale first";
            } else {
                m_op_a = gizmo_center();
                m_op_b = m_pick.p;
                if (length2(m_op_b - m_op_a) > 1e-6f) {
                    push_undo();
                    m_op_pushed = true;
                    op_begin_nodes();
                    m_op = true;
                    m_op_view = m_active_view;
                }
            }
        } else if (m_op) {
            if (m_pick.valid()) {
                float f = length(m_pick.p - m_op_a) / std::max(1e-4f, length(m_op_b - m_op_a));
                if (m_snap) f = std::max(0.05f, std::round(f * 20.0f) / 20.0f);
                vec3 fv(f);
                if (m_axis_lock >= 0) fv = vec3(1), (&fv.x)[m_axis_lock] = f;
                op_scale(fv);
                m_op_value = f;
                char b[80];
                snprintf(b, sizeof b, "Scale %.3f%s  (type a factor or sx,sy,sz, Enter)", f, m_axis_lock >= 0 ? " along one axis" : "");
                m_status = b;
            }
            if (click) m_op = false, m_op_pushed = false;
        }
        break;
    }
    // ---- tape measure: two clicks, the distance and its components
    case Tool::Tape: {
        if (click && m_pick.valid()) {
            if (!m_op || m_tape_done) {
                m_op = true;
                m_tape_done = false;
                m_op_a = m_tape_a = m_pick.p;
            } else {
                m_tape_b = m_pick.p;
                m_tape_done = true;
            }
        }
        if (m_op && !m_tape_done && m_pick.valid()) m_tape_b = m_pick.p;
        if (m_op) {
            const vec3 d = m_tape_b - m_tape_a;
            char b[128];
            snprintf(b, sizeof b, "Distance %.3f m   (dx %.3f  dy %.3f  dz %.3f)", length(d), d.x, d.y, d.z);
            m_status = b;
        }
        break;
    }
    // ---- eraser: click or drag over nodes and elements
    case Tool::Erase: {
        if (click) {
            push_undo();
            m_drag = Drag::Erase;
        }
        if (m_drag == Drag::Erase) {
            if (down) erase_hovered();
            if (release) m_drag = Drag::None;
        }
        break;
    }
    // ---- joint: click near an end of a frame element (Shift: its preset's joint again)
    case Tool::Joint: {
        int bi, end;
        if (click && joint_hover(bi, end)) {
            push_undo();
            const bool reset = ImGui::GetIO().KeyShift;
            set_beam_joint(bi, end, reset ? -1 : m_joint_type);
            const int j = beam_joint(bi, end);
            m_status = "Beam " + std::to_string(bi) + ", end " + (end ? "B" : "A") + ": " + phys::frame_joint_name((phys::FrameJoint)j) + (reset ? " (its preset's)" : "");
        } else if (click) {
            m_status = "Click near an end of a frame element (a FEM beam)";
        }
        break;
    }
    // ---- collision volumes: a point of one (any volume's: that one active) dragged, a node clicked an anchor or no
    // more, Ctrl+click a point added; with symmetry the mirrors too
    case Tool::Volume: {
        if (m_vol >= (int)M.volumes.size()) m_vol = (int)M.volumes.size() - 1;
        // the point under the mouse (the active volume's first)
        m_vol_hover = m_vol_hover_point = -1;
        if (in_view && m_drag == Drag::None) {
            float bd = 12.0f * 12.0f;
            for (int pass = 0; pass < 2; pass++)
                for (int v = 0; v < (int)M.volumes.size(); v++) {
                    if ((pass == 0) != (v == m_vol)) continue;
                    for (int k = 0; k < (int)M.volumes[v].verts.size(); k++) {
                        vec2 s;
                        if (!project(m_active_view, to_world(M.volumes[v].verts[k]), s)) continue;
                        const float d = d2(s, m_mouse);
                        if (d < bd) bd = d, m_vol_hover = v, m_vol_hover_point = k;
                    }
                }
        }
        if (click && m_vol_hover_point >= 0 && !ctrl) {
            m_vol = m_vol_hover, m_vol_point = m_vol_hover_point;
            m_vol_drag_twin = m_symmetry ? volume_point_twin(m_vol, m_vol_point) : -1;
            m_vol_p0 = M.volumes[m_vol].verts[m_vol_point];
            m_vol_twin_p0 = m_vol_drag_twin >= 0 ? M.volumes[m_vol].verts[m_vol_drag_twin] : vec3(0);
            m_drag_start = m_mouse;
            m_drag_pushed = false;
            m_drag = Drag::VolumePoint;
        } else if (click && ctrl && m_pick.valid()) {
            if (active_volume() < 0) m_status = "No volume: select frame nodes and make one (New in the options)";
            else volume_add_point(m_vol, m_pick.p);
        } else if (click && shift && m_pick.kind == PK_Node) {
            if (active_volume() < 0) m_status = "No volume: select frame nodes and make one (New in the options)";
            else volume_toggle_anchor(m_vol, m_pick.node);
        } else if (click) {
            m_vol_point = -1;
            if (m_pick.kind == PK_Node) m_status = "Shift+click a node: an anchor of the volume, or no more; Ctrl+click: a point of it here";
        }
        if (m_drag == Drag::VolumePoint) {
            const int v = active_volume();
            if (v < 0 || m_vol_point < 0 || m_vol_point >= (int)M.volumes[v].verts.size()) {
                m_drag = Drag::None;
            } else if (down && d2(m_mouse, m_drag_start) > 9.0f) {
                m_pick = pick_point(true, m_vol_p0, true);
                if (m_pick.valid()) {
                    if (!m_drag_pushed) push_undo(), m_drag_pushed = true;
                    const vec3 p = m_pick.p;
                    M.volumes[v].verts[m_vol_point] = p;
                    if (m_vol_drag_twin >= 0 && m_vol_drag_twin < (int)M.volumes[v].verts.size()) M.volumes[v].verts[m_vol_drag_twin] = vec3(p.x, p.y, -p.z);
                    char s[96];
                    snprintf(s, sizeof s, "Point %d at %.3f, %.3f, %.3f", m_vol_point, p.x, p.y, p.z);
                    m_status = s;
                }
            }
            if (release) m_drag = Drag::None;
        }
        break;
    }
    // ---- merge: click a node, then the node it goes into
    case Tool::Merge: {
        if (click && m_pick.kind == PK_Node) {
            if (m_picks.empty()) {
                m_picks = {m_pick.node};
                m_status = "Node " + std::to_string(m_pick.node) + ": now the node it goes into";
            } else if (m_pick.node != m_picks[0]) {
                const int src = m_picks[0], dst = m_pick.node;
                m_picks.clear();
                merge_into(src, dst);
            }
        }
        break;
    }
    // ---- node picks: triangle / shell (3), shock / rod / wheel (2); a click off a node makes one
    case Tool::Tri:
    case Tool::Shell:
    case Tool::FemTri:
    case Tool::Shock:
    case Tool::Rod:
    case Tool::Wheel: {
        if (click && m_pick.valid()) {
            if (m_picks.empty()) push_undo();
            const int n = node_from_pick(m_pick);
            if (std::find(m_picks.begin(), m_picks.end(), n) == m_picks.end()) m_picks.push_back(n);
            const size_t need = m_tool == Tool::Tri || m_tool == Tool::Shell || m_tool == Tool::FemTri ? 3 : 2;
            if (m_picks.size() == need) {
                finish_picks();
                m_picks.clear();
            }
        }
        break;
    }
    default: break;
    }
}

void ModelEditor::finish_picks() {
    edit::Model& M = m_model;
    const int a = m_picks[0], b = m_picks[1];
    switch (m_tool) {
    case Tool::Tri:
    case Tool::Shell:
    case Tool::FemTri: {
        int c = m_picks[2];
        // facing the camera of the view it was made in
        int bb = b;
        const vec3 pa = M.nodes[a].p, pb = M.nodes[b].p, pc = M.nodes[c].p;
        const vec3 n = cross(pb - pa, pc - pa);
        if (dot(n, to_model(m_views[m_active_view].cam.pos) - (pa + pb + pc) * (1.0f / 3)) < 0) std::swap(bb, c);
        add_tri_sym(a, bb, c, m_tool == Tool::FemTri ? 2 : m_tool == Tool::Shell ? 1 : 0);
        m_status = m_tool == Tool::FemTri ? "FEM triangle added (a shell element of the frame)"
                   : m_tool == Tool::Shell ? "Shell triangle added (a triangle element of the sheet)" : "Cab triangle added";
        break;
    }
    case Tool::Shock: {
        auto add = [&](int x, int y) {
            edit::Shock s = M.shocks.empty() ? edit::Shock() : M.shocks.back();
            s.a = x, s.b = y, s.layer = m_layer;
            M.shocks.push_back(s);
        };
        add(a, b);
        if (m_symmetry) {
            const int x = twin_or_self(a), y = twin_or_self(b);
            if (x >= 0 && y >= 0 && !(x == a && y == b)) add(x, y);
        }
        m_status = "Shock added";
        break;
    }
    case Tool::Rod: {
        auto add = [&](int x, int y, float f) {
            edit::Hydro h;
            h.a = x, h.b = y, h.factor = f, h.layer = m_layer;
            M.hydros.push_back(h);
        };
        add(a, b, 0.3f);
        if (m_symmetry) {
            const int x = twin_or_self(a), y = twin_or_self(b);
            if (x >= 0 && y >= 0 && !(x == a && y == b)) add(x, y, -0.3f); // (the other side's rod works the other way)
        }
        m_status = "Steering rod added (its length follows the steering)";
        break;
    }
    case Tool::Wheel: {
        auto add = [&](int x, int y) {
            edit::Wheel w = M.wheels.empty() ? edit::Wheel() : M.wheels.back();
            w.n1 = x, w.n2 = y, w.layer = m_layer, w.arm = -1;
            M.wheels.push_back(w);
        };
        add(a, b);
        if (m_symmetry) {
            const int x = twin_or_self(a), y = twin_or_self(b);
            if (x >= 0 && y >= 0 && !(x == a && y == b)) add(x, y);
        }
        m_status = "Wheel added (axle: inner node, then outer)";
        break;
    }
    default: break;
    }
}

void ModelEditor::box_select(bool add, bool remove) {
    // left to right: what is inside the box; right to left: what the box touches
    const float x0 = std::min(m_drag_start.x, m_mouse.x), x1 = std::max(m_drag_start.x, m_mouse.x);
    const float y0 = std::min(m_drag_start.y, m_mouse.y), y1 = std::max(m_drag_start.y, m_mouse.y);
    const bool crossing = m_mouse.x < m_drag_start.x;
    const edit::Model& M = m_model;
    const int vi = m_active_view;
    std::vector<char> in(M.nodes.size(), 0);
    for (int i = 0; i < (int)M.nodes.size(); i++) {
        vec2 s;
        if (node_shown(i) && project(vi, to_world(M.nodes[i].p), s)) in[i] = s.x >= x0 && s.x <= x1 && s.y >= y0 && s.y <= y1;
    }
    if (!add && !remove) clear_selection();
    auto put = [&](std::vector<int>& v, int i) {
        if (remove) {
            auto it = std::lower_bound(v.begin(), v.end(), i);
            if (it != v.end() && *it == i) v.erase(it);
        } else {
            sorted_insert(v, i);
        }
    };
    for (int i = 0; i < (int)M.nodes.size(); i++)
        if (in[i] && node_pickable(i)) put(m_sel, i);
    auto take = [&](std::initializer_list<int> nodes) {
        int k = 0;
        for (int n : nodes) k += in[n];
        return crossing ? k > 0 : k == (int)nodes.size();
    };
    for (int i = 0; i < (int)M.beams.size(); i++)
        if (elem_pickable(Elem::Beam, i) && take({M.beams[i].a, M.beams[i].b})) put(m_sel_beams, i);
    for (int i = 0; i < (int)M.shocks.size(); i++)
        if (elem_pickable(Elem::Shock, i) && take({M.shocks[i].a, M.shocks[i].b})) put(m_sel_shocks, i);
    for (int i = 0; i < (int)M.hydros.size(); i++)
        if (elem_pickable(Elem::Hydro, i) && take({M.hydros[i].a, M.hydros[i].b})) put(m_sel_hydros, i);
    for (int i = 0; i < (int)M.tris.size(); i++)
        if (elem_pickable(Elem::Tri, i) && take({M.tris[i].a, M.tris[i].b, M.tris[i].c})) put(m_sel_tris, i);
    for (int i = 0; i < (int)M.wheels.size(); i++)
        if (elem_pickable(Elem::Wheel, i) && take({M.wheels[i].n1, M.wheels[i].n2})) put(m_sel_wheels, i);
    for (int i = 0; i < (int)M.joints.size(); i++)
        if (elem_pickable(Elem::Joint, i) && take({M.joints[i].parent, M.joints[i].child})) put(m_sel_joints, i);
}

// ------------------------------------------------------------------------------------------------ typed values
bool ModelEditor::vcb_active() const {
    switch (m_tool) {
    case Tool::Line: return m_chain >= 0;
    case Tool::Rect: case Tool::Circle: case Tool::Move: case Tool::Scale: case Tool::PushPull: return m_op;
    case Tool::Rotate: return m_op_stage == 2;
    default: return false;
    }
}

void ModelEditor::vcb_input() {
    ImGuiIO& io = ImGui::GetIO();
    if (io.WantTextInput || !vcb_active()) {
        if (!vcb_active()) m_vcb.clear();
        return;
    }
    for (ImWchar c : io.InputQueueCharacters)
        if ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == ',' || c == ';') m_vcb += (char)c;
    if (!m_vcb.empty() && ImGui::IsKeyPressed(ImGuiKey_Backspace)) m_vcb.pop_back();
    if (!m_vcb.empty() && (ImGui::IsKeyPressed(ImGuiKey_Enter) || ImGui::IsKeyPressed(ImGuiKey_KeypadEnter))) {
        vcb_apply();
        m_vcb.clear();
    }
}

void ModelEditor::vcb_apply() {
    std::vector<float> v;
    for (const std::string& s : split_any(m_vcb, ",;"))
        if (!trim(s).empty()) v.push_back((float)atof(s.c_str()));
    if (v.empty()) return;
    edit::Model& M = m_model;
    switch (m_tool) {
    case Tool::Line: {
        const vec3 a = M.nodes[m_chain].p;
        vec3 p;
        if (v.size() >= 3) {
            p = a + vec3(v[0], v[1], v[2]);
        } else {
            const vec3 d = m_pick.valid() ? m_pick.p - a : vec3(0);
            if (length2(d) < 1e-10f) return;
            p = a + normalize(d) * v[0];
        }
        push_undo();
        const int n = add_node(p, true);
        if (n != m_chain) add_beam_sym(m_chain, n);
        m_chain = n;
        break;
    }
    case Tool::Rect: {
        vec3 u, w;
        plane_axes(plane_normal(m_op_view), u, w);
        const vec3 d = m_pick.valid() ? m_pick.p - m_op_a : u + w;
        const float su = dot(d, u) < 0 ? -1.0f : 1.0f, sw = dot(d, w) < 0 ? -1.0f : 1.0f;
        const float a = v[0], b = v.size() > 1 ? v[1] : v[0];
        make_rect(m_op_a, m_op_a + u * (a * su) + w * (b * sw), m_op_view);
        m_op = false;
        break;
    }
    case Tool::Circle: {
        const vec3 n = plane_normal(m_op_view);
        vec3 u, w;
        plane_axes(n, u, w);
        vec3 d = m_pick.valid() ? m_pick.p - m_op_a : u;
        d -= n * dot(d, n);
        if (length2(d) < 1e-10f) d = u;
        make_circle(m_op_a, m_op_a + normalize(d) * v[0], m_op_view);
        m_op = false;
        break;
    }
    case Tool::PushPull:
        pushpull_apply(v[0]);
        m_op = false;
        m_pp_region.clear();
        break;
    case Tool::Move: {
        vec3 delta;
        if (v.size() >= 3) {
            delta = vec3(v[0], v[1], v[2]);
        } else {
            const vec3 d = m_pick.valid() ? m_pick.p - m_op_a : vec3(0);
            if (length2(d) < 1e-10f) return;
            delta = normalize(d) * v[0];
        }
        op_move(delta);
        m_op = false;
        m_op_pushed = false;
        break;
    }
    case Tool::Rotate: {
        const float cur = m_pick.valid() ? rotate_angle(m_pick.p) : 1.0f;
        op_rotate(v[0] * kDeg2Rad * (cur < 0 ? -1.0f : 1.0f));
        m_op = false;
        m_op_stage = 0;
        m_op_pushed = false;
        break;
    }
    case Tool::Scale:
        op_scale(v.size() >= 3 ? vec3(v[0], v[1], v[2]) : vec3(v[0]));
        m_op = false;
        m_op_pushed = false;
        break;
    default: break;
    }
}

// ------------------------------------------------------------------------------------------------ hotkeys
void ModelEditor::hotkeys() {
    ImGuiIO& io = ImGui::GetIO();
    if (io.WantTextInput) return;
    auto pressed = [](ImGuiKey k) { return ImGui::IsKeyPressed(k, false); };
    const bool ctrl = io.KeyCtrl || io.KeySuper, shift = io.KeyShift;
    if (!ctrl) {
        // the beams' width, Shift: the nodes' size
        if (pressed(ImGuiKey_LeftBracket)) (shift ? m_node_px : m_beam_px) = std::max(shift ? 2.0f : 1.0f, (shift ? m_node_px : m_beam_px) - (shift ? 1.0f : 0.5f));
        if (pressed(ImGuiKey_RightBracket)) (shift ? m_node_px : m_beam_px) = std::min(shift ? 20.0f : 10.0f, (shift ? m_node_px : m_beam_px) + (shift ? 1.0f : 0.5f));
    }
    if (m_mode == Mode::Physics) {
        if (pressed(ImGuiKey_Escape)) end_mode();
        if (pressed(ImGuiKey_G)) {
            m_phys_gravity = !m_phys_gravity;
            m_game.world.settings.gravity = m_phys_gravity ? m_saved_gravity : vec3(0);
        }
        if (pressed(ImGuiKey_Space)) m_game.paused = !m_game.paused;
        if (pressed(ImGuiKey_R)) test_physics();
        // the speed: - / = (or the keypad) a step slower / faster, Backspace real time; N a step while paused
        const int ti = time_scale_index(m_phys_time);
        if (pressed(ImGuiKey_Minus) || pressed(ImGuiKey_KeypadSubtract)) set_phys_time(kTimeScales[std::max(0, ti - 1)]);
        if (pressed(ImGuiKey_Equal) || pressed(ImGuiKey_KeypadAdd)) set_phys_time(kTimeScales[std::min(kTimeScaleCount - 1, ti + 1)]);
        if (pressed(ImGuiKey_Backspace)) set_phys_time(1.0f);
        if (pressed(ImGuiKey_N) && m_game.paused) m_game.step_once = true;
        return;
    }
    if (ctrl) {
        if (pressed(ImGuiKey_Z)) shift ? redo() : undo();
        if (pressed(ImGuiKey_Y)) redo();
        if (pressed(ImGuiKey_S)) {
            std::string err;
            toast(save(&err) ? "Saved " + m_file : err);
        }
        if (pressed(ImGuiKey_A)) {
            clear_selection();
            for (int i = 0; i < (int)m_model.nodes.size(); i++)
                if (node_pickable(i)) m_sel.push_back(i);
        }
        if (pressed(ImGuiKey_D) && !m_sel.empty()) {
            // a copy in place, taken by the move tool: it follows the mouse until the click
            duplicate_selection(vec3(0));
            set_tool(Tool::Move);
            m_op_pushed = true;
            op_begin_nodes();
            m_op_a = gizmo_center();
            m_op = true;
            m_op_view = m_active_view;
        }
        if (pressed(ImGuiKey_M)) mirror_selection();
        if (pressed(ImGuiKey_B)) connect_selection(shift);
        if (pressed(ImGuiKey_L)) select_connected();
        if (pressed(ImGuiKey_I)) select_invert();
        if (pressed(ImGuiKey_G)) select_grow();
        if (pressed(ImGuiKey_F)) fill_selection();
        if (pressed(ImGuiKey_J)) merge_selection();
        if (pressed(ImGuiKey_H)) shift ? unhide_all() : hide_selection();
        if (pressed(ImGuiKey_T)) test_drive();
        if (pressed(ImGuiKey_P)) test_physics();
        return;
    }
    if (pressed(ImGuiKey_Escape)) {
        if (m_gfx_pick) m_gfx_pick = 0;
        else if (m_op || m_chain >= 0 || !m_picks.empty() || m_drag != Drag::None) tool_cancel();
        else if (m_tool == Tool::Volume && m_vol_point >= 0) m_vol_point = -1;
        else if (m_tool != Tool::Select) set_tool(Tool::Select);
        else clear_selection();
    }
    if ((pressed(ImGuiKey_Delete) || pressed(ImGuiKey_Backspace)) && !vcb_active() && m_vcb.empty()) {
        if (m_tool == Tool::Volume && active_volume() >= 0 && m_vol_point >= 0) volume_remove_point(active_volume(), m_vol_point);
        else delete_selection();
    }
    if (io.KeyAlt) {
        // the graphics: Alt+G only the graphics' nodes shown (or all again), Alt+H hides / shows the selected part,
        // Alt+Shift+H shows every part
        if (pressed(ImGuiKey_G)) set_gfx_only(m_gfx_only ? 0 : 1);
        // the selection filter: Alt+1 - 8 keeps only nodes, beams, shells, cab triangles, shocks, rods, wheels, joints;
        // with Shift that kind is deselected
        for (int k = 0; k < (int)SelKind::Count; k++)
            if (pressed((ImGuiKey)(ImGuiKey_1 + k))) select_filter((SelKind)k, !shift);
        if (pressed(ImGuiKey_H)) {
            if (shift) {
                for (int k = 1; k <= 4; k++)
                    for (int i = 0; i < gfx_count(k); i++) gfx_set_hidden(k, i, false);
            } else if (m_gfx_kind > 0 && m_gfx_sel >= 0) {
                gfx_set_hidden(m_gfx_kind, m_gfx_sel, !gfx_hidden(m_gfx_kind, m_gfx_sel));
            }
        }
        return;
    }
    if (ImGui::IsMouseDown(ImGuiMouseButton_Right)) return; // (Q / E fly)
    struct K {
        ImGuiKey key;
        Tool tool;
    };
    static const K keys[] = {{ImGuiKey_Space, Tool::Select}, {ImGuiKey_L, Tool::Line},   {ImGuiKey_N, Tool::Node},   {ImGuiKey_R, Tool::Rect},
                             {ImGuiKey_C, Tool::Circle},     {ImGuiKey_P, Tool::PushPull}, {ImGuiKey_M, Tool::Move},  {ImGuiKey_O, Tool::Rotate},
                             {ImGuiKey_K, Tool::Scale},      {ImGuiKey_U, Tool::Tape},   {ImGuiKey_E, Tool::Erase},  {ImGuiKey_T, Tool::Tri},
                             {ImGuiKey_Y, Tool::Shell},      {ImGuiKey_J, Tool::Shock},  {ImGuiKey_H, Tool::Rod},    {ImGuiKey_B, Tool::Wheel},
                             {ImGuiKey_Q, Tool::FemTri},     {ImGuiKey_Z, Tool::Volume}};
    for (const K& k : keys)
        if (pressed(k.key)) set_tool(k.tool);
    if (pressed(ImGuiKey_G)) m_snap = !m_snap;
    if (pressed(ImGuiKey_X)) m_symmetry = !m_symmetry;
    if (pressed(ImGuiKey_I)) m_show_ids = !m_show_ids;
    if (pressed(ImGuiKey_V)) {
        m_quad = !m_quad;
        m_maximized = -1;
    }
    if (pressed(ImGuiKey_F)) frame_selection();
    if (pressed(ImGuiKey_Home)) {
        const std::vector<int> keep = m_sel;
        m_sel.clear();
        frame_selection();
        m_sel = keep;
    }
    if (!vcb_active()) {
        const ImGuiKey nums[4] = {ImGuiKey_1, ImGuiKey_2, ImGuiKey_3, ImGuiKey_4};
        for (int i = 0; i < 4; i++)
            if (pressed(nums[i])) set_view(i);
    }
    // arrows lock the inference to an axis (SketchUp): right x (red), up y (green), left z (blue), down frees it
    if (pressed(ImGuiKey_RightArrow)) m_axis_lock = m_axis_lock == 0 ? -1 : 0;
    if (pressed(ImGuiKey_UpArrow)) m_axis_lock = m_axis_lock == 1 ? -1 : 1;
    if (pressed(ImGuiKey_LeftArrow)) m_axis_lock = m_axis_lock == 2 ? -1 : 2;
    if (pressed(ImGuiKey_DownArrow)) m_axis_lock = -1;
    if (pressed(ImGuiKey_PageUp)) m_work_y += m_snap_size > 0 ? m_snap_size * 2 : 0.1f;
    if (pressed(ImGuiKey_PageDown)) m_work_y -= m_snap_size > 0 ? m_snap_size * 2 : 0.1f;
}

// ------------------------------------------------------------------------------------------------ the physics test
void ModelEditor::physics_input(float dt) {
    // the tools of the game in the physics test: grab (drag a node), destroy (hold and sweep: what is under the cursor
    // breaks), shoot (projectiles), laser (hold and sweep: a cut along the path); the ray of the view under the mouse
    ImGuiIO& io = ImGui::GetIO();
    const bool in_view = !io.WantCaptureMouse && view_at(m_mouse) >= 0 && !m_split_hover;
    const bool click = in_view && ImGui::IsMouseClicked(ImGuiMouseButton_Left), lmb = ImGui::IsMouseDown(ImGuiMouseButton_Left);
    const int vi = m_drag == Drag::Grab || m_phys_held ? m_drag_view : m_active_view;
    vec3 ro, rd;
    mouse_ray(vi, ro, rd);
    // (a flat view's camera stands far back: shots and the laser start a few metres before the model)
    vec3 org = ro;
    if (m_views[vi].ortho) org = ro + rd * std::max(0.0f, dot(to_world(m_views[vi].target) - ro, rd) - 8.0f);
    m_game.tool = (bl::Tool)m_phys_tool;
    if (in_view || m_phys_held) m_game.update_cursor(org, rd);
    else m_game.cursor_valid = false;
    if (click && m_phys_tool != 0) m_phys_held = true, m_drag_view = m_active_view, m_fire_timer = 0;
    switch (m_phys_tool) {
    case 0:
        if (m_drag == Drag::None && click) {
            m_game.grab_begin(ro, rd);
            if (m_game.grab_active) m_drag = Drag::Grab, m_drag_view = m_active_view;
        }
        if (m_drag == Drag::Grab) {
            if (io.MouseWheel != 0) {
                m_game.grab_strength = clampf(m_game.grab_strength * std::pow(1.25f, io.MouseWheel), 0.05f, 50.0f);
                m_status = format("Grab strength %.2gx (the wheel while pulling)", m_game.grab_strength);
            }
            m_game.grab_update(ro, rd);
            if (!lmb) m_game.grab_end(), m_drag = Drag::None;
        }
        break;
    case 1:
        if (m_phys_held && lmb) m_game.destroy_under_cursor();
        break;
    case 2:
        if (m_phys_held && lmb) {
            m_fire_timer -= dt;
            if (m_fire_timer <= 0) {
                m_game.shoot(org, rd);
                m_fire_timer += 1.0f / std::max(0.5f, m_game.fire_rate);
            }
        }
        break;
    case 3:
        if (m_phys_held && lmb) m_game.laser_sweep(org, rd);
        break;
    default: break;
    }
    if (!lmb && m_phys_held) {
        m_phys_held = false;
        m_game.laser_release();
    }
    // keys: 1 - 4 the tools
    if (!io.WantTextInput && !(io.KeyCtrl || io.KeySuper)) {
        const ImGuiKey keys[4] = {ImGuiKey_1, ImGuiKey_2, ImGuiKey_3, ImGuiKey_4};
        for (int i = 0; i < 4; i++)
            if (ImGui::IsKeyPressed(keys[i], false)) m_phys_tool = i;
    }
}

// ------------------------------------------------------------------------------------------------ the frame
void ModelEditor::frame(GLFWwindow* win, float dt) {
    if (!m_active) return;
    m_win = win;
    if (m_toast_time > 0) m_toast_time -= dt;
    if (m_game.scene_index != m_scene) {
        // another scene was loaded (the Scene menu): the editor steps aside, its model stays
        if (m_mode == Mode::Physics) m_game.world.settings.gravity = m_saved_gravity, m_game.debug.beams = m_saved_debug_beams;
        m_mode = Mode::Edit;
        m_preview = m_test = nullptr;
        m_active = false;
        m_game.debug.hide_terrain = false;
        return;
    }
    ImGuiIO& io = ImGui::GetIO();
    if (m_mode == Mode::Drive) {
        if (!io.WantTextInput && ImGui::IsKeyPressed(ImGuiKey_Escape, false)) end_mode();
        if (m_test && !alive(m_test)) end_mode();
        return;
    }
    if (m_mode == Mode::Physics && m_test && !alive(m_test)) end_mode();
    layout(io.DisplaySize.x, io.DisplaySize.y);
    m_prev_mouse = m_mouse;
    m_mouse = vec2(io.MousePos.x, io.MousePos.y);
    // the active view follows the mouse (not while an operation started in a view goes on)
    const bool held = m_drag != Drag::None || (m_op && (m_tool == Tool::Rect || m_tool == Tool::Circle || m_tool == Tool::Rotate || m_tool == Tool::PushPull));
    if (!held && !io.WantCaptureMouse) {
        const int v = view_at(m_mouse);
        if (v >= 0) m_active_view = v;
    }
    if (!m_views[m_active_view].shown) m_active_view = m_quad ? (m_maximized >= 0 ? m_maximized : 0) : m_single;
    update_cameras();
    camera_input(dt);
    update_cameras();
    if (m_mode == Mode::Deform) {
        deform_input(dt);
        update_preview(dt);
        return;
    }
    hotkeys();
    if (m_mode == Mode::Physics) {
        physics_input(dt);
        if (m_test) {
            m_test->ghost = m_show_gfx ? std::max(0.3f, m_gfx_alpha) : 0.0f; // (the beams show through)
            apply_part_states(m_test);
        }
        return;
    }
    vcb_input();
    update_gfx_mask();
    update_hover();
    tool_input();
    m_hidden.resize(m_model.nodes.size(), 0);
    if (getenv("BL_EDITOR_LOG")) {
        // (headless checks: the views once, then every change of the model)
        static int last[4] = {-1, -1, -1, -1};
        static float area[4] = {0, 0, 0, 0};
        if (memcmp(area, m_area, sizeof area)) {
            memcpy(area, m_area, sizeof area);
            for (int i = 0; i < 4; i++)
                if (m_views[i].shown) printf("editor view %d (%s): %.0f %.0f %.0f %.0f\n", i, m_views[i].ortho ? "flat" : "3D", m_views[i].rx, m_views[i].ry, m_views[i].rw, m_views[i].rh);
        }
        static std::string last_status;
        if (m_status != last_status) last_status = m_status, printf("editor status: %s\n", m_status.c_str());
        const int now[4] = {(int)m_model.nodes.size(), (int)m_model.beams.size(), (int)m_model.tris.size(), (int)m_model.wheels.size()};
        if (memcmp(now, last, sizeof now)) {
            memcpy(last, now, sizeof now);
            printf("editor: %d nodes, %d beams, %d triangles, %d wheels | tool %d view %d | %s\n", now[0], now[1], now[2], now[3], (int)m_tool, m_active_view, m_status.c_str());
            if (now[0] > 0) {
                const vec3 p = m_model.nodes.back().p;
                printf("editor:   last node %d at (%.3f %.3f %.3f)\n", now[0] - 1, p.x, p.y, p.z);
            }
        }
    }
    if (m_gfx_sel >= 0 && m_gfx_sel >= gfx_count(m_gfx_kind)) m_gfx_sel = -1;
    update_preview(dt);
}

} // namespace bl
