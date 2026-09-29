// The model editor's frame: the views (the game's world drawn once per view, the stage's background and horizon, the
// floor on or off), the grid, the model, the tools' previews and the graphics bindings (editor.h).
#include "game/editor.h"

#include "core/util.h"
#include "game/editor_internal.h"
#include "game/game.h"
#include "vehicle/vehicle.h"

#include "imgui.h"

#include <algorithm>
#include <cmath>

namespace bl {

using namespace edit_detail;

namespace {

const uint32_t kPickCol[] = {
    Renderer::rgba(1, 1, 1, 0),             // none
    Renderer::rgba(0.85f, 0.85f, 0.85f, 1), // plane
    Renderer::rgba(0.3f, 1, 0.35f, 1),      // node
    Renderer::rgba(1, 1, 1, 1),             // axis (the axis colour is used)
    Renderer::rgba(0.3f, 0.9f, 1, 1),       // midpoint
    Renderer::rgba(1, 0.35f, 0.3f, 1),      // on a beam
    Renderer::rgba(1, 0.4f, 1, 1),          // on the reference mesh
};

// RoR's placement of a flexbody / prop in the frame of its ref, x and y nodes (visual.cpp's place())
void placement(vec3 pref, vec3 px, vec3 py, vec3 off, vec3 rot_deg, vec3& pos, mat3& orient) {
    const vec3 X = px - pref, Y = py - pref;
    const vec3 normal = normalize_or(cross(Y, X), vec3(0, 1, 0));
    pos = pref + X * off.x + Y * off.y + normal * off.z;
    const vec3 rx = normalize_or(X, vec3(1, 0, 0)), ry = cross(rx, normal);
    orient = mat3(rx, normal, ry) * to_mat3(quat_euler_xyz_deg(rot_deg.x, rot_deg.y, rot_deg.z));
}

} // namespace

// ------------------------------------------------------------------------------------------------ the views
void ModelEditor::render(Renderer& r, int fb_w, int fb_h) {
    r.px_scale = (float)fb_w / std::max(1.0f, ImGui::GetIO().DisplaySize.x);
    if (!m_active || m_mode == Mode::Drive) {
        r.set_viewport(-1, 0, 1, 1);
        r.draw_sky = true;
        r.sky_clouds = true;
        m_game.render(r, fb_w, fb_h);
        r.render();
        return;
    }
    ImGuiIO& io = ImGui::GetIO();
    const float sx = (float)fb_w / std::max(1.0f, io.DisplaySize.x), sy = (float)fb_h / std::max(1.0f, io.DisplaySize.y);
    const LightSettings saved = m_game.light;
    const DebugView saved_dbg = m_game.debug;
    // the stage: a plain background (the flat views) and a sky gradient with the horizon (the 3D view), the floor
    // shown or not (it collides either way), the model's own lines instead of the game's debug drawing
    m_game.debug.hide_terrain = !m_floor;
    if (m_mode == Mode::Edit || m_mode == Mode::Deform) m_game.debug.beams = m_game.debug.nodes = m_game.debug.stress = false;
    if (m_mode == Mode::Physics) m_game.debug.nodes = true, m_game.debug.frames = false; // (the nodes to drag; the joints' axes looked like stray beams)
    m_game.debug.beam_px = m_beam_px;
    const float saved_point = r.point_size;
    r.point_size = m_node_px * sx;
    m_game.light.fog_color = m_bg;
    m_game.light.sky_color = m_bg * 1.35f;
    m_game.light.ground_color = saved.ground_color;
    m_game.light.fog_density = 0.0004f;
    m_game.light.sun_color = saved.sun_color * 0.6f; // (softer: the floor seen from above stays dark)
    r.sky_clouds = false;
    build_fill_mesh();
    r.clear_screen(fb_w, fb_h, m_bg * 0.55f); // (the gaps between the views)
    bool first = true;
    for (int i = 0; i < 4; i++) {
        View& v = m_views[i];
        if (!v.shown) continue;
        const int x = (int)std::lround(v.rx * sx), w = std::max(1, (int)std::lround(v.rw * sx)), h = std::max(1, (int)std::lround(v.rh * sy));
        const int y = fb_h - (int)std::lround((v.ry + v.rh) * sy);
        r.set_viewport(x, y, w, h);
        m_game.light.shadows = v.ortho == 0 && saved.shadows;
        r.draw_sky = v.ortho == 0;
        m_game.debug.hide_terrain = !m_floor || v.ortho != 0; // (the flat views: a plain background with the grid)
        m_game.render(r, fb_w, fb_h, &v.cam, first);
        draw(r, i);
        r.render();
        first = false;
    }
    r.set_viewport(-1, 0, 1, 1);
    r.draw_sky = true;
    r.sky_clouds = true;
    r.point_size = saved_point;
    m_game.light = saved;
    m_game.debug = saved_dbg;
}

void ModelEditor::draw_grid(Renderer& r, int view) {
    const View& v = m_views[view];
    const vec3 o = m_origin;
    // the step: a power of ten for the zoom (minor lines), every tenth line major
    const float span = v.ortho ? v.cam.ortho_half * std::max(1.0f, v.rw / std::max(1.0f, v.rh)) * 1.1f : std::max(4.0f, v.dist * 2.5f);
    float step = std::pow(10.0f, std::floor(std::log10(std::max(0.001f, span / 12.0f))));
    while (span / step > 160) step *= 2;
    const uint32_t minor = Renderer::rgba(1, 1, 1, 0.07f), major = Renderer::rgba(1, 1, 1, 0.2f);
    auto grid = [&](vec3 center, vec3 u, vec3 w) {
        const float cu = std::round(dot(center, u) / step) * step, cw = std::round(dot(center, w) / step) * step;
        const vec3 base = center - u * dot(center, u) - w * dot(center, w);
        const int n = (int)std::ceil(span / step);
        for (int k = -n; k <= n; k++) {
            const float a = cu + k * step, b = cw + k * step;
            const bool ma = std::fabs(std::fmod(std::fabs(a) + step * 0.25f, step * 10.0f)) < step * 0.5f;
            const bool mb = std::fabs(std::fmod(std::fabs(b) + step * 0.25f, step * 10.0f)) < step * 0.5f;
            r.line(o + base + u * a + w * (cw - span), o + base + u * a + w * (cw + span), ma ? major : minor);
            r.line(o + base + w * b + u * (cu - span), o + base + w * b + u * (cu + span), mb ? major : minor);
        }
    };
    const ViewFrame vf = view_frame(v.cam, v.rh);
    if (v.ortho == 0 || v.ortho == 3) {
        // the ground grid at the work height, the axes through the origin
        const float y = m_work_y + 0.002f;
        grid(vec3(v.target.x, y, v.target.z), vec3(1, 0, 0), vec3(0, 0, 1));
        const float L = span;
        thick_line(r, vf, o + vec3(-L, y, 0), o + vec3(L, y, 0), dim(kAxisCol[0], 0.55f), 1.5f);
        thick_line(r, vf, o + vec3(0, y, -L), o + vec3(0, y, L), dim(kAxisCol[2], 0.55f), 1.5f);
        if (v.ortho == 0) thick_line(r, vf, o + vec3(0, y, 0), o + vec3(0, std::max(1.0f, v.dist * 0.15f), 0), dim(kAxisCol[1], 0.8f), 1.5f);
        // the front of the model: an arrow toward -x on the ground
        const float s = std::max(0.3f, step * 3);
        r.line(o + vec3(-L * 0.02f - s * 2, y, 0), o + vec3(-L * 0.02f - s * 1.4f, y, s * 0.4f), kAxisCol[0]);
        r.line(o + vec3(-L * 0.02f - s * 2, y, 0), o + vec3(-L * 0.02f - s * 1.4f, y, -s * 0.4f), kAxisCol[0]);
    } else {
        // behind the model, facing the view
        const vec3 fwd = v.cam.forward();
        const vec3 c = v.target + fwd * 60.0f;
        if (v.ortho == 1) grid(c, vec3(0, 0, 1), vec3(0, 1, 0));
        else grid(c, vec3(1, 0, 0), vec3(0, 1, 0));
        const vec3 d = fwd * 60.0f;
        // the ground line, the axes through the origin
        const vec3 h = v.ortho == 1 ? vec3(0, 0, 1) : vec3(1, 0, 0);
        thick_line(r, vf, o + d - h * span * 3 + vec3(0, m_work_y, 0), o + d + h * span * 3 + vec3(0, m_work_y, 0), Renderer::rgba(0.9f, 0.9f, 0.9f, 0.5f), 1.5f);
        thick_line(r, vf, o + d + vec3(0, -span * 3, 0), o + d + vec3(0, span * 3, 0), dim(kAxisCol[1], 0.5f), 1.0f);
        if (v.ortho == 1) thick_line(r, vf, o + d, o + d, kAxisCol[2], 1);
    }
}

// ------------------------------------------------------------------------------------------------ the model
void ModelEditor::build_fill_mesh() {
    m_tri_mesh_ready = false;
    if (!m_fill || m_mode != Mode::Edit) return;
    const edit::Model& M = m_model;
    std::vector<Vertex> vs;
    std::vector<uint32_t> idx;
    for (int i = 0; i < (int)M.tris.size(); i++) {
        if (!elem_shown(Elem::Tri, i)) continue;
        const edit::Tri& t = M.tris[i];
        const vec3 a = to_world(M.nodes[t.a].p), b = to_world(M.nodes[t.b].p), c = to_world(M.nodes[t.c].p);
        const vec3 n = normalize_or(cross(b - a, c - a), vec3(0, 1, 0));
        for (vec3 p : {a, b, c}) {
            idx.push_back((uint32_t)vs.size());
            vs.push_back({p, n, vec2(t.shell || t.fem ? 1.0f : 0.0f, 0)});
        }
    }
    if (vs.empty()) return;
    if (m_tri_mesh_count != (int)vs.size()) {
        m_tri_mesh.create(vs, idx, true);
        m_tri_mesh_count = (int)vs.size();
    } else {
        m_tri_mesh.update_vertices(vs.data(), (int)vs.size());
    }
    m_tri_mesh_ready = true;
}

void ModelEditor::draw(Renderer& r, int view) {
    const View& vw = m_views[view];
    const ViewFrame vf = view_frame(vw.cam, vw.rh);
    if (m_grid) draw_grid(r, view);
    // the reference mesh (a mockup: see-through)
    if (m_ref_show && !m_ref_idx.empty() && m_ref_mesh.index_count() > 0) {
        m_ref_mat->color.w = m_model.ref_mockup ? m_model.ref_alpha : 1.0f;
        m_ref_mat->blend = m_model.ref_mockup;
        r.draw_mesh(&m_ref_mesh, m_ref_mat.get(), ref_matrix());
    }
    if (m_mode == Mode::Deform) {
        draw_deform(r, view);
        return;
    }
    if (m_mode == Mode::Physics) {
        if (m_game.grab_active && m_game.grab_body && m_game.grab_node >= 0 && m_game.grab_node < (int)m_game.grab_body->nodes.size()) {
            const vec3 p = m_game.grab_body->nodes[m_game.grab_node].p;
            thick_line(r, vf, p, m_game.grab_body->grab_target, Renderer::rgba(1, 0.8f, 0.2f, 1), 2);
            ring(r, vf, p, Renderer::rgba(1, 0.8f, 0.2f, 1), 8);
        }
        return;
    }
    const edit::Model& M = m_model;
    auto P = [&](int n) { return to_world(M.nodes[n].p); };
    // the skeleton's opacity (the Graphics tab: the beams and nodes give way to the meshes; 0: only the selected and the
    // hovered are drawn); the lines give way a little to the graphics anyway
    const float skel = m_skel_alpha * (m_show_gfx && m_preview ? 0.75f : 1.0f);
    const bool skel_off = skel < 0.02f;
    auto hi_of = [&](Elem k, int i) { return elem_selected(k, i) ? 1 : (m_hover_kind == k && m_hover_elem == i ? 2 : 0); };
    // widths (points): the display setting; the selected and the hovered wider, held ends and shocks a little wider
    const float bw = m_beam_px, sel_w = std::max(bw + 2.5f, 4.0f), hov_w = std::max(bw + 1.5f, 3.0f);
    auto seg = [&](Elem k, int i, int a, int b, uint32_t c, int layer, float scale) {
        const int hi = hi_of(k, i);
        if (hi) thick_line(r, vf, P(a), P(b), hi == 1 ? kSelCol : kHoverCol, hi == 1 ? sel_w : hov_w);
        else if (!skel_off) r.thick_line(P(a), P(b), dim(M.layer_locked(layer) ? dim(c, 0.35f) : c, skel), bw * scale);
    };
    // beams: the preset's colour; ends held (a rigid joint) thicker, ropes dashed, invisible ones dimmer
    for (int i = 0; i < (int)M.beams.size(); i++) {
        if (!elem_shown(Elem::Beam, i)) continue;
        const edit::Beam& b = M.beams[i];
        const edit::BeamGroup& g = M.groups[std::clamp(b.group, 0, (int)M.groups.size() - 1)];
        uint32_t c = dim(col(g.color), g.invisible ? 0.5f : 1.0f);
        if (g.type == edit::BEAM_ROPE && !hi_of(Elem::Beam, i)) {
            if (!skel_off) dashed(r, P(b.a), P(b.b), dim(c, skel), std::max(0.02f, length(P(b.b) - P(b.a)) / 14.0f), bw);
            continue;
        }
        seg(Elem::Beam, i, b.a, b.b, c, b.layer, g.is_frame() ? 2.4f : g.hold_rotation ? 1.8f : 1.0f);
    }
    // the frame elements' joints other than welded: a mark near each end (ball: a ring, hinges: their axis, swivel: a
    // ring round the member, elastic: a spring); the joint tool's end under the mouse lit
    {
        int hb = -1, he = -1;
        if (m_tool == Tool::Joint) joint_hover(hb, he);
        for (int i = 0; i < (int)M.beams.size(); i++) {
            const edit::Beam& b = M.beams[i];
            if (!M.groups[std::clamp(b.group, 0, (int)M.groups.size() - 1)].is_frame() || !elem_shown(Elem::Beam, i) || skel_off) continue;
            for (int end = 0; end < 2; end++) {
                const int j = beam_joint(i, end);
                const bool lit = i == hb && end == he;
                if (j == 0 && !lit) continue;
                const vec3 pn = P(end ? b.b : b.a), po = P(end ? b.a : b.b), pa = P(b.a), pb = P(b.b);
                const float L = length(po - pn);
                if (L < 1e-4f) continue;
                const vec3 e1 = normalize(pb - pa), dir = (po - pn) / L;
                const vec3 ref = std::fabs(e1.y) < 0.9f ? vec3(0, 1, 0) : vec3(1, 0, 0);
                const vec3 e2 = normalize(ref - e1 * dot(e1, ref)), e3 = cross(e1, e2);
                const float px = vf.px_at(pn);
                const vec3 c = pn + dir * std::min(L * 0.3f, px * 18.0f);
                const float sz = px * 8.0f;
                const vec3 jc = joint_color(j);
                const uint32_t col = lit ? kHoverCol : Renderer::rgba(jc.x, jc.y, jc.z, 1);
                if (lit) ring(r, vf, pn, kHoverCol, 11, 16, 2.5f);
                switch (j) {
                case 0: square(r, vf, c, col, 5); break;
                case 1: ring(r, vf, c, col, 6, 14, 2.5f); break;
                case 2: thick_line(r, vf, c - e3 * sz * 1.4f, c + e3 * sz * 1.4f, col, 3.0f), ring(r, vf, c, col, 3, 10, 2.0f); break;
                case 3: thick_line(r, vf, c - e2 * sz * 1.4f, c + e2 * sz * 1.4f, col, 3.0f), ring(r, vf, c, col, 3, 10, 2.0f); break;
                case 4:
                    for (int k = 0; k < 16; k++) {
                        const float a0 = 2 * kPi * k / 16, a1 = 2 * kPi * (k + 1) / 16;
                        thick_line(r, vf, c + (e2 * std::cos(a0) + e3 * std::sin(a0)) * sz, c + (e2 * std::cos(a1) + e3 * std::sin(a1)) * sz, col, 2.0f);
                    }
                    break;
                default:
                    for (int k = 0; k < 6; k++) {
                        const vec3 q0 = c + dir * (sz * (k / 3.0f - 1.0f)) + e2 * (sz * (k % 2 ? 0.6f : -0.6f));
                        const vec3 q1 = c + dir * (sz * ((k + 1) / 3.0f - 1.0f)) + e2 * (sz * (k % 2 ? -0.6f : 0.6f));
                        thick_line(r, vf, q0, q1, col, 2.0f);
                    }
                    break;
                }
            }
        }
    }
    for (int i = 0; i < (int)M.shocks.size(); i++)
        if (elem_shown(Elem::Shock, i)) seg(Elem::Shock, i, M.shocks[i].a, M.shocks[i].b, Renderer::rgba(0.3f, 1, 1, 1), M.shocks[i].layer, 1.4f);
    for (int i = 0; i < (int)M.hydros.size(); i++)
        if (elem_shown(Elem::Hydro, i)) seg(Elem::Hydro, i, M.hydros[i].a, M.hydros[i].b, Renderer::rgba(1, 0.4f, 1, 1), M.hydros[i].layer, 1.4f);
    for (int i = 0; i < (int)M.joints.size(); i++) {
        if (!elem_shown(Elem::Joint, i)) continue;
        const edit::Joint& j = M.joints[i];
        const uint32_t c = Renderer::rgba(1, 0.75f, 0.2f, 1);
        seg(Elem::Joint, i, j.parent, j.child, c, j.layer, 1.4f);
        square(r, vf, P(j.parent), c, 5);
    }
    // the sheet's welds (a short tick from the frame node), the parts' mounts (a line with a square at the part's node)
    // and the slide nodes (a line to the middle of their rail): not editable here, shown so the parts' ties are seen
    if (!skel_off) {
        const uint32_t cw = dim(Renderer::rgba(1.0f, 0.85f, 0.25f, 0.9f), skel), cm = dim(Renderer::rgba(1.0f, 0.45f, 0.15f, 1), skel),
                       cs = dim(Renderer::rgba(0.3f, 1.0f, 0.6f, 1), skel);
        auto shown = [&](int n) { return n >= 0 && n < (int)M.nodes.size() && node_shown(n); };
        for (const edit::Weld& w : M.welds)
            if (shown(w.anchor) && shown(w.node)) r.thick_line(P(w.anchor), P(w.node), cw, bw * 0.8f);
        for (const edit::Mount& mt : M.mounts)
            if (shown(mt.a) && shown(mt.b)) r.thick_line(P(mt.a), P(mt.b), cm, bw * 1.2f), square(r, vf, P(mt.b), cm, 5);
        for (const edit::SlideNode& sn : M.slidenodes) {
            if (!shown(sn.node)) continue;
            vec3 mid(0);
            for (int n : sn.rail) mid += P(n) * (1.0f / (float)sn.rail.size());
            r.thick_line(P(sn.node), mid, cs, bw), square(r, vf, P(sn.node), cs, 5);
        }
    }
    for (int i = 0; i < (int)M.wheels.size(); i++) {
        if (!elem_shown(Elem::Wheel, i)) continue;
        const edit::Wheel& w = M.wheels[i];
        const int hi = hi_of(Elem::Wheel, i);
        uint32_t c = hi == 1 ? kSelCol : hi == 2 ? kHoverCol : Renderer::rgba(1, 0.9f, 0.3f, 0.9f);
        if (!hi && M.layer_locked(w.layer)) c = dim(c, 0.35f);
        if (!hi && skel_off) continue;
        if (!hi) c = dim(c, skel);
        const vec3 a = P(w.n1), b = P(w.n2), ax = normalize_or(b - a, vec3(0, 0, 1));
        const float ww = hi == 1 ? sel_w : hi == 2 ? hov_w : bw;
        r.thick_line(a, b, c, ww);
        const vec3 u = normalize_or(cross(ax, vec3(0, 1, 0)), vec3(1, 0, 0)), v = cross(ax, u);
        const int n = 28;
        for (int k = 0; k < n; k++) {
            const float t0 = 2 * kPi * k / n, t1 = 2 * kPi * (k + 1) / n;
            const vec3 p0 = (u * std::cos(t0) + v * std::sin(t0)), p1 = (u * std::cos(t1) + v * std::sin(t1));
            r.thick_line(a + p0 * w.radius, a + p1 * w.radius, c, ww);
            r.thick_line(b + p0 * w.radius, b + p1 * w.radius, c, ww);
            if (w.type != 0 && w.rim_radius > 0) r.thick_line(b + p0 * w.rim_radius, b + p1 * w.rim_radius, dim(c, 0.5f), ww * 0.7f);
            if (k % 7 == 0) r.thick_line(a + p0 * w.radius, b + p0 * w.radius, c, ww * 0.7f);
        }
    }
    // triangles: the wire, the outside of the selected, the fill (shells tinted)
    for (int i = 0; i < (int)M.tris.size(); i++) {
        if (!elem_shown(Elem::Tri, i)) continue;
        const edit::Tri& t = M.tris[i];
        const int hi = hi_of(Elem::Tri, i);
        const vec3 sc = t.fem ? fem_preset_color(t.fem_preset) * 1.1f + vec3(0.12f) : t.shell ? shell_preset_color(t.shell_preset) * 1.15f + vec3(0.1f) : vec3(0);
        uint32_t c = hi == 1 ? kSelCol : hi == 2 ? kHoverCol : t.shell || t.fem ? Renderer::rgba(sc.x, sc.y, sc.z, t.fem ? 0.9f : 0.75f)
                     : t.hull() ? Renderer::rgba(1.0f, 0.62f, 0.15f, 0.55f) : Renderer::rgba(0.5f, 0.75f, 1, t.collision ? 0.5f : 0.25f);
        if (!hi && M.layer_locked(t.layer)) c = dim(c, 0.35f);
        const vec3 a = P(t.a), b = P(t.b), cc = P(t.c);
        if (hi) {
            thick_line(r, vf, a, b, c, hov_w), thick_line(r, vf, b, cc, c, hov_w), thick_line(r, vf, cc, a, c, hov_w);
            const vec3 m = (a + b + cc) * (1.0f / 3);
            const vec3 n = normalize_or(cross(b - a, cc - a), vec3(0, 1, 0));
            thick_line(r, vf, m, m + n * std::max(0.05f, vf.px_at(m) * 30), Renderer::rgba(1, 1, 1, 1), 2); // (the outside)
        } else if ((!m_fill || m_model.beams.empty() || t.shell || t.fem) && !skel_off) {
            c = dim(c, skel);
            r.thick_line(a, b, c, bw * 0.7f), r.thick_line(b, cc, c, bw * 0.7f), r.thick_line(cc, a, c, bw * 0.7f);
        }
    }
    m_tri_mat->color.w = 0.22f * std::min(1.0f, skel);
    if (m_tri_mesh_ready && !skel_off) r.draw_mesh(&m_tri_mesh, m_tri_mat.get(), mat4());
    // nodes: a point, rings round the selected and the hovered
    for (int i = 0; i < (int)M.nodes.size(); i++) {
        if (!node_shown(i)) continue;
        const edit::Node& n = M.nodes[i];
        uint32_t c = n.fixed ? Renderer::rgba(1, 0.25f, 0.25f, 1) : n.load_bearing ? Renderer::rgba(1, 0.95f, 0.55f, 1) : Renderer::rgba(0.8f, 0.8f, 0.7f, 1);
        if (M.layer_locked(n.layer)) c = dim(c, 0.4f);
        const bool picked = i == m_chain || std::find(m_picks.begin(), m_picks.end(), i) != m_picks.end();
        // (the rings just outside the node's dot, whatever its size)
        const float rr = m_node_px * 0.5f + 3.0f;
        if (is_selected(i)) {
            c = kSelCol;
            ring(r, vf, P(i), kSelCol, rr, 14, 2.0f);
        }
        if (i == m_hover_node || picked) {
            c = kHoverCol;
            ring(r, vf, P(i), m_gfx_pick ? Renderer::rgba(0.3f, 1, 0.5f, 1) : kHoverCol, picked ? rr : rr + 2.0f, 14, 2.0f);
        } else if (!is_selected(i)) {
            if (skel_off) continue;
            c = dim(c, skel);
        }
        r.point(P(i), c);
    }
    // the move gizmo of the select tool
    if (m_tool == Tool::Select && !selection_empty() && view == m_active_view) {
        const vec3 c = to_world(gizmo_center());
        const float L = vf.px_at(c) * 70.0f;
        for (int a = 0; a < 3; a++) {
            vec3 e(0);
            (&e.x)[a] = L;
            const bool hi = m_hover_axis == a || (m_drag == Drag::MoveAxis && m_drag_axis == a);
            thick_line(r, vf, c, c + e, hi ? kHoverCol : kAxisCol[a], hi ? 4.0f : 2.5f);
            ring(r, vf, c + e, hi ? kHoverCol : kAxisCol[a], 4, 8);
        }
    }
    draw_tool_preview(r, view);
    draw_graphics_binding(r, view);
}

// ------------------------------------------------------------------------------------------------ the tools
vec3 ModelEditor::joint_color(int j) {
    switch (j) {
    case 1: return vec3(1.0f, 0.6f, 0.2f);   // ball
    case 2: return vec3(0.45f, 1.0f, 0.5f);  // hinge, vertical plane
    case 3: return vec3(0.3f, 0.9f, 1.0f);   // hinge, horizontal plane
    case 4: return vec3(1.0f, 0.45f, 0.9f);  // swivel
    case 5: return vec3(1.0f, 0.9f, 0.3f);   // elastic
    default: return vec3(0.9f, 0.92f, 0.95f); // welded
    }
}

void ModelEditor::draw_tool_preview(Renderer& r, int view) {
    const View& vw = m_views[view];
    const ViewFrame vf = view_frame(vw.cam, vw.rh);
    const edit::Model& M = m_model;
    auto W = [&](vec3 p) { return to_world(p); };
    const uint32_t white = Renderer::rgba(1, 1, 1, 0.9f), ghost = Renderer::rgba(1, 1, 1, 0.55f);
    const Pick& pk = m_pick;
    // the inference marker
    if (pk.valid()) {
        const vec3 p = W(pk.p);
        const uint32_t c = pk.kind == PK_Axis ? kAxisCol[pk.axis] : kPickCol[pk.kind];
        switch (pk.kind) {
        case PK_Node: ring(r, vf, p, c, 8), ring(r, vf, p, c, 6); break;
        case PK_Mid: square(r, vf, p, c, 6), square(r, vf, p, c, 4); break;
        case PK_Edge: square(r, vf, p, c, 5); break;
        case PK_Ref: ring(r, vf, p, c, 5, 6); break;
        case PK_Axis: ring(r, vf, p, c, 4, 8); break;
        default: {
            const float s = vf.px_at(p) * 6;
            r.line(p - vf.right * s, p + vf.right * s, c);
            r.line(p - vf.up * s, p + vf.up * s, c);
        }
        }
    }
    auto axis_line = [&](vec3 a, vec3 b) {
        // the rubber band, in the axis colour when it follows an axis
        const uint32_t c = pk.kind == PK_Axis ? kAxisCol[pk.axis] : white;
        thick_line(r, vf, W(a), W(b), c, 2);
    };
    switch (m_tool) {
    case Tool::Line:
        if (m_chain >= 0 && pk.valid()) axis_line(M.nodes[m_chain].p, pk.p);
        break;
    case Tool::Tri: case Tool::Shell: case Tool::FemTri: case Tool::Shock: case Tool::Rod: case Tool::Wheel:
        if (!m_picks.empty() && pk.valid()) {
            axis_line(M.nodes[m_picks.back()].p, pk.p);
            if (m_picks.size() == 2) thick_line(r, vf, W(M.nodes[m_picks[0]].p), W(pk.p), ghost, 2);
            if (m_picks.size() == 2) thick_line(r, vf, W(M.nodes[m_picks[0]].p), W(M.nodes[m_picks[1]].p), ghost, 2);
        }
        break;
    case Tool::Rect:
        if (m_op && pk.valid()) {
            vec3 u, v;
            plane_axes(plane_normal(m_op_view), u, v);
            const vec3 a = m_op_a, d = pk.p - a;
            const vec3 du = u * dot(d, u), dv = v * dot(d, v);
            const vec3 c[4] = {a, a + du, a + du + dv, a + dv};
            for (int k = 0; k < 4; k++) thick_line(r, vf, W(c[k]), W(c[(k + 1) % 4]), white, 2);
            for (int k = 1; k < m_rect_div[0]; k++) r.line(W(a + du * ((float)k / m_rect_div[0])), W(a + du * ((float)k / m_rect_div[0]) + dv), ghost);
            for (int k = 1; k < m_rect_div[1]; k++) r.line(W(a + dv * ((float)k / m_rect_div[1])), W(a + dv * ((float)k / m_rect_div[1]) + du), ghost);
            if (m_rect_diagonals) r.line(W(c[0]), W(c[2]), ghost);
        }
        break;
    case Tool::Circle:
        if (m_op && pk.valid()) {
            const vec3 n = plane_normal(m_op_view);
            vec3 d = pk.p - m_op_a;
            d -= n * dot(d, n);
            const float rad = length(d);
            if (rad > 1e-4f) {
                const vec3 e1 = d / rad, e2 = cross(n, e1);
                const int sides = std::clamp(m_circle_sides, 3, 64);
                for (int k = 0; k < sides; k++) {
                    const float a0 = 2 * kPi * k / sides, a1 = 2 * kPi * (k + 1) / sides;
                    thick_line(r, vf, W(m_op_a + (e1 * std::cos(a0) + e2 * std::sin(a0)) * rad), W(m_op_a + (e1 * std::cos(a1) + e2 * std::sin(a1)) * rad), white, 2);
                }
                r.line(W(m_op_a), W(m_op_a + d), ghost);
            }
        }
        break;
    case Tool::PushPull: {
        if (!m_op && m_hover_tri >= 0) {
            std::vector<int> region;
            vec3 n;
            pushpull_region(m_hover_tri, region, n);
            for (int t : region) {
                const edit::Tri& tr = M.tris[t];
                thick_line(r, vf, W(M.nodes[tr.a].p), W(M.nodes[tr.b].p), kHoverCol, 2);
                thick_line(r, vf, W(M.nodes[tr.b].p), W(M.nodes[tr.c].p), kHoverCol, 2);
                thick_line(r, vf, W(M.nodes[tr.c].p), W(M.nodes[tr.a].p), kHoverCol, 2);
            }
        }
        if (m_op) {
            const vec3 off = m_pp_n * m_op_value;
            for (int t : m_pp_region) {
                const edit::Tri& tr = M.tris[t];
                const int v[3] = {tr.a, tr.b, tr.c};
                for (int e = 0; e < 3; e++) {
                    thick_line(r, vf, W(M.nodes[v[e]].p + off), W(M.nodes[v[(e + 1) % 3]].p + off), white, 2);
                    r.line(W(M.nodes[v[e]].p), W(M.nodes[v[e]].p + off), ghost);
                }
            }
        }
        break;
    }
    case Tool::Move:
        if (m_op && pk.valid()) axis_line(m_op_a, pk.p);
        break;
    case Tool::Merge:
        // the node that goes, an arrow to where it goes
        if (!m_picks.empty() && m_picks[0] < (int)M.nodes.size()) {
            const vec3 a = W(M.nodes[m_picks[0]].p), b = pk.kind == PK_Node ? W(pk.p) : a;
            ring(r, vf, a, Renderer::rgba(1, 0.5f, 0.2f, 1), 9, 14, 2.0f);
            if (pk.kind == PK_Node && pk.node != m_picks[0]) {
                dashed(r, a, b, Renderer::rgba(1, 0.75f, 0.3f, 1), std::max(0.01f, vf.px_at(a) * 6), 2.0f);
                ring(r, vf, b, Renderer::rgba(0.4f, 1, 0.5f, 1), 11, 14, 2.5f);
            }
        }
        break;
    case Tool::Rotate:
        if (m_op_stage >= 1) {
            // the protractor in the plane of the view it was started in
            const vec3 n = plane_normal(m_op_view), c = W(m_op_a);
            vec3 u, v;
            plane_axes(n, u, v);
            const float rad = vf.px_at(c) * 60.0f;
            for (int k = 0; k < 36; k++) {
                const float a0 = 2 * kPi * k / 36, a1 = 2 * kPi * (k + 1) / 36;
                r.line(c + (u * std::cos(a0) + v * std::sin(a0)) * rad, c + (u * std::cos(a1) + v * std::sin(a1)) * rad, Renderer::rgba(0.4f, 0.7f, 1, 0.9f));
                if (k % 3 == 0) r.line(c + (u * std::cos(a0) + v * std::sin(a0)) * rad * 0.88f, c + (u * std::cos(a0) + v * std::sin(a0)) * rad, Renderer::rgba(0.4f, 0.7f, 1, 0.9f));
            }
            if (m_op_stage == 1 && pk.valid()) axis_line(m_op_a, pk.p);
            if (m_op_stage == 2) {
                thick_line(r, vf, c, W(m_op_b), ghost, 2);
                if (pk.valid()) thick_line(r, vf, c, W(pk.p), white, 2);
            }
        }
        break;
    case Tool::Scale:
        if (m_op) {
            thick_line(r, vf, W(m_op_a), W(m_op_b), ghost, 1.5f);
            if (pk.valid()) thick_line(r, vf, W(m_op_a), W(pk.p), white, 2);
            square(r, vf, W(m_op_a), white, 4);
        }
        break;
    case Tool::Tape:
        if (m_op) {
            dashed(r, W(m_tape_a), W(m_tape_b), Renderer::rgba(1, 0.9f, 0.3f, 1), std::max(0.01f, vf.px_at(W(m_tape_a)) * 6));
            ring(r, vf, W(m_tape_a), Renderer::rgba(1, 0.9f, 0.3f, 1), 4);
            ring(r, vf, W(m_tape_b), Renderer::rgba(1, 0.9f, 0.3f, 1), 4);
        }
        break;
    default: break;
    }
}

// ------------------------------------------------------------------------------------------------ the graphics
void ModelEditor::draw_graphics_binding(Renderer& r, int view) {
    if (!m_show_binding) return;
    const View& vw = m_views[view];
    const ViewFrame vf = view_frame(vw.cam, vw.rh);
    const edit::Model& M = m_model;
    auto RP = [&](int ref) { return to_world(M.ref_position(ref)); };
    auto frame = [&](int ref, int x, int y, vec3 off, vec3 rot, bool sel, const std::vector<int>* forset) {
        if (!M.ref_valid(ref) || !M.ref_valid(x) || !M.ref_valid(y)) return;
        const vec3 pr = RP(ref), px = RP(x), py = RP(y);
        vec3 pos;
        mat3 orient;
        placement(pr, px, py, off, rot, pos, orient);
        if (!sel) {
            r.line(pr, pos, Renderer::rgba(0.9f, 0.8f, 0.3f, 0.35f));
            square(r, vf, pos, Renderer::rgba(0.9f, 0.8f, 0.3f, 0.5f), 3);
            return;
        }
        // the nodes: ref (yellow), x (red), y (green); the mesh's origin and its axes
        if (forset)
            for (int n : *forset)
                if (M.ref_valid(n)) square(r, vf, RP(n), Renderer::rgba(1, 1, 1, 0.95f), 4), square(r, vf, RP(n), Renderer::rgba(0.1f, 0.1f, 0.1f, 0.9f), 5);
        thick_line(r, vf, pr, px, kAxisCol[0], 3);
        thick_line(r, vf, pr, py, kAxisCol[1], 3);
        ring(r, vf, pr, Renderer::rgba(1, 0.9f, 0.2f, 1), 9);
        ring(r, vf, px, kAxisCol[0], 7);
        ring(r, vf, py, kAxisCol[1], 7);
        dashed(r, pr, pos, Renderer::rgba(1, 0.9f, 0.2f, 1), std::max(0.01f, vf.px_at(pos) * 5));
        const float L = vf.px_at(pos) * 45.0f;
        for (int a = 0; a < 3; a++) thick_line(r, vf, pos, pos + orient.c[a] * L, kAxisCol[a], 2.5f);
        square(r, vf, pos, Renderer::rgba(1, 1, 1, 1), 5);
    };
    for (int i = 0; i < (int)M.flexbodies.size(); i++) {
        const edit::Flexbody& f = M.flexbodies[i];
        frame(f.ref, f.x, f.y, f.offset, f.rot, m_gfx_kind == 1 && m_gfx_sel == i, &f.forset);
    }
    for (int i = 0; i < (int)M.props.size(); i++) {
        const edit::Prop& p = M.props[i];
        frame(p.ref, p.x, p.y, p.offset, p.rot, m_gfx_kind == 2 && m_gfx_sel == i, nullptr);
    }
}

} // namespace bl
