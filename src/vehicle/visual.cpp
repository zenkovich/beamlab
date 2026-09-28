#include "vehicle/visual.h"
#include "core/jobs.h"
#include "core/profiler.h"
#include "core/util.h"

#include <algorithm>
#include <cmath>

namespace bl {

using namespace phys;

namespace {

struct Placement {
    vec3 pos;
    mat3 orient;
};

// RoR prop / flexbody placement from ref, x, y nodes (FlexBody.cpp / GfxActor::UpdateProps).
Placement place(vec3 pref, vec3 px, vec3 py, vec3 off, const quat& rot) {
    vec3 X = px - pref, Y = py - pref;
    vec3 normal = normalize_or(cross(Y, X), vec3(0, 1, 0));
    Placement p;
    p.pos = pref + X * off.x + Y * off.y + normal * off.z;
    vec3 rx = normalize_or(X, vec3(1, 0, 0));
    vec3 ry = cross(rx, normal);
    p.orient = mat3(rx, normal, ry) * to_mat3(rot);
    return p;
}

inline void locator_frame(const Node* nd, uint32_t ref, uint32_t nx, uint32_t ny, vec3& o, vec3& X, vec3& Y, vec3& Z) {
    o = nd[ref].p;
    X = nd[nx].p - o;
    Y = nd[ny].p - o;
    Z = normalize_or(cross(X, Y), vec3(0, 1, 0));
}

} // namespace

uint32_t VehicleVisual::alloc(uint32_t n) {
    uint32_t first = (uint32_t)m_verts.size();
    m_verts.resize(m_verts.size() + n);
    return first;
}

void VehicleVisual::add_tris(const MaterialPtr& m, const std::vector<uint32_t>& idx) {
    if (!m || idx.empty()) return;
    const int part = split_parts ? m_cur_part : -1;
    for (auto& t : m_tri_lists)
        if (t.mat == m.get() && t.part == part) {
            t.idx.insert(t.idx.end(), idx.begin(), idx.end());
            return;
        }
    m_keep.push_back(m);
    m_tri_lists.push_back({m.get(), part, idx});
}

MaterialPtr VehicleVisual::material(const std::string& name, bool double_sided) {
    std::string key = name + (double_sided ? "#ds" : "");
    auto it = m_mats.find(key);
    if (it != m_mats.end()) return it->second;
    if (name.rfind("color/", 0) == 0) { // (BeamLab: a flat colour skin, `color/r,g,b` in 0..1)
        float r = 0.6f, g = 0.6f, b = 0.62f;
        sscanf(name.c_str() + 6, "%f,%f,%f", &r, &g, &b);
        MaterialPtr m = std::make_shared<Material>();
        m->color = vec4(r, g, b, 1);
        m->double_sided = double_sided;
        m->name = name;
        m_mats[key] = m;
        return m;
    }
    const MaterialDesc* md = m_lib.find(name);
    MaterialPtr m;
    if (!md) {
        if (m_warn) m_warn->push_back("material not found: " + name);
        m = std::make_shared<Material>();
        m->color = vec4(0.6f, 0.6f, 0.62f, 1);
        m->double_sided = double_sided;
        m->name = name;
    } else if (md->invisible) {
        m = nullptr;
    } else {
        m = std::make_shared<Material>();
        m->name = name;
        if (!md->diffuse_tex.empty()) m->diffuse = TextureCache::get().load(md->diffuse_tex);
        m->color = md->diffuse;
        if (m->diffuse && m->color.x + m->color.y + m->color.z < 0.05f) m->color = vec4(1, 1, 1, m->color.w); // black diffuse + texture: RoR shows texture
        // alpha_blend + a near-zero rejection (managed *_transparent, glass): real blending that keeps the
        // translucency; alpha_blend + a real rejection threshold (decals, grilles): alpha-tested cut-out.
        const bool glass = md->blend && md->alpha_test && md->alpha_ref < 0.05f;
        if (md->alpha_test) m->alpha_ref = glass ? std::max(1.0f / 255.0f, md->alpha_ref) : std::max(0.1f, md->alpha_ref);
        m->blend = md->blend && (!md->alpha_test || glass);
        if (m->blend && m->diffuse && !m->diffuse->has_alpha && m->color.w >= 0.99f) m->blend = false;
        if (!m->blend && m->diffuse && m->diffuse->has_alpha && !md->alpha_test) m->alpha_ref = 0.5f;
        m->double_sided = md->double_sided || double_sided;
        float spec = (md->specular.x + md->specular.y + md->specular.z) / 3.0f;
        m->specular = spec > 0.01f ? std::min(1.0f, spec) : 0.25f;
        m->gloss = md->shininess > 1 ? md->shininess : 32.0f;
        m->reflect = md->reflective ? 0.3f : 0.05f;
        m->emissive = md->emissive * 0.6f;
        m->unlit = !md->lighting;
        m->cast_shadow = !m->blend;
    }
    m_mats[key] = m;
    return m;
}

const OgreMesh* VehicleVisual::mesh(const std::string& file) {
    auto it = m_meshes.find(file);
    if (it != m_meshes.end()) return it->second.get();
    std::string path = find_file_ci(m_dir, file);
    std::unique_ptr<OgreMesh> m;
    if (!path.empty()) {
        m = std::make_unique<OgreMesh>();
        std::string err;
        if (!load_ogre_mesh(path, *m, &err)) {
            if (m_warn) m_warn->push_back("mesh load failed: " + file + " (" + err + ")");
            m.reset();
        }
    } else if (m_warn) {
        m_warn->push_back("mesh not found: " + file);
    }
    const OgreMesh* r = m.get();
    m_meshes[file] = std::move(m);
    return r;
}

// ================================================================================================ build
void VehicleVisual::build(const ror::Document& d, const SoftBody& b, std::vector<std::string>& warnings) {
    PROFILE_ZONE("Vehicle visual build");
    m_warn = &warnings;
    m_dir = d.dir;
    m_lib.add_ror_builtins();
    m_lib.load_dir(d.dir);
    for (const auto& mm : d.managed_materials) m_lib.add_managed(mm, d.dir);
    const Node* nd = b.nodes.data();
    const int N = b.node_count();

    // ---- cab
    // (a `sheet/...` cab material is BeamLab's sheet body, drawn by the vehicle itself: apply_vehicle_sheet_body)
    MaterialPtr cab_mat = d.cab_material.empty() || d.cab_material.rfind("sheet/", 0) == 0 ? nullptr : material(d.cab_material);
    if (cab_mat) {
        m_cab_first = (uint32_t)m_verts.size();
        MaterialPtr back_mat;
        for (size_t s = 0; s < d.submeshes.size(); s++) {
            const auto& sm = d.submeshes[s];
            if (sm.texcoords.empty()) continue;
            m_cur_part = part_code(PART_CAB, (int)s);
            std::vector<uint32_t> tris, back;
            uint32_t base = (uint32_t)m_cab_node.size();
            for (const auto& tc : sm.texcoords) {
                m_cab_node.push_back((uint32_t)std::clamp(tc.node, 0, N - 1));
                Vertex v;
                v.uv = vec2(tc.u, tc.v);
                m_verts.push_back(v);
            }
            auto corner = [&](int node) -> int {
                for (size_t k = 0; k < sm.texcoords.size(); k++)
                    if (sm.texcoords[k].node == node) return (int)(base + k);
                return -1;
            };
            for (const auto& c : sm.cabs) {
                int a = corner(c.n1), bb = corner(c.n2), cc = corner(c.n3);
                if (a < 0 || bb < 0 || cc < 0) continue;
                m_cab_tris.push_back({(uint32_t)a, (uint32_t)bb, (uint32_t)cc});
                tris.insert(tris.end(), {m_cab_first + a, m_cab_first + bb, m_cab_first + cc});
                if (sm.backmesh) back.insert(back.end(), {m_cab_first + bb, m_cab_first + a, m_cab_first + cc});
            }
            add_tris(cab_mat, tris);
            if (!back.empty()) {
                if (!back_mat) {
                    back_mat = std::make_shared<Material>();
                    back_mat->color = vec4(0.05f, 0.05f, 0.055f, 1);
                    back_mat->specular = 0.1f;
                    back_mat->name = "cab-back";
                }
                add_tris(back_mat, back);
            }
        }
        m_cab_count = (uint32_t)m_verts.size() - m_cab_first;
    }

    // ---- flexbodies
    for (size_t fi = 0; fi < d.flexbodies.size(); fi++) {
        const auto& fb = d.flexbodies[fi];
        m_cur_part = part_code(PART_FLEX, (int)fi);
        const OgreMesh* om = mesh(fb.mesh);
        if (!om) continue;
        if (fb.ref >= N || fb.x >= N || fb.y >= N) continue;
        quat rot = quat_euler_xyz_deg(fb.rot.x, fb.rot.y, fb.rot.z);
        Placement pl = place(nd[fb.ref].p, nd[fb.x].p, nd[fb.y].p, fb.offset, rot);
        std::vector<uint32_t> forset;
        for (int n : fb.forset)
            if (n >= 0 && n < N) forset.push_back((uint32_t)n);
        std::sort(forset.begin(), forset.end());
        forset.erase(std::unique(forset.begin(), forset.end()), forset.end());
        bool rigid = forset.size() < 3;
        if (rigid && !fb.forset.empty()) warnings.push_back("flexbody " + fb.mesh + ": forset too small, attached rigidly");
        size_t total = 0;
        for (const auto& sm : om->submeshes) total += sm.vertices.size();
        Flex fx;
        fx.first = alloc((uint32_t)total);
        fx.count = (uint32_t)total;
        fx.loc.resize(total);
        // place vertices in definition space
        std::vector<vec3> wp(total), wn(total);
        size_t k = 0;
        for (const auto& sm : om->submeshes) {
            MaterialPtr m = material(sm.material);
            std::vector<uint32_t> idx;
            idx.reserve(sm.indices.size());
            for (uint32_t i : sm.indices) idx.push_back(fx.first + (uint32_t)k + i);
            add_tris(m, idx);
            for (const auto& v : sm.vertices) {
                wp[k] = pl.orient * v.pos + pl.pos;
                wn[k] = normalize_or(pl.orient * v.normal, vec3(0, 1, 0));
                m_verts[fx.first + k].uv = v.uv;
                k++;
            }
        }
        // bind each vertex to (ref, nx, ny) nodes (RoR FlexBody locator search)
        const float cos_lim = 0.70710678f;
        JobSystem::get().parallel_for((int)total, 256, [&](int b0, int b1, int) {
            for (int i = b0; i < b1; i++) {
                Locator L;
                if (rigid) {
                    L.ref = fb.ref;
                    L.nx = fb.x;
                    L.ny = fb.y;
                } else {
                    float d0 = 1e30f, d1 = 1e30f;
                    uint32_t r0 = forset[0], r1 = forset[1];
                    for (uint32_t n : forset) {
                        float dd = length2(nd[n].p - wp[i]);
                        if (dd < d0) {
                            d1 = d0;
                            r1 = r0;
                            d0 = dd;
                            r0 = n;
                        } else if (dd < d1) {
                            d1 = dd;
                            r1 = n;
                        }
                    }
                    vec3 vx = normalize_or(nd[r1].p - nd[r0].p, vec3(1, 0, 0));
                    float d2 = 1e30f;
                    uint32_t r2 = UINT32_MAX;
                    for (uint32_t n : forset) {
                        if (n == r0 || n == r1) continue;
                        float dd = length2(nd[n].p - wp[i]);
                        if (dd >= d2) continue;
                        vec3 vy = normalize_or(nd[n].p - nd[r0].p, vec3(0, 1, 0));
                        if (std::fabs(dot(vx, vy)) <= cos_lim) {
                            d2 = dd;
                            r2 = n;
                        }
                    }
                    if (r2 == UINT32_MAX) {
                        // no well conditioned third node: fall back to the flexbody frame
                        r0 = fb.ref;
                        r1 = fb.x;
                        r2 = fb.y;
                    }
                    L.ref = r0;
                    L.nx = r1;
                    L.ny = r2;
                }
                vec3 o, X, Y, Z;
                locator_frame(nd, L.ref, L.nx, L.ny, o, X, Y, Z);
                mat3 Minv = inverse(mat3(X, Y, Z));
                L.c = Minv * (wp[i] - o);
                L.n = Minv * wn[i];
                fx.loc[i] = L;
            }
        });
        m_flex.push_back(std::move(fx));
    }

    // ---- props (rigid meshes following ref/x/y nodes)
    for (size_t pi = 0; pi < d.props.size(); pi++) {
        const auto& p = d.props[pi];
        if (p.ref >= N || p.x >= N || p.y >= N) continue;
        const OgreMesh* om = mesh(p.mesh);
        if (!om) continue;
        Rigid r;
        r.kind = Rigid::PROP;
        r.ref = p.ref;
        r.x = p.x;
        r.y = p.y;
        r.off = p.offset;
        r.rot = quat_euler_xyz_deg(p.rot.x, p.rot.y, p.rot.z);
        const bool seat = p.special == ror::PropDef::SEAT;
        r.mesh = rigid_mesh(om, p.mesh + (seat ? "#seat" : ""), seat ? "driversseat" : "");
        r.part = part_code(PART_PROP, (int)pi);
        m_rigid.push_back(std::move(r));
        // dashboard steering wheel (only when the mod ships the wheel mesh)
        if ((p.special == ror::PropDef::DASHBOARD || p.special == ror::PropDef::DASHBOARD_RH)) {
            std::string wm = p.wheel_mesh.empty() ? std::string("dirwheel.mesh") : p.wheel_mesh;
            if (!find_file_ci(m_dir, wm).empty())
                if (const OgreMesh* wmesh = mesh(wm)) {
                    Rigid s;
                    s.kind = Rigid::STEERING_WHEEL;
                    s.ref = p.ref;
                    s.x = p.x;
                    s.y = p.y;
                    s.off = p.offset;
                    s.rot = quat_euler_xyz_deg(p.rot.x, p.rot.y, p.rot.z);
                    s.wheel_offset = p.has_wheel_offset ? p.wheel_offset : vec3(p.special == ror::PropDef::DASHBOARD_RH ? 0.67f : -0.67f, -0.61f, 0.24f);
                    s.wheel_angle = p.wheel_angle;
                    s.mesh = rigid_mesh(wmesh, wm, "");
                    s.part = part_code(PART_PROP, (int)pi);
                    m_rigid.push_back(std::move(s));
                }
        }
    }

    // ---- wheels
    for (int wi = 0; wi < (int)b.wheels.size(); wi++) {
        const Wheel& w = b.wheels[wi];
        if (w.tag < 0 || w.tag >= (int)d.wheels.size()) continue;
        const ror::WheelDef& wd = d.wheels[w.tag];
        const int n = (int)w.nodes.size() / 2;
        if (n < 3) continue;
        m_cur_part = part_code(PART_WHEEL, w.tag);
        if (wd.type == ror::WheelDef::WHEELS || wd.type == ror::WheelDef::WHEELS2) {
            bool w2 = wd.type == ror::WheelDef::WHEELS2;
            MaterialPtr face = material(wd.face_material, true), band = material(wd.band_material, true);
            Tyre t;
            t.wheel = wi;
            t.type = 0;
            t.rays = n;
            t.wheels2 = w2;
            t.rim_radius = wd.rim_radius;
            // layout: 2 hubs, face ring outer (n), face ring inner (n), band (2n) [, tyre face outer (n), inner (n)]
            uint32_t count = 2 + 2 * n + 2 * n + (w2 ? 2 * n : 0);
            t.first = alloc(count);
            uint32_t f = t.first;
            std::vector<uint32_t> fi, bi;
            m_verts[f].uv = m_verts[f + 1].uv = vec2(0.5f, 0.5f);
            for (int i = 0; i < n; i++) {
                float a = 2 * kPi * i / n;
                float rr = w2 ? 0.5f * wd.rim_radius / std::max(0.01f, wd.radius) : 0.5f;
                vec2 uv(0.5f + rr * std::sin(a), 0.5f + rr * std::cos(a));
                m_verts[f + 2 + i].uv = uv;
                m_verts[f + 2 + n + i].uv = uv;
                m_verts[f + 2 + 2 * n + 2 * i].uv = vec2((float)(i & 1), 0);
                m_verts[f + 2 + 2 * n + 2 * i + 1].uv = vec2((float)(i & 1), 1);
                if (w2) {
                    float a2 = 2 * kPi * (i + 0.5f) / n;
                    vec2 uv2(0.5f + 0.5f * std::sin(a2), 0.5f + 0.5f * std::cos(a2));
                    m_verts[f + 2 + 4 * n + i].uv = uv2;
                    m_verts[f + 2 + 5 * n + i].uv = uv2;
                }
            }
            for (int i = 0; i < n; i++) {
                int j = (i + 1) % n;
                uint32_t fo = f + 2, fin = f + 2 + n, bb = f + 2 + 2 * n;
                fi.insert(fi.end(), {f, fo + i, fo + j});
                fi.insert(fi.end(), {f + 1, fin + j, fin + i});
                if (w2) {
                    uint32_t to = f + 2 + 4 * n, ti = f + 2 + 5 * n;
                    fi.insert(fi.end(), {fo + i, to + i, fo + j, to + i, to + j, fo + j});
                    fi.insert(fi.end(), {fin + j, ti + i, fin + i, fin + j, ti + j, ti + i});
                }
                bi.insert(bi.end(), {bb + 2 * i, bb + 2 * i + 1, bb + 2 * j, bb + 2 * j, bb + 2 * i + 1, bb + 2 * j + 1});
            }
            add_tris(face, fi);
            add_tris(band, bi);
            m_tyres.push_back(t);
        } else {
            // meshwheels*/flexbodywheels: rim mesh + generated tyre
            if (!wd.rim_mesh.empty())
                if (const OgreMesh* om = mesh(wd.rim_mesh)) {
                    Rigid r;
                    r.kind = Rigid::RIM;
                    r.wheel = wi;
                    r.side = wd.side;
                    r.mesh = rigid_mesh(om, wd.rim_mesh, "");
                    r.part = part_code(PART_WHEEL, w.tag);
                    m_rigid.push_back(std::move(r));
                }
            std::string tmat = wd.type == ror::WheelDef::FLEXBODYWHEELS ? std::string() : wd.tyre_material;
            MaterialPtr tm = tmat.empty() ? material("tracks/wheelband", true) : material(tmat, true);
            Tyre t;
            t.wheel = wi;
            t.type = 1;
            t.rays = n;
            t.rim_radius = wd.rim_radius;
            t.wheels2 = false;
            t.first = alloc(6 * (n + 1));
            static const float vv[6] = {0, 0.23f, 0.27f, 0.73f, 0.77f, 1};
            std::vector<uint32_t> ti;
            for (int i = 0; i <= n; i++)
                for (int k = 0; k < 6; k++) m_verts[t.first + i * 6 + k].uv = vec2((float)i / n, vv[k]);
            for (int i = 0; i < n; i++)
                for (int k = 0; k < 5; k++) {
                    uint32_t a = t.first + i * 6 + k, b2 = a + 1, c = a + 6, dd = a + 7;
                    ti.insert(ti.end(), {a, c, b2, b2, c, dd});
                }
            add_tris(tm, ti);
            m_tyres.push_back(t);
        }
    }

    // ---- index buffer grouped by material (opaque first)
    std::stable_sort(m_tri_lists.begin(), m_tri_lists.end(), [](const auto& a, const auto& b) { return !a.mat->blend && b.mat->blend; });
    for (auto& tl : m_tri_lists) {
        Batch bt;
        bt.mat = tl.mat;
        bt.first = (int)m_indices.size();
        bt.count = (int)tl.idx.size();
        bt.part = tl.part;
        m_indices.insert(m_indices.end(), tl.idx.begin(), tl.idx.end());
        m_batches.push_back(bt);
    }
    m_cur_part = -1;
    m_warn = nullptr;
    // initial pose
    update(b, 0.0f);
}

// ================================================================================================ update
void VehicleVisual::update(const SoftBody& b, float dir_state) {
    if (b.sleeping && m_uploaded && !m_need_update) return;
    m_need_update = false;
    // culling bounds: the body's box plus a margin for meshes that stick out of the node cloud (mirrors, bumpers)
    m_bounds = b.aabb;
    m_bounds.expand(1.0f);
    const Node* nd = b.nodes.data();
    Vertex* V = m_verts.data();
    // cab: node positions + smooth normals
    if (m_cab_count) {
        for (uint32_t i = 0; i < m_cab_count; i++) {
            V[m_cab_first + i].pos = nd[m_cab_node[i]].p;
            V[m_cab_first + i].normal = vec3(0);
        }
        for (const CabTri& t : m_cab_tris) {
            Vertex &a = V[m_cab_first + t.a], &bb = V[m_cab_first + t.b], &c = V[m_cab_first + t.c];
            vec3 n = normalize_or(cross(bb.pos - a.pos, c.pos - a.pos), vec3(0));
            a.normal += n;
            bb.normal += n;
            c.normal += n;
        }
        for (uint32_t i = 0; i < m_cab_count; i++) V[m_cab_first + i].normal = normalize_or(V[m_cab_first + i].normal, vec3(0, 1, 0));
    }
    // flexbodies
    for (const Flex& f : m_flex) {
        Vertex* out = V + f.first;
        const Locator* L = f.loc.data();
        for (uint32_t i = 0; i < f.count; i++) {
            const Locator& l = L[i];
            vec3 o = nd[l.ref].p;
            vec3 X = nd[l.nx].p - o, Y = nd[l.ny].p - o;
            vec3 Z = cross(X, Y);
            float zl = length(Z);
            Z = zl > 1e-9f ? Z / zl : vec3(0, 1, 0);
            out[i].pos = o + X * l.c.x + Y * l.c.y + Z * l.c.z;
            out[i].normal = normalize_or(X * l.n.x + Y * l.n.y + Z * l.n.z, vec3(0, 1, 0));
        }
    }
    // rigid parts
    for (Rigid& r : m_rigid) {
        Placement pl;
        if (r.kind == Rigid::RIM) {
            const Wheel& w = b.wheels[r.wheel];
            vec3 a0 = nd[w.axle0].p, a1 = nd[w.axle1].p;
            pl.pos = (a0 + a1) * 0.5f;
            vec3 a = normalize_or(a0 - a1, vec3(0, 0, 1));
            if (r.side != 'r') a = -a;
            uint32_t start = w.rim.empty() ? w.nodes[0] : w.rim[0];
            vec3 ray = nd[start].p - a0;
            vec3 o = normalize_or(cross(a, ray), vec3(0, 1, 0));
            vec3 z = cross(a, o);
            pl.orient = mat3(a, o, z);
        } else {
            pl = place(nd[r.ref].p, nd[r.x].p, nd[r.y].p, r.off, r.rot);
            if (r.kind == Rigid::STEERING_WHEEL) {
                pl.pos = pl.pos + pl.orient * r.wheel_offset;
                pl.orient = pl.orient * to_mat3(quat::axis_angle(vec3(1, 0, 0), -59.0f * kDeg2Rad) *
                                                 quat::axis_angle(vec3(0, 1, 0), dir_state * r.wheel_angle * kDeg2Rad));
            }
        }
        r.model = mat4::from_mat3(pl.orient, pl.pos);
    }
    // tyres
    for (const Tyre& t : m_tyres) {
        const Wheel& w = b.wheels[t.wheel];
        vec3 a0 = nd[w.axle0].p, a1 = nd[w.axle1].p;
        vec3 axis = normalize_or(a1 - a0, vec3(0, 0, 1));
        const int n = t.rays;
        Vertex* out = V + t.first;
        if (t.type == 0) {
            out[0].pos = a0;
            out[1].pos = a1;
            out[0].normal = -axis;
            out[1].normal = axis;
            for (int i = 0; i < n; i++) {
                uint32_t fo, fi2;
                if (t.wheels2 && !w.rim.empty()) {
                    fo = w.rim[2 * i];
                    fi2 = w.rim[2 * i + 1];
                } else {
                    fo = w.nodes[2 * i];
                    fi2 = w.nodes[2 * i + 1];
                }
                out[2 + i].pos = nd[fo].p;
                out[2 + i].normal = -axis;
                out[2 + n + i].pos = nd[fi2].p;
                out[2 + n + i].normal = axis;
                vec3 po = nd[w.nodes[2 * i]].p, pi = nd[w.nodes[2 * i + 1]].p;
                out[2 + 2 * n + 2 * i].pos = po;
                out[2 + 2 * n + 2 * i + 1].pos = pi;
                vec3 c = (a0 + a1) * 0.5f;
                vec3 rn = po - c;
                rn = normalize_or(rn - axis * dot(rn, axis), vec3(0, 1, 0));
                out[2 + 2 * n + 2 * i].normal = rn;
                out[2 + 2 * n + 2 * i + 1].normal = rn;
                if (t.wheels2) {
                    out[2 + 4 * n + i].pos = po;
                    out[2 + 4 * n + i].normal = -axis;
                    out[2 + 5 * n + i].pos = pi;
                    out[2 + 5 * n + i].normal = axis;
                }
            }
        } else {
            float rim_r = t.rim_radius;
            for (int i = 0; i <= n; i++) {
                int ii = i % n;
                vec3 o = nd[w.nodes[2 * ii]].p, q = nd[w.nodes[2 * ii + 1]].p;
                vec3 ro = o - a0, rq = q - a1;
                ro = normalize_or(ro - axis * dot(ro, axis), vec3(0, 1, 0));
                rq = normalize_or(rq - axis * dot(rq, axis), vec3(0, 1, 0));
                Vertex* rv = out + i * 6;
                rv[0].pos = a0 + ro * rim_r;
                rv[1].pos = o - (o - a0) * 0.05f;
                rv[2].pos = o - (o - q) * 0.1f;
                rv[3].pos = q - (q - o) * 0.1f;
                rv[4].pos = q - (q - a1) * 0.05f;
                rv[5].pos = a1 + rq * rim_r;
                vec3 radial = normalize_or(ro + rq, vec3(0, 1, 0));
                rv[0].normal = -axis;
                rv[1].normal = normalize(radial * 0.6f - axis * 0.8f);
                rv[2].normal = radial;
                rv[3].normal = radial;
                rv[4].normal = normalize(radial * 0.6f + axis * 0.8f);
                rv[5].normal = axis;
            }
        }
    }
    m_dirty = true;
}

int VehicleVisual::rigid_mesh(const OgreMesh* om, const std::string& key, const std::string& material_override) {
    auto it = m_rmesh_index.find(key);
    if (it != m_rmesh_index.end()) return it->second;
    auto rm = std::make_unique<RigidMesh>();
    std::vector<std::pair<const Material*, std::vector<uint32_t>>> lists;
    for (const auto& sm : om->submeshes) {
        MaterialPtr m = material(material_override.empty() ? sm.material : material_override);
        uint32_t base = (uint32_t)rm->verts.size();
        for (const auto& v : sm.vertices) rm->verts.push_back({v.pos, v.normal, v.uv});
        if (!m) continue; // invisible material
        m_keep.push_back(m);
        auto li = std::find_if(lists.begin(), lists.end(), [&](const auto& l) { return l.first == m.get(); });
        if (li == lists.end()) {
            lists.push_back({m.get(), {}});
            li = lists.end() - 1;
        }
        for (uint32_t i : sm.indices) li->second.push_back(base + i);
    }
    std::stable_sort(lists.begin(), lists.end(), [](const auto& a, const auto& b) { return !a.first->blend && b.first->blend; });
    for (auto& l : lists) {
        rm->batches.push_back({l.first, (int)rm->idx.size(), (int)l.second.size()});
        rm->idx.insert(rm->idx.end(), l.second.begin(), l.second.end());
    }
    int index = (int)m_rmeshes.size();
    m_rmeshes.push_back(std::move(rm));
    m_rmesh_index[key] = index;
    return index;
}

const Material* VehicleVisual::ghost_of(const Material* m, float alpha) {
    if (!m || alpha >= 0.999f) return m;
    auto& g = m_ghost[m];
    if (!g) g = std::make_unique<Material>(*m);
    g->blend = true;
    g->alpha_ref = 0.0f;
    g->cast_shadow = false;
    g->double_sided = true;
    g->color = vec4(m->color.x, m->color.y, m->color.z, m->color.w * alpha);
    return g.get();
}

const Material* VehicleVisual::variant(const Material* m, int mode, float alpha) {
    // 0 selected: tinted orange, glowing a little, opaque; 1 under the mouse: tinted light blue; 2 faint: grey, see-through
    auto& g = m_variant[mode][m];
    if (!g) g = std::make_unique<Material>(*m);
    g->cast_shadow = false;
    g->double_sided = true;
    auto mix = [](vec3 a, vec3 b, float t) { return a + (b - a) * t; };
    const vec3 c(m->color.x, m->color.y, m->color.z);
    if (mode == 0) {
        g->color = vec4(mix(c, vec3(1.0f, 0.55f, 0.12f), 0.5f), m->color.w);
        g->emissive = vec3(0.35f, 0.16f, 0.02f);
        g->blend = m->blend;
    } else if (mode == 1) {
        g->color = vec4(mix(c, vec3(0.55f, 0.85f, 1.0f), 0.45f), m->color.w * alpha);
        g->emissive = vec3(0.05f, 0.12f, 0.2f);
        g->blend = alpha < 0.999f || m->blend;
        g->alpha_ref = g->blend ? 0.0f : m->alpha_ref;
    } else {
        g->color = vec4(mix(c, vec3(0.55f, 0.56f, 0.6f), 0.7f), m->color.w * alpha);
        g->emissive = vec3(0);
        g->blend = true;
        g->alpha_ref = 0.0f;
    }
    return g.get();
}

const Material* VehicleVisual::part_material(const Material* m, int part, float alpha) {
    if (!m) return nullptr;
    uint8_t st = PS_NORMAL;
    if (part >= 0 && !m_state.empty()) {
        auto it = m_state.find(part);
        if (it != m_state.end()) st = it->second;
    }
    switch (st) {
    case PS_HIDDEN: return nullptr;
    case PS_SELECTED: return variant(m, 0, 1.0f);
    case PS_HOVER: return variant(m, 1, std::max(alpha, 0.85f));
    case PS_FAINT: return variant(m, 2, std::max(0.08f, alpha * 0.25f));
    default: {
        const float a = alpha * others_alpha;
        return a <= 0.001f ? nullptr : ghost_of(m, a);
    }
    }
}

void VehicleVisual::draw(Renderer& r, float alpha) {
    if (alpha <= 0.001f && m_state.empty()) return;
    for (const Rigid& rg : m_rigid) {
        if (rg.mesh < 0) continue;
        RigidMesh& rm = *m_rmeshes[(size_t)rg.mesh];
        if (rm.idx.empty()) continue;
        if (!rm.ready) {
            rm.gpu.create(rm.verts, rm.idx, false);
            rm.ready = true;
        }
        for (const Batch& b : rm.batches)
            if (const Material* mat = part_material(b.mat, rg.part, alpha)) r.draw_mesh(&rm.gpu, mat, rg.model, b.first, b.count);
    }
    if (m_verts.empty() || m_indices.empty()) return;
    if (!m_gpu_ready) {
        m_gpu.create(m_verts, m_indices, true);
        m_gpu_ready = true;
        m_dirty = false;
        m_uploaded = true;
    } else if (m_dirty) {
        m_gpu.update_vertices(m_verts.data(), (int)m_verts.size(), m_bounds);
        m_dirty = false;
        m_uploaded = true;
    }
    for (const Batch& b : m_batches)
        if (const Material* mat = part_material(b.mat, b.part, alpha)) r.draw_mesh(&m_gpu, mat, mat4(), b.first, b.count);
}

namespace {
bool hit_tri(vec3 ro, vec3 rd, vec3 a, vec3 b, vec3 c, float& t) {
    const vec3 e1 = b - a, e2 = c - a, p = cross(rd, e2);
    const float det = dot(e1, p);
    if (std::fabs(det) < 1e-14f) return false;
    const float inv = 1.0f / det;
    const vec3 s = ro - a;
    const float u = dot(s, p) * inv;
    if (u < 0 || u > 1) return false;
    const vec3 q = cross(s, e1);
    const float v = dot(rd, q) * inv;
    if (v < 0 || u + v > 1) return false;
    t = dot(e2, q) * inv;
    return t > 0;
}
} // namespace

int VehicleVisual::pick(vec3 ro, vec3 rd, float* t_out) const {
    auto hidden = [&](int part) {
        if (part < 0) return false;
        auto it = m_state.find(part);
        return it != m_state.end() && it->second == PS_HIDDEN;
    };
    float best = 1e30f;
    int code = -1;
    for (const Batch& b : m_batches) {
        if (b.part < 0 || hidden(b.part)) continue;
        for (int i = b.first; i + 2 < b.first + b.count; i += 3) {
            float t;
            if (hit_tri(ro, rd, m_verts[m_indices[i]].pos, m_verts[m_indices[i + 1]].pos, m_verts[m_indices[i + 2]].pos, t) && t < best) best = t, code = b.part;
        }
    }
    for (const Rigid& rg : m_rigid) {
        if (rg.mesh < 0 || rg.part < 0 || hidden(rg.part)) continue;
        const RigidMesh& rm = *m_rmeshes[(size_t)rg.mesh];
        // (the ray in the mesh's own space: a rotation and a translation, distances unchanged)
        const mat3 R = rg.model.upper3();
        const mat3 Rt = transpose(R);
        const vec3 lo = Rt * (ro - rg.model.translation()), ld = Rt * rd;
        for (const Batch& b : rm.batches)
            for (int i = b.first; i + 2 < b.first + b.count; i += 3) {
                float t;
                if (hit_tri(lo, ld, rm.verts[rm.idx[i]].pos, rm.verts[rm.idx[i + 1]].pos, rm.verts[rm.idx[i + 2]].pos, t) && t < best) best = t, code = rg.part;
            }
    }
    if (t_out) *t_out = best;
    return code;
}

} // namespace bl
