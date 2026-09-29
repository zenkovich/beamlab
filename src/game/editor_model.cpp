#include "game/editor_model.h"
#include "phys/frame_fem.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <tuple>
#include <set>
#include <sstream>

namespace bl::edit {

namespace {

constexpr float kTwinEps = 1e-3f;

bool has_opt(const std::string& s, char c) { return s.find(c) != std::string::npos; }

std::string fmt(const char* f, double v) {
    char buf[64];
    snprintf(buf, sizeof buf, f, v);
    return buf;
}
std::string fmt(const char* f, int v) {
    char buf[64];
    snprintf(buf, sizeof buf, f, v);
    return buf;
}

// RoR's "never": a number beyond the float range means never deforms / never breaks
const char* kNever = "99999999999999999999999999999999999999999";

std::string num(float v) {
    if (v >= 1e30f) return kNever;
    if (v == std::floor(v) && std::fabs(v) < 1e9f) return fmt("%.0f", v);
    return fmt("%g", v);
}

std::string defaults_line(const ElemDefaults& b) {
    return "set_beam_defaults " + num(b.spring) + ", " + num(b.damp) + ", " + num(b.deform >= 1e29f ? 1e30f : b.deform) + ", " + num(b.brk >= 1e29f ? 1e30f : b.brk) +
           ", 0.05, tracks/beam, " + num(std::max(0.0f, b.plastic)) + "\n";
}

// scaled: the values as set_beam_defaults_scale makes them (the builder scales a hydro's; a shock's stops and a
// wheel's beams take the raw ones)
ElemDefaults defaults_of(const ror::BeamDefaults& d, bool scaled) {
    ElemDefaults e;
    e.spring = scaled ? d.k() : d.spring, e.damp = scaled ? d.d() : d.damp;
    const float brk = scaled ? d.break_scaled() : d.brk;
    e.brk = brk >= 1e29f ? 1e30f : brk, e.plastic = std::max(0.0f, d.plastic);
    e.deform = d.deform_threshold() >= 1e29f ? 1e30f : d.deform_threshold();
    return e;
}

void remap_elements(Model& m, const std::vector<int>& map) {
    auto ok = [&](int n) { return n >= 0 && n < (int)map.size() && map[n] >= 0; };
    std::vector<Beam> beams;
    for (Beam b : m.beams)
        if (ok(b.a) && ok(b.b)) {
            b.a = map[b.a];
            b.b = map[b.b];
            beams.push_back(b);
        }
    m.beams.swap(beams);
    std::vector<Shock> shocks;
    for (Shock s : m.shocks)
        if (ok(s.a) && ok(s.b)) {
            s.a = map[s.a];
            s.b = map[s.b];
            shocks.push_back(s);
        }
    m.shocks.swap(shocks);
    std::vector<Hydro> hydros;
    for (Hydro h : m.hydros)
        if (ok(h.a) && ok(h.b)) {
            h.a = map[h.a];
            h.b = map[h.b];
            hydros.push_back(h);
        }
    m.hydros.swap(hydros);
    std::vector<Tri> tris;
    for (Tri t : m.tris)
        if (ok(t.a) && ok(t.b) && ok(t.c)) {
            t.a = map[t.a];
            t.b = map[t.b];
            t.c = map[t.c];
            tris.push_back(t);
        }
    m.tris.swap(tris);
    std::vector<Wheel> wheels;
    std::vector<int> wmap(m.wheels.size(), -1);
    for (size_t i = 0; i < m.wheels.size(); i++) {
        Wheel w = m.wheels[i];
        if (ok(w.n1) && ok(w.n2)) {
            w.n1 = map[w.n1];
            w.n2 = map[w.n2];
            w.arm = ok(w.arm) ? map[w.arm] : -1;
            w.rigidity = ok(w.rigidity) ? map[w.rigidity] : -1;
            wmap[i] = (int)wheels.size();
            wheels.push_back(w);
        }
    }
    m.wheels.swap(wheels);
    // graphics references: explicit nodes through the node map, a wheel's nodes through the wheel map
    auto gmap = [&](int n) {
        if (is_wheel_ref(n)) {
            const int w = wheel_of_ref(n);
            return w >= 0 && w < (int)wmap.size() && wmap[w] >= 0 ? wheel_ref(wmap[w], wheel_node_of_ref(n)) : -1;
        }
        return ok(n) ? map[n] : -1;
    };
    std::vector<Joint> joints;
    for (Joint j : m.joints)
        if (ok(j.parent) && ok(j.child)) {
            j.parent = map[j.parent];
            j.child = map[j.child];
            joints.push_back(j);
        }
    m.joints.swap(joints);
    std::vector<Weld> welds;
    for (Weld w : m.welds)
        if (ok(w.anchor) && ok(w.node)) {
            w.anchor = map[w.anchor], w.node = map[w.node];
            w.anchor2 = ok(w.anchor2) ? map[w.anchor2] : -1;
            welds.push_back(w);
        }
    m.welds.swap(welds);
    std::vector<Mount> mounts;
    for (Mount mt : m.mounts)
        if (ok(mt.a) && ok(mt.b) && (mt.kind != 'h' || ok(mt.b2))) {
            mt.a = map[mt.a], mt.b = map[mt.b];
            if (mt.kind == 'h') mt.b2 = map[mt.b2];
            mounts.push_back(mt);
        }
    m.mounts.swap(mounts);
    std::vector<Volume> volumes;
    for (Volume v : m.volumes) {
        std::vector<int> an;
        for (int a : v.anchors)
            if (ok(a)) an.push_back(map[a]);
        if (an.size() < 3) continue; // (too few anchors left: the volume goes)
        v.anchors = an;
        volumes.push_back(v);
    }
    m.volumes.swap(volumes);
    std::vector<SlideNode> slides;
    for (SlideNode sn : m.slidenodes) {
        if (!ok(sn.node)) continue;
        std::vector<int> rail;
        for (int r : sn.rail)
            if (ok(r)) rail.push_back(map[r]);
        if (rail.size() < 2) continue;
        sn.node = map[sn.node], sn.rail.swap(rail);
        slides.push_back(sn);
    }
    m.slidenodes.swap(slides);
    std::vector<Flexbody> fbs;
    for (Flexbody f : m.flexbodies)
        if (gmap(f.ref) >= 0 && gmap(f.x) >= 0 && gmap(f.y) >= 0) {
            f.ref = gmap(f.ref), f.x = gmap(f.x), f.y = gmap(f.y);
            std::vector<int> fs;
            for (int n : f.forset)
                if (gmap(n) >= 0) fs.push_back(gmap(n));
            f.forset.swap(fs);
            fbs.push_back(f);
        }
    m.flexbodies.swap(fbs);
    std::vector<Prop> props;
    for (Prop pr : m.props)
        if (gmap(pr.ref) >= 0 && gmap(pr.x) >= 0 && gmap(pr.y) >= 0) {
            pr.ref = gmap(pr.ref), pr.x = gmap(pr.x), pr.y = gmap(pr.y);
            props.push_back(pr);
        }
    m.props.swap(props);
    for (Submesh& sm : m.submeshes) {
        std::vector<std::pair<int, vec2>> tc;
        for (auto& [n, uv] : sm.texcoords)
            if (ok(n)) tc.push_back({map[n], uv});
        sm.texcoords.swap(tc);
    }
    auto cam = [&](int& c) { c = ok(c) ? map[c] : -1; };
    cam(m.cam_center);
    cam(m.cam_back);
    cam(m.cam_left);
}

} // namespace

Model::Model() {
    // (stiffness a node can take ~ its mass: k dt^2 / m below ~0.45 summed over its beams at 0.5 ms; a 10 kg node
    // carries about six 3e6 N/m beams. RoR's 9e6 default needs its 50 kg nodes)
    auto preset = [&](const char* name, float k, float d, float deform, float brk, float plastic, int type, bool invisible, bool hold, vec4 color) {
        BeamGroup g;
        g.name = name, g.spring = k, g.damp = d, g.deform = deform, g.brk = brk, g.plastic = plastic;
        g.type = type, g.invisible = invisible, g.hold_rotation = hold, g.color = color;
        groups.push_back(g);
    };
    preset("Frame", 3.0e6f, 400.0f, 8.0e4f, 7.0e5f, 0.0f, BEAM_NORMAL, false, false, vec4(0.85f, 0.85f, 0.85f, 1));
    // a frame element: a welded 40 x 2 mm steel tube (FEM, rigid joints)
    preset("Steel tube 40x2", 3.0e6f, 400.0f, 8.0e4f, 7.0e5f, 0.0f, BEAM_FRAME, false, false, vec4(0.55f, 0.72f, 1.0f, 1));
    preset("Soft", 8.0e5f, 200.0f, 3.0e4f, 3.0e5f, 0.0f, BEAM_NORMAL, false, false, vec4(0.55f, 0.85f, 1.0f, 1));
    preset("Plastic", 3.0e6f, 300.0f, 3.0e4f, 1.0e30f, 0.05f, BEAM_NORMAL, false, false, vec4(1.0f, 0.75f, 0.3f, 1));
    preset("Stiff", 6.0e6f, 800.0f, 1.0e30f, 1.0e30f, 0.0f, BEAM_NORMAL, false, false, vec4(1.0f, 0.4f, 0.4f, 1));
    preset("Rigid joint", 3.0e6f, 400.0f, 8.0e4f, 7.0e5f, 0.0f, BEAM_NORMAL, false, true, vec4(0.45f, 1.0f, 0.55f, 1));
    preset("Hidden", 3.0e6f, 400.0f, 8.0e4f, 7.0e5f, 0.0f, BEAM_NORMAL, true, false, vec4(0.5f, 0.5f, 0.5f, 1));
    preset("Rope", 1.0e5f, 200.0f, 1.0e30f, 1.0e5f, 0.0f, BEAM_ROPE, false, false, vec4(0.9f, 0.8f, 0.4f, 1));
}

int Model::node_uses(int n) const {
    int c = 0;
    for (const Beam& b : beams) c += (b.a == n || b.b == n);
    for (const Shock& s : shocks) c += (s.a == n || s.b == n);
    for (const Hydro& h : hydros) c += (h.a == n || h.b == n);
    for (const Tri& t : tris) c += (t.a == n || t.b == n || t.c == n);
    for (const Wheel& w : wheels) c += (w.n1 == n || w.n2 == n || w.arm == n);
    for (const Joint& j : joints) c += (j.parent == n || j.child == n);
    for (const Weld& w : welds) c += (w.anchor == n || w.node == n || w.anchor2 == n);
    for (const Mount& mt : mounts) c += (mt.a == n || mt.b == n || (mt.kind == 'h' && mt.b2 == n));
    for (const Volume& v : volumes) c += (int)std::count(v.anchors.begin(), v.anchors.end(), n);
    for (const SlideNode& sn : slidenodes) c += sn.node == n || std::find(sn.rail.begin(), sn.rail.end(), n) != sn.rail.end();
    for (const Flexbody& f : flexbodies) c += (f.ref == n || f.x == n || f.y == n);
    for (const Prop& p : props) c += (p.ref == n || p.x == n || p.y == n);
    return c;
}

int Model::shell_count() const {
    int c = 0;
    for (const Tri& t : tris) c += t.shell && !t.fem;
    return c;
}

int Model::fem_count() const {
    int c = 0;
    for (const Tri& t : tris) c += t.fem;
    return c;
}

FemPreset Model::fem_preset(int i) const { return i >= 0 && i < (int)fem_presets.size() ? fem_presets[i] : FemPreset(); }

int Model::ensure_fem_preset() {
    if (fem_presets.empty()) fem_presets.push_back(FemPreset());
    return 0;
}

void Model::remove_fem_preset(int i) {
    if (i < 0 || i >= (int)fem_presets.size() || fem_presets.size() <= 1) return;
    fem_presets.erase(fem_presets.begin() + i);
    for (Tri& t : tris) t.fem_preset = t.fem_preset == i ? 0 : t.fem_preset > i ? t.fem_preset - 1 : t.fem_preset;
}

int Model::nearest_node(vec3 p, float r) const {
    int best = -1;
    float bd = r * r;
    for (int i = 0; i < (int)nodes.size(); i++) {
        const float d = length2(nodes[i].p - p);
        if (d < bd) bd = d, best = i;
    }
    return best;
}

int Model::twin(int n) const {
    if (n < 0 || n >= (int)nodes.size()) return -1;
    vec3 q = nodes[n].p;
    if (std::fabs(q.z) < kTwinEps) return -1;
    q.z = -q.z;
    return nearest_node(q, kTwinEps);
}

vec3 Model::centroid() const {
    vec3 c(0);
    for (const Node& n : nodes) c += n.p;
    return nodes.empty() ? c : c * (1.0f / (float)nodes.size());
}

std::string Model::slug() const {
    std::string s;
    for (char c : title) {
        if (isalnum((unsigned char)c)) s += (char)tolower((unsigned char)c);
        else if (!s.empty() && s.back() != '_') s += '_';
    }
    while (!s.empty() && s.back() == '_') s.pop_back();
    return s.empty() ? "model" : s;
}

void Model::remove_nodes(const std::vector<int>& ids) {
    std::vector<int> map(nodes.size(), 0);
    for (int i : ids)
        if (i >= 0 && i < (int)nodes.size()) map[i] = -1;
    std::vector<Node> kept;
    for (int i = 0; i < (int)nodes.size(); i++) {
        if (map[i] < 0) continue;
        map[i] = (int)kept.size();
        kept.push_back(nodes[i]);
    }
    nodes.swap(kept);
    remap_elements(*this, map);
}

int Model::merge_nodes(const std::vector<std::vector<int>>& groups, const std::vector<vec3>& at) {
    const int N = (int)nodes.size();
    std::vector<int> into(N, -1); // (a merged node: the node it goes into)
    int gone = 0;
    for (size_t g = 0; g < groups.size(); g++) {
        const std::vector<int>& grp = groups[g];
        if (grp.size() < 2 || grp[0] < 0 || grp[0] >= N) continue;
        const int keep = grp[0];
        if (g < at.size()) nodes[keep].p = at[g];
        for (size_t k = 1; k < grp.size(); k++) {
            const int o = grp[k];
            if (o < 0 || o >= N || o == keep || into[o] >= 0 || into[keep] >= 0) continue;
            into[o] = keep;
            nodes[keep].fixed |= nodes[o].fixed;
            nodes[keep].load_bearing |= nodes[o].load_bearing;
            if (nodes[o].load > nodes[keep].load) nodes[keep].load = nodes[o].load;
            gone++;
        }
    }
    if (!gone) return 0;
    std::vector<int> map(N, -1);
    std::vector<Node> kept;
    for (int i = 0; i < N; i++)
        if (into[i] < 0) map[i] = (int)kept.size(), kept.push_back(nodes[i]);
    for (int i = 0; i < N; i++)
        if (into[i] >= 0) map[i] = map[into[i]];
    nodes.swap(kept);
    remap_elements(*this, map);
    // what folded up, and the doubles
    std::set<std::pair<int, int>> seen;
    std::vector<Beam> bs;
    for (const Beam& b : beams)
        if (b.a != b.b && seen.insert({std::min(b.a, b.b), std::max(b.a, b.b)}).second) bs.push_back(b);
    beams.swap(bs);
    shocks.erase(std::remove_if(shocks.begin(), shocks.end(), [](const Shock& x) { return x.a == x.b; }), shocks.end());
    hydros.erase(std::remove_if(hydros.begin(), hydros.end(), [](const Hydro& x) { return x.a == x.b; }), hydros.end());
    joints.erase(std::remove_if(joints.begin(), joints.end(), [](const Joint& x) { return x.parent == x.child; }), joints.end());
    welds.erase(std::remove_if(welds.begin(), welds.end(), [](const Weld& x) { return x.anchor == x.node; }), welds.end());
    mounts.erase(std::remove_if(mounts.begin(), mounts.end(), [](const Mount& x) { return x.a == x.b; }), mounts.end());
    std::set<std::array<int, 3>> tseen;
    std::vector<Tri> ts;
    for (const Tri& t : tris) {
        if (t.a == t.b || t.b == t.c || t.a == t.c) continue;
        std::array<int, 3> k{t.a, t.b, t.c};
        std::sort(k.begin(), k.end());
        if (tseen.insert(k).second) ts.push_back(t);
    }
    tris.swap(ts);
    for (Flexbody& f : flexbodies) {
        std::sort(f.forset.begin(), f.forset.end());
        f.forset.erase(std::unique(f.forset.begin(), f.forset.end()), f.forset.end());
    }
    for (Submesh& sm : submeshes) {
        std::set<int> have;
        std::vector<std::pair<int, vec2>> tc;
        for (const auto& e : sm.texcoords)
            if (have.insert(e.first).second) tc.push_back(e);
        sm.texcoords.swap(tc);
    }
    return gone;
}

void Model::remove_beam(int i) { if (i >= 0 && i < (int)beams.size()) beams.erase(beams.begin() + i); }
void Model::remove_shock(int i) { if (i >= 0 && i < (int)shocks.size()) shocks.erase(shocks.begin() + i); }
void Model::remove_hydro(int i) { if (i >= 0 && i < (int)hydros.size()) hydros.erase(hydros.begin() + i); }
void Model::remove_tri(int i) { if (i >= 0 && i < (int)tris.size()) tris.erase(tris.begin() + i); }
void Model::remove_wheel(int i) {
    if (i < 0 || i >= (int)wheels.size()) return;
    // (the node map stays the identity; the wheel map drops wheel i)
    std::vector<int> map(nodes.size());
    for (int k = 0; k < (int)nodes.size(); k++) map[k] = k;
    const Wheel gone = wheels[i];
    wheels[i].n1 = -1; // (remap_elements drops it, and the graphics bound to its nodes)
    remap_elements(*this, map);
    (void)gone;
}

int Model::wheel_node_count(int w) const {
    if (w < 0 || w >= (int)wheels.size()) return 0;
    const Wheel& x = wheels[w];
    return (x.type == 1 || x.type == 4) ? 4 * x.rays : 2 * x.rays;
}

int Model::file_node(int ref) const {
    if (!is_wheel_ref(ref)) return ref;
    const int w = wheel_of_ref(ref);
    int n = (int)nodes.size();
    for (int i = 0; i < w && i < (int)wheels.size(); i++) n += wheel_node_count(i);
    return n + wheel_node_of_ref(ref);
}

ShellPreset Model::shell_preset(int i) const {
    if (i > 0 && i <= (int)shell_presets.size()) return shell_presets[i - 1];
    ShellPreset p;
    p.name = "Body";
    p.material = sheet_material, p.kg_m2 = sheet_kg_m2, p.thickness = sheet_thickness, p.max_level = sheet_max_level;
    return p;
}

void Model::set_shell_preset(int i, const ShellPreset& p) {
    if (i > 0 && i <= (int)shell_presets.size()) {
        shell_presets[i - 1] = p;
        return;
    }
    sheet_material = p.material, sheet_kg_m2 = p.kg_m2, sheet_thickness = p.thickness, sheet_max_level = p.max_level;
}

void Model::remove_shell_preset(int i) {
    if (i <= 0 || i > (int)shell_presets.size()) return;
    shell_presets.erase(shell_presets.begin() + (i - 1));
    for (Tri& t : tris) t.shell_preset = t.shell_preset == i ? 0 : t.shell_preset > i ? t.shell_preset - 1 : t.shell_preset;
}

int Model::ref_of_file_node(int n) const {
    if (n < 0) return -1;
    if (n < (int)nodes.size()) return n;
    n -= (int)nodes.size();
    for (int w = 0; w < (int)wheels.size(); w++) {
        const int c = wheel_node_count(w);
        if (n < c) return wheel_ref(w, n);
        n -= c;
    }
    return -1;
}

bool Model::ref_valid(int ref) const {
    if (!is_wheel_ref(ref)) return ref >= 0 && ref < (int)nodes.size();
    const int w = wheel_of_ref(ref);
    return w >= 0 && w < (int)wheels.size() && wheel_node_of_ref(ref) < wheel_node_count(w) && wheels[w].n1 >= 0 && wheels[w].n2 >= 0;
}

vec3 Model::ref_position(int ref) const {
    if (!is_wheel_ref(ref)) return ref >= 0 && ref < (int)nodes.size() ? nodes[ref].p : vec3(0);
    const int w = wheel_of_ref(ref), k = wheel_node_of_ref(ref);
    if (!ref_valid(ref)) return vec3(0);
    const Wheel& x = wheels[w];
    // the builder's ring: nodes in pairs (axle end 1, axle end 2) round the axle, the outer (tyre) ring first
    vec3 a = nodes[x.n1].p, b = nodes[x.n2].p;
    if (a.z > b.z) std::swap(a, b);
    const vec3 axis = normalize_or(b - a, vec3(0, 0, 1));
    const vec3 u = normalize_or(cross(axis, vec3(0, 1, 0)), vec3(1, 0, 0)), v = cross(axis, u);
    const int rays = std::max(1, x.rays);
    const bool rim = k >= 2 * rays;
    const int kk = k % (2 * rays);
    const float ang = 2.0f * kPi * (float)(kk / 2) / (float)rays;
    const float r = rim ? x.rim_radius : x.radius;
    return ((kk & 1) ? b : a) + (u * std::cos(ang) + v * std::sin(ang)) * r;
}
void Model::remove_joint(int i) { if (i >= 0 && i < (int)joints.size()) joints.erase(joints.begin() + i); }

int Model::find_beam(int a, int b) const {
    for (int i = 0; i < (int)beams.size(); i++) {
        const Beam& x = beams[i];
        if ((x.a == a && x.b == b) || (x.a == b && x.b == a)) return i;
    }
    return -1;
}

int Model::add_beam(int a, int b, int group) {
    if (a == b || a < 0 || b < 0 || a >= (int)nodes.size() || b >= (int)nodes.size() || has_beam(a, b)) return -1;
    beams.push_back({a, b, std::clamp(group, 0, std::max(0, (int)groups.size() - 1))});
    return (int)beams.size() - 1;
}

int Model::mirror(const std::vector<int>& ids) {
    std::vector<int> map(nodes.size(), -1);
    int made = 0;
    for (int i : ids) {
        if (i < 0 || i >= (int)nodes.size()) continue;
        if (std::fabs(nodes[i].p.z) < kTwinEps) {
            map[i] = i; // on the plane: shared
            continue;
        }
        int t = twin(i);
        if (t < 0) {
            Node n = nodes[i];
            n.p.z = -n.p.z;
            nodes.push_back(n);
            t = (int)nodes.size() - 1;
            made++;
        }
        map[i] = t;
    }
    auto m = [&](int n) { return n >= 0 && n < (int)map.size() ? map[n] : -1; };
    const size_t nb = beams.size(), ns = shocks.size(), nh = hydros.size(), nt = tris.size();
    for (size_t k = 0; k < nb; k++) {
        const Beam b = beams[k];
        const int a = m(b.a), c = m(b.b);
        if (a >= 0 && c >= 0 && !(a == b.a && c == b.b) && add_beam(a, c, b.group) >= 0) beams.back().layer = b.layer;
    }
    for (size_t k = 0; k < ns; k++) {
        Shock s = shocks[k];
        const int a = m(s.a), c = m(s.b);
        if (a >= 0 && c >= 0 && !(a == s.a && c == s.b)) {
            s.a = a, s.b = c;
            shocks.push_back(s);
        }
    }
    for (size_t k = 0; k < nh; k++) {
        Hydro h = hydros[k];
        const int a = m(h.a), c = m(h.b);
        if (a >= 0 && c >= 0 && !(a == h.a && c == h.b)) {
            h.a = a, h.b = c;
            h.factor = -h.factor; // (the other side steers the other way)
            hydros.push_back(h);
        }
    }
    for (size_t k = 0; k < nt; k++) {
        Tri t = tris[k];
        const int a = m(t.a), b = m(t.b), c = m(t.c);
        if (a >= 0 && b >= 0 && c >= 0 && !(a == t.a && b == t.b && c == t.c)) {
            t.a = a, t.b = c, t.c = b; // mirrored winding
            tris.push_back(t);
        }
    }
    const size_t nj = joints.size();
    for (size_t k = 0; k < nj; k++) {
        Joint j = joints[k];
        const int a = m(j.parent), c = m(j.child);
        if (a >= 0 && c >= 0 && !(a == j.parent && c == j.child)) {
            j.parent = a, j.child = c;
            joints.push_back(j);
        }
    }
    return made;
}

bool Model::node_visible(int n) const {
    if (n < 0 || n >= (int)nodes.size()) return false;
    const Node& x = nodes[n];
    if (!layer_visible(x.layer)) return false;
    return x.group < 0 || x.group >= (int)node_groups.size() || node_groups[x.group].visible;
}

void Model::remove_layer(int l) {
    if (l <= 0 || l >= (int)layers.size()) return; // (layer 0 stays)
    auto fix = [&](int& x) { x = x == l ? 0 : (x > l ? x - 1 : x); };
    for (Node& n : nodes) fix(n.layer);
    for (Beam& b : beams) fix(b.layer);
    for (Shock& s : shocks) fix(s.layer);
    for (Hydro& h : hydros) fix(h.layer);
    for (Tri& t : tris) fix(t.layer);
    for (Wheel& w : wheels) fix(w.layer);
    for (Joint& j : joints) fix(j.layer);
    for (Flexbody& f : flexbodies) fix(f.layer);
    for (Prop& p : props) fix(p.layer);
    layers.erase(layers.begin() + l);
}

void Model::remove_group(int g) {
    if (g < 0 || g >= (int)node_groups.size()) return;
    for (Node& n : nodes) n.group = n.group == g ? -1 : (n.group > g ? n.group - 1 : n.group);
    node_groups.erase(node_groups.begin() + g);
}

void Model::set_layer(const std::vector<int>& ids, int layer) {
    std::vector<char> in(nodes.size(), 0);
    for (int i : ids)
        if (i >= 0 && i < (int)nodes.size()) in[i] = 1, nodes[i].layer = layer;
    for (Beam& b : beams)
        if (in[b.a] && in[b.b]) b.layer = layer;
    for (Shock& s : shocks)
        if (in[s.a] && in[s.b]) s.layer = layer;
    for (Hydro& h : hydros)
        if (in[h.a] && in[h.b]) h.layer = layer;
    for (Tri& t : tris)
        if (in[t.a] && in[t.b] && in[t.c]) t.layer = layer;
    for (Wheel& w : wheels)
        if (in[w.n1] && in[w.n2]) w.layer = layer;
    for (Joint& j : joints)
        if (in[j.parent] && in[j.child]) j.layer = layer;
}

void Model::auto_cameras(int& center, int& back, int& left) const {
    center = cam_center, back = cam_back, left = cam_left;
    if (nodes.empty()) {
        center = back = left = 0;
        return;
    }
    const vec3 c = centroid();
    auto pick = [&](vec3 target) {
        int best = 0;
        float bd = 1e30f;
        for (int i = 0; i < (int)nodes.size(); i++) {
            const float d = length2(nodes[i].p - target);
            if (d < bd) bd = d, best = i;
        }
        return best;
    };
    if (center < 0 || center >= (int)nodes.size()) center = pick(c);
    if (back < 0 || back >= (int)nodes.size() || back == center) {
        // the node farthest behind the centre (RoR: +x is back)
        int best = -1;
        float bx = -1e30f;
        for (int i = 0; i < (int)nodes.size(); i++)
            if (i != center && nodes[i].p.x > bx) bx = nodes[i].p.x, best = i;
        back = best >= 0 ? best : center;
    }
    if (left < 0 || left >= (int)nodes.size() || left == center || left == back) {
        int best = -1;
        float bz = -1e30f;
        for (int i = 0; i < (int)nodes.size(); i++)
            if (i != center && i != back && nodes[i].p.z > bz) bz = nodes[i].p.z, best = i;
        left = best >= 0 ? best : center;
    }
}

// ---------------------------------------------------------------------------------------------------- templates
Model make_empty() { return Model(); }

Model make_box(int nx, int ny, int nz, vec3 size, float mass) {
    Model m;
    m.title = "Box";
    m.dry_mass = mass;
    m.engine = false;
    nx = std::max(2, nx), ny = std::max(2, ny), nz = std::max(2, nz);
    auto id = [&](int i, int j, int k) { return (i * ny + j) * nz + k; };
    for (int i = 0; i < nx; i++)
        for (int j = 0; j < ny; j++)
            for (int k = 0; k < nz; k++) {
                Node n;
                n.p = vec3((i / (float)(nx - 1) - 0.5f) * size.x, j / (float)(ny - 1) * size.y, (k / (float)(nz - 1) - 0.5f) * size.z);
                n.load_bearing = true;
                m.nodes.push_back(n);
            }
    // every node to every neighbour of its cell cube (edges, face and body diagonals)
    for (int i = 0; i < nx; i++)
        for (int j = 0; j < ny; j++)
            for (int k = 0; k < nz; k++)
                for (int di = -1; di <= 1; di++)
                    for (int dj = -1; dj <= 1; dj++)
                        for (int dk = -1; dk <= 1; dk++) {
                            const int a = i + di, b = j + dj, c = k + dk;
                            if ((di | dj | dk) == 0 || a < 0 || b < 0 || c < 0 || a >= nx || b >= ny || c >= nz) continue;
                            m.add_beam(id(i, j, k), id(a, b, c), 0);
                        }
    // the surface as collision triangles
    auto quad = [&](int a, int b, int c, int d) {
        m.tris.push_back({a, b, c, true});
        m.tris.push_back({a, c, d, true});
    };
    for (int i = 0; i + 1 < nx; i++)
        for (int j = 0; j + 1 < ny; j++) {
            quad(id(i, j, 0), id(i, j + 1, 0), id(i + 1, j + 1, 0), id(i + 1, j, 0));
            quad(id(i, j, nz - 1), id(i + 1, j, nz - 1), id(i + 1, j + 1, nz - 1), id(i, j + 1, nz - 1));
        }
    for (int i = 0; i + 1 < nx; i++)
        for (int k = 0; k + 1 < nz; k++) {
            quad(id(i, 0, k), id(i + 1, 0, k), id(i + 1, 0, k + 1), id(i, 0, k + 1));
            quad(id(i, ny - 1, k), id(i, ny - 1, k + 1), id(i + 1, ny - 1, k + 1), id(i + 1, ny - 1, k));
        }
    for (int j = 0; j + 1 < ny; j++)
        for (int k = 0; k + 1 < nz; k++) {
            quad(id(0, j, k), id(0, j, k + 1), id(0, j + 1, k + 1), id(0, j + 1, k));
            quad(id(nx - 1, j, k), id(nx - 1, j + 1, k), id(nx - 1, j + 1, k + 1), id(nx - 1, j, k + 1));
        }
    return m;
}

Model make_fem_plate(int nx, int nz, vec2 size, float thickness) {
    Model m;
    m.title = "FEM plate";
    m.dry_mass = 0.0f;
    m.engine = false;
    FemPreset fp;
    fp.thickness = thickness, fp.name = "Steel " + fmt("%.1f", thickness * 1000.0f) + " mm", fp.color = vec3(0.62f, 0.64f, 0.67f);
    m.fem_presets.push_back(fp);
    nx = std::max(2, nx), nz = std::max(2, nz);
    auto id = [&](int i, int k) { return i * nz + k; };
    for (int i = 0; i < nx; i++)
        for (int k = 0; k < nz; k++) {
            Node n;
            n.p = vec3((i / (float)(nx - 1) - 0.5f) * size.x, 0.5f, (k / (float)(nz - 1) - 0.5f) * size.y);
            m.nodes.push_back(n);
        }
    auto tri = [&](int a, int b, int c) {
        Tri t;
        t.a = a, t.b = b, t.c = c, t.fem = true, t.collision = false;
        m.tris.push_back(t);
    };
    for (int i = 0; i + 1 < nx; i++)
        for (int k = 0; k + 1 < nz; k++) {
            const int a = id(i, k), b = id(i + 1, k), c = id(i + 1, k + 1), d = id(i, k + 1);
            if ((i + k) % 2 == 0) tri(a, c, b), tri(a, d, c);
            else tri(a, d, b), tri(b, d, c);
        }
    return m;
}

Model make_fem_box(int n, vec3 size, float thickness) {
    Model m;
    m.title = "FEM box";
    m.dry_mass = 0.0f;
    m.engine = false;
    FemPreset fp;
    fp.thickness = thickness, fp.name = "Steel " + fmt("%.1f", thickness * 1000.0f) + " mm", fp.color = vec3(0.78f, 0.30f, 0.12f);
    m.fem_presets.push_back(fp);
    n = std::max(1, n);
    std::map<std::tuple<long, long, long>, int> at;
    auto node = [&](vec3 p) {
        const auto key = std::make_tuple(std::lround(p.x * 1000.0), std::lround(p.y * 1000.0), std::lround(p.z * 1000.0));
        if (auto it = at.find(key); it != at.end()) return it->second;
        Node x;
        x.p = p;
        m.nodes.push_back(x);
        return at[key] = (int)m.nodes.size() - 1;
    };
    auto grid = [&](vec3 o, vec3 du, vec3 dv) {
        for (int j = 0; j < n; j++)
            for (int i = 0; i < n; i++) {
                const int a = node(o + du * (float)i + dv * (float)j), b = node(o + du * (float)(i + 1) + dv * (float)j);
                const int c = node(o + du * (float)(i + 1) + dv * (float)(j + 1)), d = node(o + du * (float)i + dv * (float)(j + 1));
                Tri t1, t2;
                t1.fem = t2.fem = true, t1.collision = t2.collision = false;
                if ((i + j) & 1) t1.a = a, t1.b = b, t1.c = c, t2.a = a, t2.b = c, t2.c = d;
                else t1.a = a, t1.b = b, t1.c = d, t2.a = b, t2.b = c, t2.c = d;
                m.tris.push_back(t1), m.tris.push_back(t2);
            }
    };
    const vec3 o(-0.5f * size.x, 0.05f, -0.5f * size.z), X(size.x / n, 0, 0), Y(0, size.y / n, 0), Z(0, 0, size.z / n);
    grid(o, Z, X), grid(o + vec3(0, size.y, 0), X, Z), grid(o, X, Y), grid(o + vec3(0, 0, size.z), Y, X), grid(o, Y, Z), grid(o + vec3(size.x, 0, 0), Z, Y);
    return m;
}

Model make_plate(int nx, int nz, vec2 size, float mass, bool sheet) {
    Model m;
    m.title = sheet ? "Sheet" : "Plate";
    m.dry_mass = mass;
    m.engine = false;
    nx = std::max(2, nx), nz = std::max(2, nz);
    auto id = [&](int i, int k) { return i * nz + k; };
    for (int i = 0; i < nx; i++)
        for (int k = 0; k < nz; k++) {
            Node n;
            n.p = vec3((i / (float)(nx - 1) - 0.5f) * size.x, 0.5f, (k / (float)(nz - 1) - 0.5f) * size.y);
            n.load_bearing = !sheet;
            m.nodes.push_back(n);
        }
    for (int i = 0; i + 1 < nx; i++)
        for (int k = 0; k + 1 < nz; k++) {
            const int a = id(i, k), b = id(i + 1, k), c = id(i + 1, k + 1), d = id(i, k + 1);
            if ((i + k) % 2 == 0) {
                m.tris.push_back({a, c, b, true, sheet});
                m.tris.push_back({a, d, c, true, sheet});
            } else {
                m.tris.push_back({a, d, b, true, sheet});
                m.tris.push_back({b, d, c, true, sheet});
            }
        }
    if (!sheet) {
        for (int i = 0; i < nx; i++)
            for (int k = 0; k < nz; k++) {
                if (i + 1 < nx) m.add_beam(id(i, k), id(i + 1, k), 0);
                if (k + 1 < nz) m.add_beam(id(i, k), id(i, k + 1), 0);
                if (i + 1 < nx && k + 1 < nz) m.add_beam(id(i, k), id(i + 1, k + 1), 0), m.add_beam(id(i + 1, k), id(i, k + 1), 0);
            }
    } else {
        // a sheet hangs on a frame of its border (beams along the rim keep the shape when it is lifted)
        for (int i = 0; i + 1 < nx; i++) m.add_beam(id(i, 0), id(i + 1, 0), 0), m.add_beam(id(i, nz - 1), id(i + 1, nz - 1), 0);
        for (int k = 0; k + 1 < nz; k++) m.add_beam(id(0, k), id(0, k + 1), 0), m.add_beam(id(nx - 1, k), id(nx - 1, k + 1), 0);
        for (int i = 0; i < nx; i += nx - 1)
            for (int k = 0; k < nz; k += nz - 1) m.nodes[id(i, k)].load_bearing = true;
    }
    return m;
}

Model make_cylinder(int segments, int rings, float radius, float length, float mass) {
    Model m;
    m.title = "Cylinder";
    m.dry_mass = mass;
    m.engine = false;
    segments = std::max(3, segments), rings = std::max(2, rings);
    auto id = [&](int r, int s) { return r * segments + (s % segments); };
    for (int r = 0; r < rings; r++)
        for (int s = 0; s < segments; s++) {
            const float a = 2.0f * kPi * s / segments;
            Node n;
            n.p = vec3((r / (float)(rings - 1) - 0.5f) * length, radius + radius * std::sin(a), radius * std::cos(a));
            n.load_bearing = true;
            m.nodes.push_back(n);
        }
    // ring, along, and both diagonals of every wall quad; the ring's chords brace the section
    for (int r = 0; r < rings; r++)
        for (int s = 0; s < segments; s++) {
            m.add_beam(id(r, s), id(r, s + 1), 0);
            m.add_beam(id(r, s), id(r, s + 2), 0);
            if (r + 1 < rings) {
                m.add_beam(id(r, s), id(r + 1, s), 0);
                m.add_beam(id(r, s), id(r + 1, s + 1), 0);
                m.add_beam(id(r + 1, s), id(r, s + 1), 0);
                m.tris.push_back({id(r, s), id(r + 1, s), id(r + 1, s + 1), true});
                m.tris.push_back({id(r, s), id(r + 1, s + 1), id(r, s + 1), true});
            }
        }
    // end caps: a fan round a centre node
    for (int r = 0; r < rings; r += rings - 1) {
        Node c;
        c.p = vec3((r / (float)(rings - 1) - 0.5f) * length, radius, 0);
        c.load_bearing = true;
        m.nodes.push_back(c);
        const int cid = (int)m.nodes.size() - 1;
        for (int s = 0; s < segments; s++) {
            m.add_beam(cid, id(r, s), 0);
            if (r == 0) m.tris.push_back({cid, id(r, s + 1), id(r, s), true});
            else m.tris.push_back({cid, id(r, s), id(r, s + 1), true});
        }
    }
    return m;
}

Model make_cart(float length, float width, float height, float mass) {
    Model m = make_box(4, 2, 2, vec3(length, height, width), mass);
    m.title = "Cart";
    m.engine = true;
    // the body sits at axle height, the wheels on axle nodes beside it
    const float r = 0.35f;
    for (Node& n : m.nodes) n.p.y += r;
    for (int i = 0; i < (int)m.nodes.size(); i++) m.nodes[i].no_ground = false;
    const float ax[2] = {-length * 0.35f, length * 0.35f};
    for (int f = 0; f < 2; f++)
        for (int s = 0; s < 2; s++) {
            const float sz = s ? 1.0f : -1.0f;
            Node n1, n2;
            n1.p = vec3(ax[f], r, sz * (width * 0.5f + 0.05f));
            n2.p = vec3(ax[f], r, sz * (width * 0.5f + 0.30f));
            n1.load_bearing = n2.load_bearing = true;
            m.nodes.push_back(n1);
            const int a = (int)m.nodes.size() - 1;
            m.nodes.push_back(n2);
            const int b = (int)m.nodes.size() - 1;
            // the axle nodes braced to the body's nearby nodes
            std::vector<std::pair<float, int>> near;
            for (int i = 0; i < a; i++) near.push_back({length2(m.nodes[i].p - n1.p), i});
            std::sort(near.begin(), near.end());
            for (int k = 0; k < 4 && k < (int)near.size(); k++) m.add_beam(a, near[k].second, 0), m.add_beam(b, near[k].second, 0);
            m.add_beam(a, b, 0);
            Wheel w;
            w.n1 = a, w.n2 = b, w.arm = near[0].second, w.radius = r, w.width = 0.2f, w.rays = 12, w.mass = 30;
            w.propulsion = 1;
            m.wheels.push_back(w);
            if (ax[f] < 0) {
                // the front axle steers: the outer node swings about a kingpin (inner node - a tower above it) and a
                // rod to a rack node on the body turns it (the rod lengthens on one side, shortens on the other)
                Node tower;
                tower.p = n1.p + vec3(0, 0.30f, 0);
                tower.load_bearing = true;
                m.nodes.push_back(tower);
                const int t = (int)m.nodes.size() - 1;
                for (int k = 0; k < 3 && k < (int)near.size(); k++) m.add_beam(t, near[k].second, 0);
                m.add_beam(t, a, 0);
                m.add_beam(t, b, 0);
                Node rack;
                rack.p = vec3(ax[f] - 0.35f, r, sz * (width * 0.5f - 0.1f));
                rack.load_bearing = true;
                m.nodes.push_back(rack);
                const int rk = (int)m.nodes.size() - 1;
                for (int k = 0; k < 3 && k < (int)near.size(); k++) m.add_beam(rk, near[k].second, 0);
                m.add_beam(rk, t, 0);
                // (the rod to the rack must stay the only x constraint of the outer node: no body beams on it)
                m.beams.erase(std::remove_if(m.beams.begin(), m.beams.end(), [&](const Beam& x) { return (x.a == b || x.b == b) && x.a != a && x.b != a && x.a != t && x.b != t; }),
                              m.beams.end());
                Hydro h;
                h.a = b, h.b = rk, h.factor = 0.15f * sz;
                m.hydros.push_back(h);
            }
        }
    // an aerial on the roof: a beam held upright by an orientation joint (no bracing needed)
    {
        int top = 0;
        float best = -1e30f;
        for (int i = 0; i < (int)m.nodes.size(); i++) {
            const float s = m.nodes[i].p.y - std::fabs(m.nodes[i].p.z) - std::fabs(m.nodes[i].p.x + length * 0.3f);
            if (s > best) best = s, top = i;
        }
        Node tip;
        tip.p = m.nodes[top].p + vec3(0, 0.6f, 0);
        tip.load_bearing = true;
        tip.load = 2.0f;
        m.nodes.push_back(tip);
        const int t = (int)m.nodes.size() - 1;
        m.add_beam(top, t, 0);
        Joint j;
        j.parent = top, j.child = t, j.k = 20000.0f;
        m.joints.push_back(j);
    }
    return m;
}

// ---------------------------------------------------------------------------------------------------- tools
std::vector<std::array<int, 3>> triangulate(const Model& m, const std::vector<int>& ids, vec3 outside_from) {
    std::vector<std::array<int, 3>> out;
    if (ids.size() < 3) return out;
    // the flattest axis: the one of least extent (a side wall: z, a roof or floor: y, a bulkhead: x)
    vec3 mn(1e30f), mx(-1e30f);
    for (int i : ids) mn = vmin(mn, m.nodes[i].p), mx = vmax(mx, m.nodes[i].p);
    const vec3 ext = mx - mn;
    int drop = ext.x <= ext.y && ext.x <= ext.z ? 0 : (ext.y <= ext.z ? 1 : 2);
    const int ax = drop == 0 ? 1 : 0, ay = drop == 2 ? 1 : 2;
    struct P {
        vec2 p;
        int id;
    };
    std::vector<P> pts;
    for (int i : ids) pts.push_back({vec2((&m.nodes[i].p.x)[ax], (&m.nodes[i].p.x)[ay]), i});
    // Bowyer-Watson: a super triangle, points inserted one by one, the cavity of triangles whose circumcircle
    // holds the point re-triangulated round it
    const vec2 c((mn.x + mx.x) * 0.5f, 0);
    float span = std::max(ext.x, std::max(ext.y, ext.z)) * 4 + 1;
    vec2 centre(0, 0);
    for (const P& p : pts) centre = centre + p.p;
    centre = centre / (float)pts.size();
    const int s0 = (int)pts.size(), s1 = s0 + 1, s2 = s0 + 2;
    pts.push_back({centre + vec2(-span, -span), -1});
    pts.push_back({centre + vec2(span, -span), -1});
    pts.push_back({centre + vec2(0, span * 1.5f), -1});
    struct T {
        int a, b, c;
        bool bad = false;
    };
    std::vector<T> tris = {{s0, s1, s2}};
    auto in_circle = [&](const T& t, vec2 p) {
        const vec2 A = pts[t.a].p - p, B = pts[t.b].p - p, C = pts[t.c].p - p;
        const float det = (A.x * A.x + A.y * A.y) * (B.x * C.y - C.x * B.y) - (B.x * B.x + B.y * B.y) * (A.x * C.y - C.x * A.y) + (C.x * C.x + C.y * C.y) * (A.x * B.y - B.x * A.y);
        const float orient = (pts[t.b].p.x - pts[t.a].p.x) * (pts[t.c].p.y - pts[t.a].p.y) - (pts[t.b].p.y - pts[t.a].p.y) * (pts[t.c].p.x - pts[t.a].p.x);
        return orient > 0 ? det > 0 : det < 0;
    };
    for (int i = 0; i < s0; i++) {
        std::vector<std::pair<int, int>> edges;
        for (T& t : tris) {
            t.bad = in_circle(t, pts[i].p);
            if (t.bad) edges.push_back({t.a, t.b}), edges.push_back({t.b, t.c}), edges.push_back({t.c, t.a});
        }
        tris.erase(std::remove_if(tris.begin(), tris.end(), [](const T& t) { return t.bad; }), tris.end());
        for (size_t e = 0; e < edges.size(); e++) {
            bool shared = false;
            for (size_t f = 0; f < edges.size() && !shared; f++)
                if (e != f && ((edges[e].first == edges[f].first && edges[e].second == edges[f].second) || (edges[e].first == edges[f].second && edges[e].second == edges[f].first)))
                    shared = true;
            if (!shared) tris.push_back({edges[e].first, edges[e].second, i});
        }
    }
    (void)c;
    for (const T& t : tris) {
        if (t.a >= s0 || t.b >= s0 || t.c >= s0) continue;
        std::array<int, 3> tri{pts[t.a].id, pts[t.b].id, pts[t.c].id};
        const vec3 pa = m.nodes[tri[0]].p, pb = m.nodes[tri[1]].p, pc = m.nodes[tri[2]].p;
        const vec3 n = cross(pb - pa, pc - pa);
        if (length2(n) < 1e-10f) continue;
        if (dot(n, (pa + pb + pc) * (1.0f / 3) - outside_from) < 0) std::swap(tri[1], tri[2]); // outward
        out.push_back(tri);
    }
    return out;
}

bool load_obj(const std::string& path, std::vector<vec3>& verts, std::vector<uint32_t>& idx) {
    std::ifstream f(path);
    if (!f) return false;
    verts.clear();
    idx.clear();
    std::string line;
    while (std::getline(f, line)) {
        if (line.size() < 3) continue;
        if (line[0] == 'v' && line[1] == ' ') {
            vec3 p;
            if (sscanf(line.c_str() + 2, "%f %f %f", &p.x, &p.y, &p.z) == 3) verts.push_back(p);
        } else if (line[0] == 'f' && line[1] == ' ') {
            std::istringstream ss(line.substr(2));
            std::vector<int> poly;
            std::string tok;
            while (ss >> tok) {
                const int v = atoi(tok.c_str()); // (the vertex index of "v/vt/vn"; negative counts from the end)
                const int id = v > 0 ? v - 1 : (int)verts.size() + v;
                if (id >= 0 && id < (int)verts.size()) poly.push_back(id);
            }
            for (size_t k = 1; k + 1 < poly.size(); k++) idx.push_back((uint32_t)poly[0]), idx.push_back((uint32_t)poly[k]), idx.push_back((uint32_t)poly[k + 1]);
        }
    }
    return !verts.empty() && !idx.empty();
}

std::string forset_string(const std::vector<int>& ids) {
    std::vector<int> v = ids;
    std::sort(v.begin(), v.end());
    v.erase(std::unique(v.begin(), v.end()), v.end());
    std::string o;
    for (size_t i = 0; i < v.size();) {
        size_t j = i;
        while (j + 1 < v.size() && v[j + 1] == v[j] + 1) j++;
        if (!o.empty()) o += ", ";
        o += j > i ? fmt("%d", v[i]) + "-" + fmt("%d", v[j]) : fmt("%d", v[i]);
        i = j + 1;
    }
    return o;
}

std::vector<int> parse_forset(const std::string& s) {
    std::vector<int> out;
    size_t p = 0;
    while (p < s.size()) {
        while (p < s.size() && !isdigit((unsigned char)s[p])) p++;
        if (p >= s.size()) break;
        int a = 0;
        while (p < s.size() && isdigit((unsigned char)s[p])) a = a * 10 + (s[p++] - '0');
        while (p < s.size() && s[p] == ' ') p++;
        int b = a;
        if (p < s.size() && s[p] == '-') {
            p++;
            while (p < s.size() && s[p] == ' ') p++;
            b = 0;
            while (p < s.size() && isdigit((unsigned char)s[p])) b = b * 10 + (s[p++] - '0');
        }
        for (int i = a; i <= b && i - a < 100000; i++) out.push_back(i);
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

// ---------------------------------------------------------------------------------------------------- files
std::string write_truck(const Model& m, bool preview) {
    std::string o;
    o += m.title + "\n";
    o += ";written by the BeamLab model editor\n";
    // the editor's layers and groups: their state here, `;layer:<name>` / `;grp:<name>` before the elements
    if (m.layers.size() > 1 || !m.layers[0].visible || m.layers[0].locked) {
        o += ";editor-layers:";
        for (const Layer& l : m.layers) o += " " + l.name + "|" + (l.visible ? "1" : "0") + "|" + (l.locked ? "1" : "0");
        o += "\n";
    }
    if (!m.node_groups.empty()) {
        o += ";editor-groups:";
        for (const Group& g : m.node_groups) o += " " + g.name + "|" + (g.visible ? "1" : "0");
        o += "\n";
    }
    // the presets whole (names, colours, the unused ones): the game reads the sections' directives, the editor these;
    // before each run of elements the preset's index (`;editor-preset:`, `;editor-shell:`, `;editor-fem:`)
    auto clean = [](std::string s) {
        for (char& c : s)
            if (c == '|' || c == '\n' || c == '\r') c = '/';
        return s;
    };
    for (const BeamGroup& g : m.groups)
        o += ";editor-beam-preset: " + clean(g.name) + "|" + fmt("%.3f", g.color.x) + " " + fmt("%.3f", g.color.y) + " " + fmt("%.3f", g.color.z) + " " +
             fmt("%.3f", g.color.w) + "|" + num(g.spring) + " " + num(g.damp) + " " + num(g.deform) + " " + num(g.brk) + " " + num(g.plastic) + "|" + fmt("%d", g.type) +
             " " + (g.invisible ? "1" : "0") + " " + (g.hold_rotation ? "1" : "0") + " " + num(g.joint_k) + "|" + clean(g.frame_material) + " " + fmt("%d", g.frame_shape) +
             " " + num(g.frame_outer) + " " + num(g.frame_wall) + " " + fmt("%d", g.frame_end_a) + " " + fmt("%d", g.frame_end_b) + " " + num(g.frame_joint_k) + " " +
             num(g.frame_break) + " " + num(g.frame_joint_damp) + "\n";
    for (const ShellPreset& sp : m.shell_presets)
        o += ";editor-shell-preset: " + clean(sp.name) + "|" + clean(sp.material) + "|" + num(sp.kg_m2) + " " + num(sp.thickness) + " " + fmt("%.3f", sp.color.x) + " " +
             fmt("%.3f", sp.color.y) + " " + fmt("%.3f", sp.color.z) + " " + fmt("%d", sp.max_level) + "\n";
    for (const FemPreset& fp : m.fem_presets)
        o += ";editor-fem-preset: " + clean(fp.name) + "|" + clean(fp.material) + "|" + num(fp.thickness) + " " + fmt("%.3f", fp.color.x) + " " + fmt("%.3f", fp.color.y) +
             " " + fmt("%.3f", fp.color.z) + "\n";
    if (!m.ref_path.empty())
        o += ";editor-ref: " + m.ref_path + "|" + fmt("%.4f", m.ref_offset.x) + "|" + fmt("%.4f", m.ref_offset.y) + "|" + fmt("%.4f", m.ref_offset.z) + "|" + fmt("%.4f", m.ref_scale) + "|" +
             fmt("%.2f", m.ref_yaw) + "|" + fmt("%.2f", m.ref_alpha) + "|" + (m.ref_mockup ? "1" : "0") + "\n";
    auto layer_name = [&](int l) { return l >= 0 && l < (int)m.layers.size() ? m.layers[l].name : m.layers[0].name; };
    int cur_layer = 0, cur_group = -1;
    auto mark = [&](int layer, int group) {
        if (layer != cur_layer) o += ";layer:" + layer_name(layer) + "\n", cur_layer = layer;
        if (group != cur_group) o += ";grp:" + (group >= 0 && group < (int)m.node_groups.size() ? m.node_groups[group].name : std::string()) + "\n", cur_group = group;
    };
    auto section = [&]() { cur_layer = 0, cur_group = -1; };
    o += "globals\n;dry mass, cargo mass, cab material\n";
    std::string cab;
    if (m.shell_count() > 0) cab = "sheet/" + m.sheet_material + "/" + num(m.sheet_kg_m2) + "/" + num(m.sheet_thickness) + (m.sheet_max_level >= 0 ? "/" + fmt("%d", m.sheet_max_level) : "");
    else if (m.skin && !m.tris.empty()) cab = "color/" + fmt("%.3f", m.skin_color.x) + "," + fmt("%.3f", m.skin_color.y) + "," + fmt("%.3f", m.skin_color.z);
    else cab = m.cab_material;
    o += num(m.dry_mass) + ", " + num(m.cargo_mass) + ", " + cab + "\n";
    o += "minimass\n" + num(m.minimass) + "\n";
    if (m.adv_deform) o += "enable_advanced_deformation\n"; // (the deformation thresholds as written, below 400000 N too)
    if (!m.managed_materials.empty()) {
        if (m.mm_double_sided) o += "set_managedmaterials_options 1\n";
        o += "managedmaterials\n";
        auto dash = [](const std::string& t) { return t.empty() ? std::string("-") : t; };
        for (const ror::ManagedMaterialDef& mm : m.managed_materials) {
            const bool flex = mm.type.rfind("flexmesh", 0) == 0;
            o += mm.name + " " + mm.type + " " + dash(mm.diffuse) + " " + (flex ? dash(mm.damaged_diffuse) + " " : std::string()) + dash(mm.specular) + "\n";
        }
    }
    o += "nodes\n;id, x, y, z, options[, load]\n";
    section();
    float cur_minimass = -1.0f;
    for (int i = 0; i < (int)m.nodes.size(); i++) {
        const Node& n = m.nodes[i];
        mark(n.layer, n.group);
        if (n.minimass != cur_minimass) o += "set_default_minimass " + num(n.minimass) + "\n", cur_minimass = n.minimass;
        std::string opt;
        if (n.load_bearing || n.load >= 0) opt += 'l';
        if (n.no_ground) opt += 'c';
        if (opt.empty()) opt = "n";
        o += fmt("%d", i) + ", " + fmt("%.4f", n.p.x) + ", " + fmt("%.4f", n.p.y) + ", " + fmt("%.4f", n.p.z) + ", " + opt;
        if (n.load >= 0) o += ", " + num(n.load);
        o += "\n";
    }
    if (!m.beams.empty()) {
        o += "beams\n";
        section();
        int cur = -1;
        for (const Beam& b : m.beams) {
            mark(b.layer, -1);
            if (b.group != cur) {
                cur = b.group;
                const BeamGroup& g = m.groups[std::clamp(cur, 0, (int)m.groups.size() - 1)];
                o += ";editor-preset: " + fmt("%d", std::clamp(cur, 0, (int)m.groups.size() - 1)) + " " + clean(g.name) + "\nset_beam_defaults " + num(g.spring) + ", " + num(g.damp) + ", " + num(g.deform) + ", " + num(g.brk) + ", 0.05, tracks/beam, " +
                     num(g.plastic) + "\n";
                if (g.is_frame()) {
                    const std::string ja = phys::frame_joint_name((phys::FrameJoint)g.frame_end_a), jb = phys::frame_joint_name((phys::FrameJoint)g.frame_end_b);
                    o += "set_frame_section " + g.frame_material + ", " + phys::frame_shape_name((phys::FrameShape)std::clamp(g.frame_shape, 0, 3)) + ", " +
                         fmt("%.4f", g.frame_outer) + ", " + fmt("%.4f", g.frame_wall) + ", " + (ja == jb ? ja : ja + "/" + jb) + ", " + num(g.frame_joint_k) +
                         (g.frame_break > 0 || g.frame_joint_damp > 0 ? ", " + num(g.frame_break) + ", " + num(g.frame_joint_damp) : std::string()) + "\n";
                }
            }
            const BeamGroup& g = m.groups[std::clamp(cur, 0, (int)m.groups.size() - 1)];
            std::string opt;
            if (g.invisible) opt += 'i';
            if (g.type == BEAM_ROPE) opt += 'r';
            if (g.type == BEAM_SUPPORT) opt += 's';
            if (g.is_frame()) opt += 'F';
            std::string joints; // (a frame element's own joints)
            if (g.is_frame() && (b.end_a >= 0 || b.end_b >= 0))
                joints = std::string(", ") + phys::frame_joint_name((phys::FrameJoint)(b.end_a >= 0 ? b.end_a : g.frame_end_a)) + ", " +
                         phys::frame_joint_name((phys::FrameJoint)(b.end_b >= 0 ? b.end_b : g.frame_end_b));
            o += fmt("%d", b.a) + ", " + fmt("%d", b.b) + (opt.empty() ? "" : ", " + opt) + joints + "\n";
        }
    }
    if (!m.shocks.empty()) {
        o += "shocks\n;n1, n2, spring, damp, short bound, long bound, precompression, options\n";
        section();
        std::string cur;
        for (const Shock& s : m.shocks) {
            const std::string bd = defaults_line(s.bd);
            if (bd != cur) o += bd, cur = bd, section();
            o += (mark(s.layer, -1), fmt("%d", s.a)) + ", " + fmt("%d", s.b) + ", " + num(s.spring) + ", " + num(s.damp) + ", " + num(s.short_bound) + ", " + num(s.long_bound) + ", " +
                 num(s.precomp) + ", " + (s.invisible ? "i" : "n") + "\n";
        }
    }
    if (!m.hydros.empty()) {
        o += "hydros\n;n1, n2, factor, options\n";
        section();
        std::string cur;
        for (const Hydro& h : m.hydros) {
            const std::string bd = defaults_line(h.bd);
            if (bd != cur) o += bd, cur = bd, section();
            mark(h.layer, -1);
            std::string opt;
            if (h.invisible) opt += 'i';
            if (h.speed_dep) opt += 's';
            if (opt.empty()) opt = "n";
            o += fmt("%d", h.a) + ", " + fmt("%d", h.b) + ", " + num(h.factor) + ", " + opt + "\n";
        }
    }
    if (!m.wheels.empty()) {
        static const char* kWheelSections[] = {"wheels", "wheels2", "meshwheels", "meshwheels2", "flexbodywheels"};
        static const char* kWheelFields[] = {
            ";radius, width, rays, node1, node2, rigidity, braking, propulsion, arm, mass, spring, damping, face, band",
            ";rim radius, radius, width, rays, node1, node2, rigidity, braking, propulsion, arm, mass, rim spring, rim damping, spring, damping, face, band",
            ";radius, rim radius, width, rays, node1, node2, rigidity, braking, propulsion, arm, mass, spring, damping, side, rim mesh, tyre material",
            ";radius, rim radius, width, rays, node1, node2, rigidity, braking, propulsion, arm, mass, spring, damping, side, rim mesh, tyre material",
            ";radius, rim radius, width, rays, node1, node2, rigidity, braking, propulsion, arm, mass, spring, damping, rim spring, rim damping, side, rim mesh, tyre mesh"};
        int cur_type = -1;
        std::string cur_bd, cur_nd;
        section();
        for (const Wheel& w : m.wheels) {
            const int t = std::clamp(w.type, 0, 4);
            // the wheel's beam defaults and node friction, written when they change
            const std::string bd = defaults_line(w.bd);
            const std::string nd = w.friction != 1.0f || !cur_nd.empty() ? "set_node_defaults -1, " + num(w.friction) + ", 1, 1\n" : std::string();
            if (t != cur_type) {
                cur_type = t;
                o += std::string(kWheelSections[t]) + "\n" + bd + nd + kWheelFields[t] + "\n";
                cur_bd = bd, cur_nd = nd;
                section();
            } else {
                if (bd != cur_bd) o += bd, cur_bd = bd, section();
                if (nd != cur_nd) o += nd, cur_nd = nd, section();
            }
            mark(w.layer, -1);
            const std::string common = fmt("%d", w.rays) + ", " + fmt("%d", w.n1) + ", " + fmt("%d", w.n2) + ", " + (w.rigidity >= 0 ? fmt("%d", w.rigidity) : "9999") + ", " +
                                       fmt("%d", w.braking) + ", " + fmt("%d", w.propulsion) + ", " + fmt("%d", w.arm) + ", " + num(w.mass);
            const std::string side(1, w.side == 'r' ? 'r' : 'l');
            const std::string rim = w.rim_mesh.empty() ? "wheel.mesh" : w.rim_mesh;
            switch (t) {
            case 0: o += num(w.radius) + ", " + num(w.width) + ", " + common + ", " + num(w.spring) + ", " + num(w.damp) + ", " + w.face_material + " " + w.band_material + "\n"; break;
            case 1:
                o += num(w.rim_radius) + ", " + num(w.radius) + ", " + num(w.width) + ", " + common + ", " + num(w.rim_spring) + ", " + num(w.rim_damp) + ", " + num(w.spring) + ", " +
                     num(w.damp) + ", " + w.face_material + " " + w.band_material + "\n";
                break;
            case 2:
            case 3:
                o += num(w.radius) + ", " + num(w.rim_radius) + ", " + num(w.width) + ", " + common + ", " + num(w.spring) + ", " + num(w.damp) + ", " + side + ", " + rim + " " +
                     (w.tyre_material.empty() ? std::string("tracks/wheelband") : w.tyre_material) + "\n";
                break;
            default:
                o += num(w.radius) + ", " + num(w.rim_radius) + ", " + num(w.width) + ", " + common + ", " + num(w.spring) + ", " + num(w.damp) + ", " + num(w.rim_spring) + ", " +
                     num(w.rim_damp) + ", " + side + ", " + rim + " " + (w.tyre_material.empty() ? std::string("tyre.mesh") : w.tyre_material) + "\n";
                break;
            }
        }
        if (!cur_nd.empty()) o += "set_node_defaults -1, 1, 1, 1\n";
    }
    {
        // joints: the model's own, and a pair for every beam of a preset that holds its ends' rotation
        std::string held;
        for (const Beam& b : m.beams) {
            const BeamGroup& g = m.groups[std::clamp(b.group, 0, (int)m.groups.size() - 1)];
            if (!g.hold_rotation || g.is_frame()) continue; // (a frame element's ends are welded by the frame itself)
            held += fmt("%d", b.a) + ", " + fmt("%d", b.b) + ", " + num(g.joint_k) + ", 0\n";
            held += fmt("%d", b.b) + ", " + fmt("%d", b.a) + ", " + num(g.joint_k) + ", 0\n";
        }
        if (!m.joints.empty() || !held.empty()) {
            o += "joints\n;(BeamLab) parent, child, stiffness N/m, break force N (0 never)\n";
            section();
            for (const Joint& j : m.joints) o += (mark(j.layer, -1), fmt("%d", j.parent)) + ", " + fmt("%d", j.child) + ", " + num(j.k) + ", " + num(j.brk) + "\n";
            if (!held.empty()) o += (mark(0, -1), std::string(";held ends of the beams of rigid presets\n")) + held;
        }
    }
    {
        std::string fixes, contacters;
        for (int i = 0; i < (int)m.nodes.size(); i++) {
            if (m.nodes[i].fixed) fixes += fmt("%d", i) + "\n";
            if (m.nodes[i].contacter) contacters += fmt("%d", i) + "\n";
        }
        if (!fixes.empty()) o += "fixes\n" + fixes;
        if (!contacters.empty()) o += "contacters\n" + contacters;
    }
    if (m.engine) {
        o += "engine\n;min rpm, max rpm, torque, differential, reverse, neutral, gears...\n";
        o += num(m.min_rpm) + ", " + num(m.max_rpm) + ", " + num(m.torque) + ", " + num(m.diff) + ", " + num(m.reverse) + ", " + num(m.neutral);
        for (float g : m.gears) o += ", " + num(g);
        o += ", -1.0\n";
        o += "engoption\n" + num(m.inertia) + ", " + (m.engine_car ? "c" : "t") + ", " + num(m.clutch_force) + "\n";
    }
    o += "brakes\n" + num(m.brake_force) + "\n";
    int center, back, left;
    m.auto_cameras(center, back, left);
    o += "cameras\n" + fmt("%d", center) + ", " + fmt("%d", back) + ", " + fmt("%d", left) + "\n";
    if (m.cinecam && !preview && m.nodes.size() >= 8) {
        // a cockpit camera above the centre, held by the eight nearest nodes (only for driving: in the editor's preview
        // and physics test its eight beams would be the only beams of a shell-only shape, along its edges)
        const vec3 c = m.nodes[center].p;
        std::vector<std::pair<float, int>> near;
        for (int i = 0; i < (int)m.nodes.size(); i++) near.push_back({length2(m.nodes[i].p - c), i});
        std::sort(near.begin(), near.end());
        float top = c.y;
        for (const Node& n : m.nodes) top = std::max(top, n.p.y);
        o += "cinecam\n;editor-cinecam (the model's cockpit camera: the Vehicle tab)\n" + fmt("%.3f", c.x) + ", " + fmt("%.3f", std::min(top, c.y + 0.8f)) + ", " + fmt("%.3f", c.z);
        for (int k = 0; k < 8; k++) o += ", " + fmt("%d", near[k].second);
        o += "\n";
    }
    auto rot_off = [&](vec3 off, vec3 rot) {
        return fmt("%.4f", off.x) + ", " + fmt("%.4f", off.y) + ", " + fmt("%.4f", off.z) + ", " + num(rot.x) + ", " + num(rot.y) + ", " + num(rot.z);
    };
    // the meshes: a hidden one is marked (the editor's view), one switched off is a comment line (not in the vehicle,
    // read back by read_markers): ";editor-off-flexbody: hidden|layer|<the line>|<the forset>"
    std::string off;
    auto flex_line = [&](const Flexbody& f) {
        return fmt("%d", m.file_node(f.ref)) + ", " + fmt("%d", m.file_node(f.x)) + ", " + fmt("%d", m.file_node(f.y)) + ", " + rot_off(f.offset, f.rot) + ", " + f.mesh;
    };
    auto flex_forset = [&](const Flexbody& f) {
        std::vector<int> fs;
        for (int n : f.forset) fs.push_back(m.file_node(n));
        return fs.empty() ? fmt("%d", m.file_node(f.ref)) : forset_string(fs);
    };
    auto prop_line = [&](const Prop& p) {
        return fmt("%d", m.file_node(p.ref)) + ", " + fmt("%d", m.file_node(p.x)) + ", " + fmt("%d", m.file_node(p.y)) + ", " + rot_off(p.offset, p.rot) + ", " + p.mesh +
               (p.extra.empty() ? "" : " " + p.extra);
    };
    int n_flex = 0, n_prop = 0;
    for (const Flexbody& f : m.flexbodies) n_flex += f.enabled || preview;
    for (const Prop& p : m.props) n_prop += p.enabled || preview;
    if (n_flex) {
        o += "flexbodies\n;ref, x, y, offset x, y, z, rotation x, y, z (deg), mesh; forset: the nodes its vertices are bound to\n";
        section();
        for (const Flexbody& f : m.flexbodies) {
            if (!f.enabled && !preview) continue;
            mark(f.layer, -1);
            if (f.hidden) o += ";editor-hidden\n";
            o += flex_line(f) + "\n";
            o += "forset " + flex_forset(f) + "\n";
        }
    }
    if (n_prop) {
        o += "props\n;ref, x, y, offset x, y, z, rotation x, y, z (deg), mesh\n";
        section();
        for (const Prop& p : m.props) {
            if (!p.enabled && !preview) continue;
            mark(p.layer, -1);
            if (p.hidden) o += ";editor-hidden\n";
            o += prop_line(p) + "\n";
        }
    }
    if (!preview) {
        for (const Flexbody& f : m.flexbodies)
            if (!f.enabled) off += ";editor-off-flexbody: " + std::string(f.hidden ? "1" : "0") + "|" + layer_name(f.layer) + "|" + flex_line(f) + "|" + flex_forset(f) + "\n";
        for (const Prop& p : m.props)
            if (!p.enabled) off += ";editor-off-prop: " + std::string(p.hidden ? "1" : "0") + "|" + layer_name(p.layer) + "|" + prop_line(p) + "\n";
        if (!off.empty()) o += ";(meshes switched off in the model editor: not part of the vehicle)\n" + off;
    }
    if (!m.tris.empty() || !m.submeshes.empty()) {
        // cab options: an imported triangle keeps its letters, its collision letter follows the collision flag
        auto cab_opts = [&](const Tri& t) {
            std::string keep, other;
            for (char c : t.options) {
                if (std::string("cpuDFS").find(c) != std::string::npos) {
                    if (keep.empty()) keep = std::string(1, c);
                } else if (c != 'n') {
                    other += c;
                }
            }
            std::string opt = (t.collision || t.shell) ? (keep.empty() ? std::string("c") : keep) + other : other;
            return opt.empty() ? std::string("n") : opt;
        };
        cur_layer = 0;
        int tri_base = 0;
        std::vector<int> order; // the triangles in the order written (cab lines count for the layer markers)
        for (int s = 0; s < (int)m.submeshes.size(); s++) {
            const Submesh& sm = m.submeshes[s];
            o += "submesh\ntexcoords\n";
            for (const auto& [n, uv] : sm.texcoords) o += fmt("%d", n) + ", " + fmt("%.5f", uv.x) + ", " + fmt("%.5f", uv.y) + "\n";
            o += "cab\n";
            section();
            for (int i = 0; i < (int)m.tris.size(); i++) {
                const Tri& t = m.tris[i];
                if (t.submesh != s || t.fem) continue;
                mark(t.layer, -1);
                o += fmt("%d", t.a) + ", " + fmt("%d", t.b) + ", " + fmt("%d", t.c) + ", " + cab_opts(t) + "\n";
            }
            if (sm.backmesh) o += "backmesh\n";
        }
        (void)tri_base;
        (void)order;
        std::vector<int> own;
        for (int i = 0; i < (int)m.tris.size(); i++)
            if ((m.tris[i].submesh < 0 || m.tris[i].submesh >= (int)m.submeshes.size()) && !m.tris[i].fem) own.push_back(i);
        if (!own.empty()) {
        // texture coordinates: a planar map of the model (x along, y + z across), the sheet body's material plane
        o += ";editor-own-submesh\n";
        std::set<int> used;
        for (int i : own) used.insert(m.tris[i].a), used.insert(m.tris[i].b), used.insert(m.tris[i].c);
        vec3 mn(1e30f), mx(-1e30f);
        for (int i : used) mn = vmin(mn, m.nodes[i].p), mx = vmax(mx, m.nodes[i].p);
        const float sx = std::max(1e-3f, mx.x - mn.x), sy = std::max(1e-3f, (mx.y - mn.y) + (mx.z - mn.z));
        o += "submesh\ntexcoords\n";
        for (int i : used) {
            const vec3& p = m.nodes[i].p;
            o += fmt("%d", i) + ", " + fmt("%.4f", (p.x - mn.x) / sx) + ", " + fmt("%.4f", ((p.y - mn.y) + (p.z - mn.z)) / sy) + "\n";
        }
        o += "cab\n";
        section();
        for (int i : own) {
            const Tri& t = m.tris[i];
            o += (mark(t.layer, -1), fmt("%d", t.a)) + ", " + fmt("%d", t.b) + ", " + fmt("%d", t.c) + ", " + cab_opts(t) + "\n";
        }
        }
        bool other_mats = false;
        for (const Tri& t : m.tris) other_mats |= t.shell && t.shell_preset > 0 && t.shell_preset <= (int)m.shell_presets.size();
        if (m.shell_count() > 0 && (m.shell_count() < (int)m.tris.size() - m.fem_count() || other_mats)) {
            // the triangle elements, by material: the sheet material of the globals first, then each of the others
            // after its `set_shell_material name, material, kg/m2, drawn thickness, r, g, b, refinement depth`
            o += "shells\n;(BeamLab) the cab triangles that are triangle elements of the sheet body\n";
            for (int k = 0; k <= (int)m.shell_presets.size(); k++) {
                bool any = false;
                for (const Tri& t : m.tris) any |= t.shell && (k == 0 ? t.shell_preset <= 0 || t.shell_preset > (int)m.shell_presets.size() : t.shell_preset == k);
                if (!any) continue;
                if (k > 0) {
                    const ShellPreset& sp = m.shell_presets[k - 1];
                    std::string name = sp.name.empty() ? "material" + std::to_string(k) : sp.name;
                    for (char& c : name)
                        if (c == ' ' || c == ',' || c == '\t' || c == ';' || c == ':' || c == '|') c = '_';
                    name += "_" + std::to_string(k); // (unique: the parser takes a name again as the same material)
                    o += ";editor-shell: " + fmt("%d", k) + "\n";
                    o += "set_shell_material " + name + ", " + sp.material + ", " + num(sp.kg_m2) + ", " + num(sp.thickness) + ", " + fmt("%.3f", sp.color.x) + ", " +
                         fmt("%.3f", sp.color.y) + ", " + fmt("%.3f", sp.color.z) + ", " + fmt("%d", sp.max_level) + "\n";
                }
                for (const Tri& t : m.tris)
                    if (t.shell && !t.fem && (k == 0 ? t.shell_preset <= 0 || t.shell_preset > (int)m.shell_presets.size() : t.shell_preset == k))
                        o += fmt("%d", t.a) + ", " + fmt("%d", t.b) + ", " + fmt("%d", t.c) + "\n";
            }
        }
        if (m.fem_count() > 0) {
            // the FEM triangles, by shell: `set_fem_shell material, thickness, r, g, b` before each's triangles
            o += "fem_tris\n;(BeamLab) triangle elements of the FEM frame: n1, n2, n3, of the set_fem_shell before them\n";
            section();
            const int np = std::max(1, (int)m.fem_presets.size());
            for (int k = 0; k < np; k++) {
                bool any = false;
                for (const Tri& t : m.tris) any |= t.fem && (std::clamp(t.fem_preset, 0, np - 1) == k);
                if (!any) continue;
                const FemPreset fp = m.fem_preset(k);
                o += ";editor-fem: " + fmt("%d", k) + "\n";
                o += "set_fem_shell " + fp.material + ", " + num(fp.thickness) + ", " + fmt("%.3f", fp.color.x) + ", " + fmt("%.3f", fp.color.y) + ", " + fmt("%.3f", fp.color.z) +
                     "\n";
                for (const Tri& t : m.tris)
                    if (t.fem && std::clamp(t.fem_preset, 0, np - 1) == k) o += (mark(t.layer, -1), fmt("%d", t.a)) + ", " + fmt("%d", t.b) + ", " + fmt("%d", t.c) + "\n";
            }
        }
    }
    if (!m.welds.empty()) {
        o += "welds\n;(BeamLab) anchor, sheet node, radius m, break force N, stiffness N/m[, anchor2, t]\n";
        section();
        for (const Weld& w : m.welds)
            o += fmt("%d", w.anchor) + ", " + fmt("%d", w.node) + ", " + num(w.radius) + ", " + num(w.brk) + ", " + num(w.k) +
                 (w.anchor2 >= 0 ? ", " + fmt("%d", w.anchor2) + ", " + num(w.t) : std::string()) + "\n";
    }
    if (!m.mounts.empty()) {
        o += "mounts\n;(BeamLab) node a, node b, break force N, stiffness N/m, turning damping N m s/rad[, kind (p c h s r), its parameter]\n";
        section();
        for (const Mount& mt : m.mounts) {
            o += fmt("%d", mt.a) + ", " + fmt("%d", mt.b) + ", " + num(mt.brk) + ", " + num(mt.k) + ", " + num(mt.damp);
            if (mt.kind == 'h') o += ", h, " + fmt("%d", mt.b2);
            else if (mt.kind == 'c' || mt.kind == 'r') o += fmt(", %c, ", mt.kind) + num(mt.param);
            else if (mt.kind == 's') o += ", s";
            o += "\n";
        }
    }
    if (!m.volumes.empty()) {
        o += "collision_volumes\n;(BeamLab) volume name, break rms (m); its anchors (frame nodes); its hull's points (x, y, z)\n";
        section();
        for (const Volume& v : m.volumes) {
            o += "volume " + (v.name.empty() ? std::string("volume") : v.name) + ", " + num(v.break_rms) + "\nanchors ";
            for (size_t i = 0; i < v.anchors.size(); i++) o += (i ? ", " : "") + fmt("%d", v.anchors[i]);
            o += "\n";
            for (const vec3& p : v.verts) o += "vertex " + num(p.x) + ", " + num(p.y) + ", " + num(p.z) + "\n";
        }
    }
    if (!m.slidenodes.empty()) {
        o += "slidenodes\n;node, rail nodes..., S spring, B break, T tolerance, R attach rate, D attach distance\n";
        section();
        for (const SlideNode& sn : m.slidenodes) {
            o += fmt("%d", sn.node);
            for (int r : sn.rail) o += ", " + fmt("%d", r);
            auto opt = [&](char c, float v) {
                if (v >= 0) o += std::string(", ") + c + num(v);
            };
            opt('S', sn.spring), opt('B', sn.brk), opt('T', sn.tolerance), opt('R', sn.attach_rate), opt('D', sn.attach_dist);
            o += "\n";
        }
    }
    o += "end\n";
    return o;
}

bool import_document(const ror::Document& d, Model& m, std::vector<std::string>& warnings) {
    m = Model();
    m.title = d.title.empty() ? "Imported" : d.title;
    {
        // the folder the vehicle lives in: its meshes and materials are found there, and the model is saved there
        std::string dir = d.dir;
        while (!dir.empty() && (dir.back() == '/' || dir.back() == '\\')) dir.pop_back();
        const size_t k = dir.find_last_of("/\\");
        if (!dir.empty()) m.home = k == std::string::npos ? dir : dir.substr(k + 1);
    }
    m.dry_mass = d.dry_mass;
    m.cargo_mass = d.cargo_mass;
    m.minimass = d.minimass;
    for (const ror::BeamDef& b : d.beams) m.adv_deform |= b.bd.adv_deform;
    bool sheet = false;
    m.skin = false;
    if (d.cab_material.rfind("color/", 0) == 0) {
        m.skin = true;
        sscanf(d.cab_material.c_str() + 6, "%f,%f,%f", &m.skin_color.x, &m.skin_color.y, &m.skin_color.z);
    } else if (d.cab_material.rfind("sheet/", 0) == 0) {
        sheet = true;
        std::vector<std::string> f;
        for (size_t a = 0; a <= d.cab_material.size();) {
            size_t b = d.cab_material.find('/', a);
            if (b == std::string::npos) b = d.cab_material.size();
            f.push_back(d.cab_material.substr(a, b - a));
            a = b + 1;
        }
        if (f.size() > 1 && !f[1].empty()) m.sheet_material = f[1];
        if (f.size() > 2) m.sheet_kg_m2 = (float)atof(f[2].c_str());
        if (f.size() > 3) m.sheet_thickness = (float)atof(f[3].c_str());
        if (f.size() > 4 && !f[4].empty()) m.sheet_max_level = atoi(f[4].c_str());
    } else {
        m.cab_material = d.cab_material;
    }
    m.managed_materials = d.managed_materials;
    for (const auto& mm : d.managed_materials) m.mm_double_sided |= mm.double_sided;
    // nodes: the explicit ones; the nodes a wheel or a cinecam makes are not part of the model
    std::vector<int> map(d.nodes.size(), -1);
    for (int i = 0; i < (int)d.nodes.size(); i++) {
        const ror::NodeSlot& s = d.nodes[i];
        if (s.kind != ror::NodeSlot::EXPLICIT || s.ref < 0 || s.ref >= (int)d.nodes_explicit.size()) continue;
        const ror::NodeDef& nd = d.nodes_explicit[s.ref];
        Node n;
        n.p = nd.pos;
        n.load_bearing = has_opt(nd.options, 'l');
        n.no_ground = has_opt(nd.options, 'c');
        n.load = nd.load_weight;
        n.minimass = nd.minimass;
        n.contacter = false;
        map[i] = (int)m.nodes.size();
        m.nodes.push_back(n);
    }
    auto ok = [&](int n) { return n >= 0 && n < (int)map.size() && map[n] >= 0; };
    int dropped = 0, dropped_gfx = 0;
    for (int c : d.contacters)
        if (ok(c)) m.nodes[map[c]].contacter = true;
    for (int f : d.fixes)
        if (ok(f)) m.nodes[map[f]].fixed = true;
    // joints: a pair (a holds b, b holds a) on a beam makes it a beam of a rigid preset
    std::set<std::pair<int, int>> jp;
    for (const auto& j : d.joints)
        if (ok(j.parent) && ok(j.child)) jp.insert({map[j.parent], map[j.child]});
    std::set<std::pair<int, int>> consumed;
    // beams: a preset per distinct set_beam_defaults + options + held ends
    m.groups.clear();
    auto group_of = [&](const ror::BeamDefaults& bd, const std::string& opt, bool held, float jk, const ror::Document::FrameSectionDef* fs) {
        BeamGroup g;
        g.spring = bd.spring * bd.scale_spring;
        g.damp = bd.damp * bd.scale_damp;
        g.deform = bd.deform * bd.scale_deform;
        g.brk = bd.brk * bd.scale_break;
        g.plastic = bd.plastic;
        g.invisible = has_opt(opt, 'i');
        g.type = has_opt(opt, 'r') ? BEAM_ROPE : has_opt(opt, 's') ? BEAM_SUPPORT : BEAM_NORMAL;
        g.hold_rotation = held;
        if (held) g.joint_k = jk;
        if (fs) { // a frame element: its section; rigid ends are held ones (one pinned end reads as pinned)
            g.type = BEAM_FRAME;
            g.frame_material = phys::frame_material(fs->material).name;
            g.frame_shape = (int)phys::frame_shape(fs->shape);
            g.frame_outer = fs->outer, g.frame_wall = fs->wall;
            g.hold_rotation = false;
            g.frame_end_a = fs->end_a, g.frame_end_b = fs->end_b, g.frame_joint_k = fs->joint_k;
            g.frame_break = fs->brk, g.frame_joint_damp = fs->joint_damp;
        }
        for (int i = 0; i < (int)m.groups.size(); i++) {
            const BeamGroup& h = m.groups[i];
            if (h.spring == g.spring && h.damp == g.damp && h.deform == g.deform && h.brk == g.brk && h.plastic == g.plastic && h.invisible == g.invisible &&
                h.type == g.type && h.hold_rotation == g.hold_rotation && (!held || fs || h.joint_k == g.joint_k) &&
                (!fs || (h.frame_material == g.frame_material && h.frame_shape == g.frame_shape && h.frame_outer == g.frame_outer && h.frame_wall == g.frame_wall &&
                         h.frame_end_a == g.frame_end_a && h.frame_end_b == g.frame_end_b && h.frame_joint_k == g.frame_joint_k && h.frame_break == g.frame_break &&
                         h.frame_joint_damp == g.frame_joint_damp)))
                return i;
        }
        static const vec4 palette[] = {vec4(0.85f, 0.85f, 0.85f, 1), vec4(0.55f, 0.85f, 1, 1), vec4(1, 0.75f, 0.3f, 1), vec4(1, 0.4f, 0.4f, 1),
                                       vec4(0.6f, 1, 0.6f, 1),       vec4(0.9f, 0.6f, 1, 1),   vec4(0.9f, 0.8f, 0.4f, 1)};
        if (fs) {
            // named after its section: "Steel tube 40x2"
            g.name = g.frame_material + " " + phys::frame_shape_name((phys::FrameShape)g.frame_shape) + " " + fmt("%.0f", g.frame_outer * 1000.0f) +
                     (g.frame_shape == 0 || g.frame_shape == 1 ? "x" + fmt("%.3g", g.frame_wall * 1000.0f) : std::string()) +
                     (g.frame_end_a == 0 && g.frame_end_b == 0 ? std::string() : std::string(" ") + phys::frame_joint_name((phys::FrameJoint)g.frame_end_a) +
                                                                        (g.frame_end_a == g.frame_end_b ? "" : std::string("/") + phys::frame_joint_name((phys::FrameJoint)g.frame_end_b)));
            g.color = vec4(0.55f, 0.72f, 1.0f, 1);
            m.groups.push_back(g);
            return (int)m.groups.size() - 1;
        }
        g.name = (held ? "Rigid " : "Group ") + std::to_string(m.groups.size() + 1);
        g.color = palette[m.groups.size() % 7];
        if (g.invisible) g.color.w = 0.5f;
        m.groups.push_back(g);
        return (int)m.groups.size() - 1;
    };
    for (const ror::BeamDef& b : d.beams) {
        if (!ok(b.n1) || !ok(b.n2)) {
            dropped++;
            continue;
        }
        const int a = map[b.n1], c = map[b.n2];
        const bool held = jp.count({a, c}) && jp.count({c, a});
        float jk = 20000.0f;
        if (held) {
            for (const auto& j : d.joints)
                if (ok(j.parent) && ok(j.child) && map[j.parent] == a && map[j.child] == c) jk = j.k;
            consumed.insert({a, c}), consumed.insert({c, a});
        }
        const ror::Document::FrameSectionDef* fs = b.frame >= 0 && b.frame < (int)d.frame_sections.size() ? &d.frame_sections[b.frame] : nullptr;
        m.add_beam(a, c, group_of(b.bd, b.options, held, jk, fs));
        if (fs) { // (its own joints, where they differ from its section's)
            if (b.end_a >= 0 && b.end_a != fs->end_a) m.beams.back().end_a = b.end_a;
            if (b.end_b >= 0 && b.end_b != fs->end_b) m.beams.back().end_b = b.end_b;
        }
    }
    if (m.groups.empty()) m.groups = Model().groups;
    for (const ror::ShockDef& s : d.shocks) {
        if (!ok(s.n1) || !ok(s.n2)) {
            dropped++;
            continue;
        }
        Shock x;
        x.a = map[s.n1], x.b = map[s.n2];
        x.spring = s.spring, x.damp = s.damp, x.short_bound = s.shortbound, x.long_bound = s.longbound, x.precomp = s.precompression;
        if (s.type >= 2) x.spring = s.spring_in, x.damp = s.damp_in; // (shocks2 / shocks3: their inward values)
        const float len = length(m.nodes[x.b].p - m.nodes[x.a].p);
        if ((has_opt(s.options, 'm') || (s.type >= 2 && has_opt(s.options, 'M'))) && len > 1e-4f) {
            // bounds in metres (M: absolute lengths) as fractions of the length
            if (has_opt(s.options, 'm')) x.short_bound = s.shortbound / len, x.long_bound = s.longbound / len;
            else x.short_bound = std::min(1.0f, (len - s.shortbound) / len), x.long_bound = std::max(0.0f, (s.longbound - len) / len);
        }
        x.invisible = has_opt(s.options, 'i');
        x.bd = defaults_of(s.bd, false);
        m.shocks.push_back(x);
    }
    for (const ror::HydroDef& h : d.hydros) {
        if (!ok(h.n1) || !ok(h.n2)) {
            dropped++;
            continue;
        }
        Hydro x;
        x.a = map[h.n1], x.b = map[h.n2], x.factor = h.factor;
        x.bd = defaults_of(h.bd, true);
        x.invisible = has_opt(h.options, 'i');
        x.speed_dep = has_opt(h.options, 's');
        m.hydros.push_back(x);
    }
    std::vector<int> wmap(d.wheels.size(), -1);
    for (size_t wi = 0; wi < d.wheels.size(); wi++) {
        const ror::WheelDef& w = d.wheels[wi];
        if (!ok(w.n1) || !ok(w.n2)) {
            dropped++;
            continue;
        }
        wmap[wi] = (int)m.wheels.size();
        Wheel x;
        x.type = (int)w.type;
        x.radius = w.radius, x.rim_radius = w.rim_radius, x.width = w.width, x.rays = w.rays;
        x.n1 = map[w.n1], x.n2 = map[w.n2];
        x.rigidity = ok(w.rigidity) ? map[w.rigidity] : -1;
        x.braking = w.braking, x.propulsion = w.propulsion;
        x.arm = ok(w.arm) ? map[w.arm] : -1;
        x.mass = w.mass, x.spring = w.spring, x.damp = w.damp, x.rim_spring = w.rim_spring, x.rim_damp = w.rim_damp;
        x.side = w.side;
        if (!w.face_material.empty()) x.face_material = w.face_material;
        if (!w.band_material.empty()) x.band_material = w.band_material;
        x.rim_mesh = w.rim_mesh, x.tyre_material = w.tyre_material;
        // (the builder: the rim beams of meshwheels2 take the beam defaults' spring and damping, every wheel their
        // strength and deformation threshold; the tyre nodes the node defaults' friction)
        x.bd = defaults_of(w.bd, false);
        x.friction = w.nd.friction > 0 ? w.nd.friction : 1.0f;
        m.wheels.push_back(x);
    }
    for (int s = 0; s < (int)d.submeshes.size(); s++) {
        const ror::SubmeshDef& sd = d.submeshes[s];
        Submesh sm;
        sm.backmesh = sd.backmesh;
        for (const ror::TexcoordDef& t : sd.texcoords)
            if (ok(t.node)) sm.texcoords.push_back({map[t.node], vec2(t.u, t.v)});
        m.submeshes.push_back(sm);
        for (const ror::CabDef& c : sd.cabs) {
            if (!ok(c.n1) || !ok(c.n2) || !ok(c.n3)) {
                dropped++;
                continue;
            }
            Tri t;
            t.a = map[c.n1], t.b = map[c.n2], t.c = map[c.n3];
            t.collision = c.options.find_first_of("cpuDFS") != std::string::npos;
            t.shell = sheet && d.shells.empty();
            t.submesh = s;
            t.options = c.options;
            m.tris.push_back(t);
        }
    }
    for (const auto& s : d.shells) {
        if (!ok(s.n1) || !ok(s.n2) || !ok(s.n3)) continue;
        std::array<int, 3> k{map[s.n1], map[s.n2], map[s.n3]};
        std::sort(k.begin(), k.end());
        for (Tri& t : m.tris) {
            std::array<int, 3> q{t.a, t.b, t.c};
            std::sort(q.begin(), q.end());
            if (q == k) t.shell = true, t.shell_preset = s.mat;
        }
    }
    // the FEM triangles and their shells (a preset each; the names are the editor's)
    for (const auto& fs : d.fem_shells) {
        FemPreset p;
        p.material = fs.material, p.thickness = fs.thickness;
        if (fs.color.x >= 0) p.color = fs.color;
        p.name = fs.material + " " + fmt("%.1f", fs.thickness * 1000.0f) + " mm";
        m.fem_presets.push_back(p);
    }
    for (const auto& ft : d.fem_tris) {
        if (!ok(ft.n1) || !ok(ft.n2) || !ok(ft.n3)) {
            dropped++;
            continue;
        }
        Tri t;
        t.a = map[ft.n1], t.b = map[ft.n2], t.c = map[ft.n3];
        t.fem = true, t.collision = false, t.shell = false, t.fem_preset = ft.shell;
        m.tris.push_back(t);
    }
    // the sheet's welds to the frame, the parts' mounts, the slide nodes
    for (const auto& w : d.welds) {
        if (!ok(w.anchor) || !ok(w.node)) {
            dropped++;
            continue;
        }
        Weld x;
        x.anchor = map[w.anchor], x.node = map[w.node], x.radius = w.radius, x.brk = w.brk, x.k = w.k;
        x.anchor2 = ok(w.anchor2) ? map[w.anchor2] : -1, x.t = w.t;
        m.welds.push_back(x);
    }
    for (const auto& mt : d.mounts) {
        if (!ok(mt.a) || !ok(mt.b) || (mt.kind == 'h' && !ok(mt.b2))) {
            dropped++;
            continue;
        }
        Mount x;
        x.a = map[mt.a], x.b = map[mt.b], x.brk = mt.brk, x.k = mt.k, x.damp = mt.damp;
        x.kind = mt.kind, x.param = mt.param, x.b2 = mt.kind == 'h' ? map[mt.b2] : -1;
        m.mounts.push_back(x);
    }
    for (const auto& v : d.volumes) {
        Volume x;
        x.name = v.name, x.break_rms = v.break_rms, x.verts = v.verts;
        for (int a : v.anchors)
            if (ok(a)) x.anchors.push_back(map[a]);
        if (x.anchors.size() < 3) {
            dropped++;
            continue;
        }
        m.volumes.push_back(x);
    }
    for (const auto& sn : d.slidenodes) {
        SlideNode x;
        x.node = ok(sn.node) ? map[sn.node] : -1;
        for (int r : sn.rail)
            if (ok(r)) x.rail.push_back(map[r]);
        if (x.node < 0 || x.rail.size() < 2) {
            dropped++;
            continue;
        }
        x.spring = sn.spring, x.brk = sn.break_force, x.tolerance = sn.tolerance, x.attach_rate = sn.attach_rate, x.attach_dist = sn.attach_dist;
        m.slidenodes.push_back(x);
    }
    for (const auto& sm : d.shell_materials) {
        ShellPreset p;
        p.name = sm.name, p.material = sm.material, p.kg_m2 = sm.kg_m2, p.thickness = sm.thickness, p.color = sm.color, p.max_level = sm.max_level;
        m.shell_presets.push_back(p);
    }
    for (const auto& j : d.joints) {
        if (!ok(j.parent) || !ok(j.child)) {
            dropped++;
            continue;
        }
        if (consumed.count({map[j.parent], map[j.child]})) continue; // (a rigid beam's held end)
        Joint x;
        x.parent = map[j.parent], x.child = map[j.child], x.k = j.k, x.brk = j.brk;
        m.joints.push_back(x);
    }
    // graphics: meshes bound to nodes (explicit ones, or those of a wheel)
    auto gref = [&](int n) {
        if (ok(n)) return map[n];
        if (n >= 0 && n < (int)d.nodes.size() && d.nodes[n].kind == ror::NodeSlot::WHEEL) {
            const int w = d.nodes[n].ref;
            if (w >= 0 && w < (int)wmap.size() && wmap[w] >= 0) return wheel_ref(wmap[w], d.nodes[n].sub);
        }
        return -1;
    };
    for (const ror::FlexbodyDef& f : d.flexbodies) {
        if (gref(f.ref) < 0 || gref(f.x) < 0 || gref(f.y) < 0) {
            dropped_gfx++;
            continue;
        }
        Flexbody x;
        x.ref = gref(f.ref), x.x = gref(f.x), x.y = gref(f.y);
        x.offset = f.offset, x.rot = f.rot, x.mesh = f.mesh;
        for (int n : f.forset)
            if (gref(n) >= 0) x.forset.push_back(gref(n));
        std::sort(x.forset.begin(), x.forset.end());
        x.forset.erase(std::unique(x.forset.begin(), x.forset.end()), x.forset.end());
        m.flexbodies.push_back(x);
    }
    for (const ror::PropDef& p : d.props) {
        if (gref(p.ref) < 0 || gref(p.x) < 0 || gref(p.y) < 0) {
            dropped_gfx++;
            continue;
        }
        Prop x;
        x.ref = gref(p.ref), x.x = gref(p.x), x.y = gref(p.y);
        x.offset = p.offset, x.rot = p.rot, x.mesh = p.mesh;
        if (p.special == ror::PropDef::DASHBOARD || p.special == ror::PropDef::DASHBOARD_RH) {
            x.extra = p.wheel_mesh;
            if (p.has_wheel_offset) x.extra += ", " + fmt("%.4f", p.wheel_offset.x) + ", " + fmt("%.4f", p.wheel_offset.y) + ", " + fmt("%.4f", p.wheel_offset.z) + ", " + num(p.wheel_angle);
        } else if (p.special == ror::PropDef::BEACON) {
            x.extra = p.beacon_material + ", " + fmt("%.3f", p.beacon_color.x) + ", " + fmt("%.3f", p.beacon_color.y) + ", " + fmt("%.3f", p.beacon_color.z);
        }
        m.props.push_back(x);
    }
    m.engine = d.engine.present;
    if (d.engine.present) {
        m.min_rpm = d.engine.shift_down_rpm, m.max_rpm = d.engine.shift_up_rpm, m.torque = d.engine.torque;
        m.diff = d.engine.diff_ratio, m.reverse = d.engine.rev_ratio, m.neutral = d.engine.neutral_ratio;
        m.gears = d.engine.gears;
    }
    if (d.engoption.present) {
        m.engine_car = d.engoption.type != 't';
        if (d.engoption.inertia > 0) m.inertia = d.engoption.inertia;
        if (d.engoption.clutch_force > 0) m.clutch_force = d.engoption.clutch_force;
    }
    if (d.has_brakes) m.brake_force = d.brakes.force;
    if (!d.cameras.empty()) {
        const ror::CameraDef& c = d.cameras[0];
        m.cam_center = ok(c.center) ? map[c.center] : -1;
        m.cam_back = ok(c.back) ? map[c.back] : -1;
        m.cam_left = ok(c.left) ? map[c.left] : -1;
    }
    m.cinecam = !d.cinecams.empty(); // (an editor's file: read_markers says)
    if (dropped) warnings.push_back(fmt("%d", dropped) + " elements on wheel or cinecam nodes were dropped");
    if (dropped_gfx) warnings.push_back(fmt("%d", dropped_gfx) + " meshes bound to wheel or cinecam nodes were dropped");
    if (!d.commands.empty() || !d.ropes.empty() || !d.ties.empty() || !d.slidenodes.empty())
        warnings.push_back("commands, ropes, ties and slidenodes are not part of the model");
    return !m.nodes.empty();
}

void read_markers(const std::string& text, Model& m) {
    // the state lines, then a pass over the sections counting data lines: the n-th data line of `nodes` is node n,
    // of `beams` beam n, ... (the editor's files; a foreign file's `;grp:` lines of Editorizer name node groups too)
    std::vector<std::string> layer_names;
    auto layer_of = [&](const std::string& name) {
        for (int i = 0; i < (int)m.layers.size(); i++)
            if (m.layers[i].name == name) return i;
        m.layers.push_back({name, true, false});
        return (int)m.layers.size() - 1;
    };
    auto group_of = [&](const std::string& name) {
        if (name.empty()) return -1;
        for (int i = 0; i < (int)m.node_groups.size(); i++)
            if (m.node_groups[i].name == name) return i;
        m.node_groups.push_back({name, true});
        return (int)m.node_groups.size() - 1;
    };
    auto split = [](const std::string& s, char sep) {
        std::vector<std::string> out;
        size_t a = 0;
        while (a <= s.size()) {
            size_t b = s.find(sep, a);
            if (b == std::string::npos) b = s.size();
            out.push_back(s.substr(a, b - a));
            a = b + 1;
        }
        return out;
    };
    // an editor's file has the cockpit camera only when it was asked for (it used to be written for every model)
    if (text.find(";written by the BeamLab model editor") != std::string::npos) m.cinecam = text.find(";editor-cinecam") != std::string::npos;
    std::string section;
    int layer = 0, group = -1;
    int counts[9] = {}; // nodes, beams, shocks, hydros, wheels, cab, joints, flexbodies, props (across repeated sections)
    // the presets whole (write_truck), each beam's preset, the parser's shell materials and FEM shells in the order of
    // their directives -> the editor's preset
    std::vector<BeamGroup> beam_presets;
    std::vector<ShellPreset> shell_presets;
    std::vector<FemPreset> fem_presets;
    std::vector<int> beam_preset, shell_map{0}, fem_map;
    int cur_preset = -1, next_shell = -1, next_fem = -1;
    auto nums = [&](const std::string& s) {
        std::vector<float> v;
        for (const std::string& t : split(s, ' '))
            if (!t.empty()) v.push_back((float)atof(t.c_str()));
        return v;
    };
    int submesh_count = 0, own_submesh = -1;
    bool own_next = false, hidden_next = false;
    std::vector<Flexbody> off_flex;
    std::vector<Prop> off_props;
    // a mesh line (ref, x, y, offset, rotation, mesh[ extra]) in the file's numbering
    auto parse_mesh_line = [&](const std::string& ln, int& ref, int& x, int& y, vec3& off, vec3& rot, std::string& mesh, std::string& extra) {
        std::vector<std::string> f;
        size_t a = 0;
        for (int k = 0; k < 9; k++) {
            const size_t c = ln.find(',', a);
            if (c == std::string::npos) return false;
            f.push_back(ln.substr(a, c - a));
            a = c + 1;
        }
        std::string rest = ln.substr(a);
        const size_t r0 = rest.find_first_not_of(" \t");
        rest = r0 == std::string::npos ? std::string() : rest.substr(r0);
        const size_t sp = rest.find_first_of(" \t");
        mesh = rest.substr(0, sp);
        extra = sp == std::string::npos ? std::string() : rest.substr(rest.find_first_not_of(" \t", sp));
        ref = m.ref_of_file_node(atoi(f[0].c_str())), x = m.ref_of_file_node(atoi(f[1].c_str())), y = m.ref_of_file_node(atoi(f[2].c_str()));
        off = vec3((float)atof(f[3].c_str()), (float)atof(f[4].c_str()), (float)atof(f[5].c_str()));
        rot = vec3((float)atof(f[6].c_str()), (float)atof(f[7].c_str()), (float)atof(f[8].c_str()));
        return ref >= 0 && x >= 0 && y >= 0 && !mesh.empty();
    };
    size_t pos = 0;
    while (pos < text.size()) {
        size_t e = text.find('\n', pos);
        if (e == std::string::npos) e = text.size();
        std::string line = text.substr(pos, e - pos);
        pos = e + 1;
        while (!line.empty() && (line.back() == '\r' || line.back() == ' ' || line.back() == '\t')) line.pop_back();
        size_t s0 = line.find_first_not_of(" \t");
        if (s0 == std::string::npos) continue;
        line = line.substr(s0);
        if (line[0] == ';') {
            if (line.rfind(";editor-layers:", 0) == 0) {
                for (const std::string& item : split(line.substr(15), ' ')) {
                    if (item.empty()) continue;
                    const std::vector<std::string> f = split(item, '|');
                    const int l = layer_of(f[0]);
                    if (f.size() > 1) m.layers[l].visible = f[1] != "0";
                    if (f.size() > 2) m.layers[l].locked = f[2] != "0";
                }
            } else if (line.rfind(";editor-groups:", 0) == 0) {
                for (const std::string& item : split(line.substr(15), ' ')) {
                    if (item.empty()) continue;
                    const std::vector<std::string> f = split(item, '|');
                    const int g = group_of(f[0]);
                    if (g >= 0 && f.size() > 1) m.node_groups[g].visible = f[1] != "0";
                }
            } else if (line.rfind(";editor-ref:", 0) == 0) {
                std::string rest = line.substr(12);
                size_t n0 = rest.find_first_not_of(" \t");
                rest = n0 == std::string::npos ? std::string() : rest.substr(n0);
                const std::vector<std::string> f = split(rest, '|');
                if (!f.empty()) m.ref_path = f[0];
                if (f.size() > 3) m.ref_offset = vec3((float)atof(f[1].c_str()), (float)atof(f[2].c_str()), (float)atof(f[3].c_str()));
                if (f.size() > 4) m.ref_scale = (float)atof(f[4].c_str());
                if (f.size() > 5) m.ref_yaw = (float)atof(f[5].c_str());
                if (f.size() > 6) m.ref_alpha = (float)atof(f[6].c_str());
                if (f.size() > 7) m.ref_mockup = f[7] != "0";
            } else if (line.rfind(";editor-beam-preset:", 0) == 0) {
                const std::vector<std::string> f = split(line.substr(line.find(':') + 2), '|');
                if (f.size() >= 5) {
                    BeamGroup g;
                    g.name = f[0];
                    const std::vector<float> c = nums(f[1]), sp = nums(f[2]), ty = nums(f[3]);
                    if (c.size() >= 4) g.color = vec4(c[0], c[1], c[2], c[3]);
                    if (sp.size() >= 5) g.spring = sp[0], g.damp = sp[1], g.deform = sp[2], g.brk = sp[3], g.plastic = sp[4];
                    if (ty.size() >= 4) g.type = (int)ty[0], g.invisible = ty[1] != 0, g.hold_rotation = ty[2] != 0, g.joint_k = ty[3];
                    const std::vector<std::string> fr = split(f[4], ' ');
                    if (fr.size() >= 9) {
                        g.frame_material = fr[0], g.frame_shape = atoi(fr[1].c_str());
                        g.frame_outer = (float)atof(fr[2].c_str()), g.frame_wall = (float)atof(fr[3].c_str());
                        g.frame_end_a = atoi(fr[4].c_str()), g.frame_end_b = atoi(fr[5].c_str()), g.frame_joint_k = (float)atof(fr[6].c_str());
                        g.frame_break = (float)atof(fr[7].c_str()), g.frame_joint_damp = (float)atof(fr[8].c_str());
                    }
                    beam_presets.push_back(g);
                }
            } else if (line.rfind(";editor-shell-preset:", 0) == 0) {
                const std::vector<std::string> f = split(line.substr(line.find(':') + 2), '|');
                if (f.size() >= 3) {
                    ShellPreset p;
                    p.name = f[0], p.material = f[1];
                    const std::vector<float> v = nums(f[2]);
                    if (v.size() >= 6) p.kg_m2 = v[0], p.thickness = v[1], p.color = vec3(v[2], v[3], v[4]), p.max_level = (int)v[5];
                    shell_presets.push_back(p);
                }
            } else if (line.rfind(";editor-fem-preset:", 0) == 0) {
                const std::vector<std::string> f = split(line.substr(line.find(':') + 2), '|');
                if (f.size() >= 3) {
                    FemPreset p;
                    p.name = f[0], p.material = f[1];
                    const std::vector<float> v = nums(f[2]);
                    if (v.size() >= 4) p.thickness = v[0], p.color = vec3(v[1], v[2], v[3]);
                    fem_presets.push_back(p);
                }
            } else if (line.rfind(";editor-preset:", 0) == 0) {
                cur_preset = atoi(line.c_str() + 15);
            } else if (line.rfind(";editor-shell:", 0) == 0) {
                next_shell = atoi(line.c_str() + 14);
            } else if (line.rfind(";editor-fem:", 0) == 0) {
                next_fem = atoi(line.c_str() + 12);
            } else if (line.rfind(";editor-own-submesh", 0) == 0) {
                own_next = true;
            } else if (line == ";editor-hidden") {
                hidden_next = true;
            } else if (line.rfind(";editor-off-flexbody:", 0) == 0 || line.rfind(";editor-off-prop:", 0) == 0) {
                const bool flex = line[12] == 'f';
                const std::vector<std::string> f = split(line.substr(line.find(':') + 1), '|');
                if (f.size() >= 3) {
                    int ref, x, y;
                    vec3 off, rot;
                    std::string mesh, extra;
                    if (parse_mesh_line(f[2], ref, x, y, off, rot, mesh, extra)) {
                        const bool hid = f[0].find('1') != std::string::npos;
                        std::string ln = f[1];
                        while (!ln.empty() && ln.front() == ' ') ln.erase(ln.begin());
                        const int lay = layer_of(ln);
                        if (flex) {
                            Flexbody fb;
                            fb.ref = ref, fb.x = x, fb.y = y, fb.offset = off, fb.rot = rot, fb.mesh = mesh, fb.layer = lay, fb.hidden = hid, fb.enabled = false;
                            if (f.size() > 3)
                                for (int n : parse_forset(f[3]))
                                    if (m.ref_of_file_node(n) >= 0) fb.forset.push_back(m.ref_of_file_node(n));
                            std::sort(fb.forset.begin(), fb.forset.end());
                            off_flex.push_back(fb);
                        } else {
                            Prop pr;
                            pr.ref = ref, pr.x = x, pr.y = y, pr.offset = off, pr.rot = rot, pr.mesh = mesh, pr.extra = extra, pr.layer = lay, pr.hidden = hid, pr.enabled = false;
                            off_props.push_back(pr);
                        }
                    }
                }
            } else if (line.rfind(";layer:", 0) == 0) {
                layer = layer_of(line.substr(7));
            } else if (line.rfind(";grp:", 0) == 0) {
                std::string name = line.substr(5);
                size_t n0 = name.find_first_not_of(" \t");
                name = n0 == std::string::npos ? std::string() : name.substr(n0);
                group = group_of(name);
            }
            continue;
        }
        // a section keyword: a word alone on the line (the directives have arguments after a space)
        bool word = !line.empty();
        for (char c : line) word = word && (islower((unsigned char)c) || c == '_' || isdigit((unsigned char)c));
        if (word) {
            section = line;
            layer = 0, group = -1;
            if (line == "submesh") {
                if (own_next) own_submesh = submesh_count;
                submesh_count++;
                own_next = false;
            }
            continue;
        }
        if (line.rfind("set_shell_material", 0) == 0) shell_map.push_back(next_shell), next_shell = -1; // (the parser's material k + 1)
        if (line.rfind("set_fem_shell", 0) == 0) fem_map.push_back(next_fem), next_fem = -1;        // (its FEM shell k)
        if (line.rfind("set_", 0) == 0 || line.rfind("forset", 0) == 0 || line == "end") continue;
        if (section == "nodes" || section == "nodes2") {
            const int i = counts[0]++;
            if (i < (int)m.nodes.size()) m.nodes[i].layer = layer, m.nodes[i].group = group;
        } else if (section == "beams") {
            const int i = counts[1]++;
            if (i < (int)m.beams.size()) m.beams[i].layer = layer;
            if (i >= (int)beam_preset.size()) beam_preset.resize(i + 1, -1);
            beam_preset[i] = cur_preset;
        } else if (section == "shocks" || section == "shocks2" || section == "shocks3") {
            const int i = counts[2]++;
            if (i < (int)m.shocks.size()) m.shocks[i].layer = layer;
        } else if (section == "hydros") {
            const int i = counts[3]++;
            if (i < (int)m.hydros.size()) m.hydros[i].layer = layer;
        } else if (section == "wheels" || section == "wheels2" || section == "meshwheels" || section == "meshwheels2" || section == "flexbodywheels") {
            const int i = counts[4]++;
            if (i < (int)m.wheels.size()) m.wheels[i].layer = layer;
        } else if (section == "cab") {
            const int i = counts[5]++;
            if (i < (int)m.tris.size()) m.tris[i].layer = layer;
        } else if (section == "joints") {
            const int i = counts[6]++;
            if (i < (int)m.joints.size()) m.joints[i].layer = layer;
        } else if (section == "flexbodies") {
            const int i = counts[7]++;
            if (i < (int)m.flexbodies.size()) m.flexbodies[i].layer = layer, m.flexbodies[i].hidden = hidden_next;
            hidden_next = false;
        } else if (section == "props") {
            const int i = counts[8]++;
            if (i < (int)m.props.size()) m.props[i].layer = layer, m.props[i].hidden = hidden_next;
            hidden_next = false;
        }
    }
    for (const Flexbody& f : off_flex) m.flexbodies.push_back(f);
    for (const Prop& p : off_props) m.props.push_back(p);
    // the editor's presets as they were (the import made them from the directives: merged where the numbers are the
    // same, named by their numbers, the unused ones gone)
    if (!beam_presets.empty()) {
        std::vector<int> was(m.beams.size());
        for (size_t i = 0; i < m.beams.size(); i++) was[i] = m.beams[i].group;
        for (size_t i = 0; i < m.beams.size(); i++) {
            const int p = i < beam_preset.size() ? beam_preset[i] : -1;
            if (p >= 0 && p < (int)beam_presets.size()) m.beams[i].group = p;
            else { // (no marker: the imported preset, added after the editor's)
                beam_presets.push_back(m.groups[std::clamp(was[i], 0, (int)m.groups.size() - 1)]);
                const int g = (int)beam_presets.size() - 1;
                for (size_t j = i; j < m.beams.size(); j++)
                    if (was[j] == was[i] && (j >= beam_preset.size() || beam_preset[j] < 0)) m.beams[j].group = g, was[j] = -2;
            }
        }
        m.groups = beam_presets;
    }
    if (!shell_presets.empty()) {
        for (Tri& t : m.tris)
            if (t.shell && t.shell_preset > 0) {
                const int k = t.shell_preset < (int)shell_map.size() ? shell_map[t.shell_preset] : -1;
                t.shell_preset = k >= 1 && k <= (int)shell_presets.size() ? k : 0;
            }
        m.shell_presets = shell_presets;
    }
    if (!fem_presets.empty()) {
        for (Tri& t : m.tris)
            if (t.fem) {
                const int k = t.fem_preset >= 0 && t.fem_preset < (int)fem_map.size() ? fem_map[t.fem_preset] : -1;
                t.fem_preset = k >= 0 && k < (int)fem_presets.size() ? k : 0;
            }
        m.fem_presets = fem_presets;
    }
    // the editor's own submesh (planar texture coordinates made at every save) is the editor's again
    if (own_submesh >= 0 && own_submesh < (int)m.submeshes.size()) {
        for (Tri& t : m.tris) {
            if (t.submesh == own_submesh) t.submesh = -1, t.options.clear();
            else if (t.submesh > own_submesh) t.submesh--;
        }
        m.submeshes.erase(m.submeshes.begin() + own_submesh);
    }
    (void)layer_names;
}

std::vector<std::string> validate(const Model& m) {
    std::vector<std::string> out;
    if (m.nodes.empty()) out.push_back("no nodes");
    if (m.title.empty()) out.push_back("no title");
    int loose = 0, dup = 0, zero = 0, flat = 0;
    for (int i = 0; i < (int)m.nodes.size(); i++)
        if (m.node_uses(i) == 0) loose++;
    for (size_t i = 0; i < m.beams.size(); i++) {
        const Beam& b = m.beams[i];
        if (length2(m.nodes[b.a].p - m.nodes[b.b].p) < 1e-6f) zero++;
        for (size_t j = i + 1; j < m.beams.size(); j++)
            if ((m.beams[j].a == b.a && m.beams[j].b == b.b) || (m.beams[j].a == b.b && m.beams[j].b == b.a)) dup++;
    }
    for (const Tri& t : m.tris) {
        const vec3 n = cross(m.nodes[t.b].p - m.nodes[t.a].p, m.nodes[t.c].p - m.nodes[t.a].p);
        if (length2(n) < 1e-8f) flat++;
    }
    if (loose) out.push_back(fmt("%d", loose) + " nodes without beams or triangles (they fall off)");
    if (zero) out.push_back(fmt("%d", zero) + " beams of zero length");
    if (dup) out.push_back(fmt("%d", dup) + " duplicate beams");
    if (flat) out.push_back(fmt("%d", flat) + " degenerate (flat) triangles");
    for (const Wheel& w : m.wheels) {
        if (w.n1 == w.n2) out.push_back("a wheel's axle nodes are the same node");
        if (w.n1 >= 0 && w.n2 >= 0 && length2(m.nodes[w.n1].p - m.nodes[w.n2].p) < 1e-4f) out.push_back("a wheel's axle is shorter than 1 cm");
        if (w.arm < 0) out.push_back("a wheel has no arm node");
    }
    for (const Joint& j : m.joints)
        if (j.parent == j.child) out.push_back("a joint's parent and child are the same node");
    for (const Flexbody& f : m.flexbodies) {
        if (!m.ref_valid(f.ref) || !m.ref_valid(f.x) || !m.ref_valid(f.y)) out.push_back("flexbody " + f.mesh + ": a node it is bound to is gone");
        if (f.ref == f.x || f.ref == f.y || f.x == f.y) out.push_back("flexbody " + f.mesh + ": its ref, x and y nodes must differ");
        if (f.forset.empty()) out.push_back("flexbody " + f.mesh + ": no forset nodes");
    }
    for (const Prop& p : m.props)
        if (p.ref == p.x || p.ref == p.y || p.x == p.y) out.push_back("prop " + p.mesh + ": its ref, x and y nodes must differ");
    if (m.fem_count() > 0 && m.fem_presets.empty()) out.push_back("FEM triangles without a FEM shell preset (the default, 1 mm steel, is written)");
    for (const FemPreset& fp : m.fem_presets)
        if (!(fp.thickness > 0)) out.push_back("FEM shell " + fp.name + ": no thickness");
    if (m.engine && m.wheels.empty()) out.push_back("an engine but no wheels");
    if (m.engine && m.gears.empty()) out.push_back("an engine without gears");
    return out;
}

} // namespace bl::edit
