#include "vehicle/vehicle.h"
#include "core/profiler.h"
#include "core/util.h"
#include "vehicle/builder.h"
#include "vehicle/drivetrain.h"
#include "vehicle/visual.h"

#include <algorithm>
#include <map>
#include <set>
#include <cmath>
#include <cstdio>

namespace bl {

using namespace phys;

// ------------------------------------------------------------------ registry
static std::vector<VehicleEntry> scan_vehicles() {
        std::vector<VehicleEntry> r;
        std::string root = asset_path("vehicles");
        for (const auto& folder : list_dir(root, false, true)) {
            std::string dir = path_join(root, folder);
            std::string group = folder;
            std::string src;
            if (read_text_file(path_join(dir, "SOURCE.txt"), src)) {
                size_t e = src.find('\n');
                group = trim(src.substr(0, e));
            }
            for (const auto& f : list_dir(dir, true, false)) {
                std::string ext = path_ext_lower(f);
                if (ext != ".truck" && ext != ".car" && ext != ".trailer" && ext != ".load") continue;
                VehicleEntry e;
                e.folder = folder;
                e.file = path_join(dir, f);
                e.id = folder + "/" + path_stem(f);
                e.type = ext.substr(1);
                e.group = group;
                e.title = trim(ror::read_truck_title(e.file));
                if (e.title.empty()) e.title = path_stem(f);
                std::string text;
                e.drivable = e.type != "trailer" && e.type != "load";
                if (e.drivable && read_text_file(e.file, text)) {
                    bool engine = false;
                    for (size_t p = 0; !engine && (p = text.find("engine", p)) != std::string::npos; p += 6)
                        engine = (p == 0 || text[p - 1] == '\n') && (p + 6 >= text.size() || isspace((unsigned char)text[p + 6]));
                    e.drivable = engine;
                }
                r.push_back(e);
            }
        }
        std::stable_sort(r.begin(), r.end(), [](const VehicleEntry& a, const VehicleEntry& b) {
            if (a.group != b.group) return a.group < b.group;
            return a.title < b.title;
        });
        return r;
}

static std::vector<VehicleEntry>& registry_storage() {
    static std::vector<VehicleEntry> reg = scan_vehicles();
    return reg;
}

const std::vector<VehicleEntry>& vehicle_registry() { return registry_storage(); }

void refresh_vehicle_registry() { registry_storage() = scan_vehicles(); }

const VehicleEntry* find_vehicle(const std::string& id) {
    for (const auto& e : vehicle_registry())
        if (e.id == id) return &e;
    // accept folder name alone
    for (const auto& e : vehicle_registry())
        if (e.folder == id) return &e;
    return nullptr;
}

// ------------------------------------------------------------------ vehicle
Vehicle::~Vehicle() = default;

static float ground_height_at(const World& w, float x, float z, float from_y) {
    float t = w.statics.raycast(vec3(x, from_y, z), vec3(0, -1, 0), 400.0f);
    if (t >= 0) return from_y - t;
    return w.statics.has_terrain ? w.statics.terrain.height(x, z) : 0.0f;
}

std::unique_ptr<Vehicle> Vehicle::create(const VehicleEntry& e, World& world, vec3 pos, float yaw_deg, std::string* err) {
    PROFILE_ZONE("Vehicle create");
    auto v = std::unique_ptr<Vehicle>(new Vehicle());
    v->id = e.id;
    if (!(e.text.empty() ? ror::parse_truck_file(e.file, v->m_def) : ror::parse_truck_text(e.text, e.file, v->m_def))) {
        if (err) *err = "parse failed: " + e.file;
        return nullptr;
    }
    const ror::Document& d = v->m_def;
    v->name = trim(d.title);
    for (auto& w : d.warnings) v->m_warnings.push_back(w);
    auto body = std::make_unique<SoftBody>();
    body->name = v->name;
    v->m_drive = std::make_unique<Drivetrain>();
    if (!VehicleBuilder::build(d, *body, *v->m_drive, v->m_warnings)) {
        if (err) *err = "build failed";
        return nullptr;
    }
    v->m_drive->init(d, *body);

    // ---- local orientation from the first camera line (RoR convention)
    const int N = body->node_count();
    vec3 fwd(-1, 0, 0), left(0, 0, 1);
    if (!d.cameras.empty()) {
        const auto& c = d.cameras[0];
        if (c.center < N && c.back < N && c.left < N) {
            v->m_cam_center = c.center;
            v->m_cam_back = c.back;
            v->m_cam_left = c.left;
            vec3 pc = body->nodes[c.center].p, pb = body->nodes[c.back].p, pl = body->nodes[c.left].p;
            vec3 f = pc - pb, l = pl - pc;
            if (length(f) > 1e-3f && length(l) > 1e-3f) {
                fwd = normalize(f);
                left = normalize(l - fwd * dot(l, fwd));
            }
        }
    }
    vec3 up = cross(fwd, left);
    if (up.y < 0) {
        left = -left; // camera "left" node on the wrong side (RoR roll correction)
        v->m_warnings.push_back("camera left node on the right side - corrected");
        up = cross(fwd, left);
    }
    v->m_drive->cam_center = v->m_cam_center;
    v->m_drive->cam_back = v->m_cam_back;
    v->m_local_fwd = fwd;
    v->m_local_left = left;
    for (int i = 0; i < N; i++)
        if (d.nodes[i].kind == ror::NodeSlot::CINECAM) {
            v->m_cinecam = i;
            break;
        }

    // ---- visuals are built in definition space
    v->m_visual = std::make_unique<VehicleVisual>();
    v->m_visual->split_parts = e.split_parts;
    v->m_visual->build(d, *body, v->m_warnings);

    // pristine template for resets
    v->m_spawn_nodes = body->nodes;
    v->m_spawn_beams = body->beams;
    v->m_spawn_shocks = body->shocks;
    v->m_spawn_frames = body->frames;
    v->m_spawn_joints = body->joints;
    v->m_spawn_fem = body->fem;
    v->m_spawn_info = body->info; // (a torn frame's debris switches nodes off: a reset turns them on)
    v->m_mass = body->total_mass();

    Vehicle* raw = v.get();
    body->pre_substep = [raw](SoftBody& b, float dt) { raw->substep(b, dt); };
    body->can_sleep = true;
    v->body = world.add_body(std::move(body));
    v->m_world = &world;
    v->reset(pos, yaw_deg);
    log_info("vehicle '%s': %d nodes, %d beams, %d shocks, %d wheels, %d coll tris, %.0f kg, %d warnings", v->name.c_str(), N,
             (int)v->body->beams.size(), (int)v->body->shocks.size(), (int)v->body->wheels.size(), (int)v->body->tris.size(), v->m_mass,
             (int)v->m_warnings.size());
    return v;
}

void Vehicle::reset(vec3 pos, float yaw_deg) {
    SoftBody& b = *body;
    // restore pristine state
    b.nodes = m_spawn_nodes;
    b.beams = m_spawn_beams;
    b.shocks = m_spawn_shocks;
    b.frames = m_spawn_frames;
    b.joints = m_spawn_joints;
    b.welds = m_spawn_welds;
    b.fem = m_spawn_fem;
    if (!m_sheet && !m_spawn_info.empty()) {
        b.info = m_spawn_info;
        b.force.assign(b.nodes.size(), vec3(0));
        b.node_wheel.clear();
        b.contacter_count = -1;
        b.topo_version++;
    }
    if (m_sheet) {
        // the sheet as it was: its refinements and cracks added nodes and triangles
        b.info = m_spawn_info;
        b.shells = m_spawn_shells;
        b.tris = m_spawn_tris;
        b.node_base_mass = m_spawn_base_mass;
        b.force.assign(b.nodes.size(), vec3(0));
        b.ext_force.clear();
        b.wind_area.clear();
        b.node_shells.assign(b.nodes.size(), {});
        for (uint32_t si = 0; si < b.shells.size(); si++)
            for (int c = 0; c < 3; c++) b.node_shells[b.shells[si].n[c]].push_back(si);
        b.shell_events.clear();
        b.shell_impacts.clear();
        b.shell_hit.speed = 0;
        b.shell_level = 0;
        b.shell_stats = {};
        b.shk.dirty_shells.clear();
        b.shk.dirty_nodes.clear();
        b.shk.version = ~0u;
        b.shell_acc_stale = true;
        b.topo_version++;
        b.contacter_count = -1;
    }
    for (auto& w : b.wheels) {
        w.speed = w.avg_speed = w.last_torque = w.last_retorque = w.torque = w.brake = 0;
        w.detached = false;
    }
    b.stats = BodyStats();
    assist_speed = -1;
    peak_g = 0;
    last_com_vel = vec3(0);
    for (auto& s : b.slides) {
        s.seg = -1;
        s.broken = false;
    }
    // rotation: local (fwd, up, left) -> world (wf, Y, wl)
    float yaw = yaw_deg * kDeg2Rad;
    vec3 wf(std::sin(yaw), 0, std::cos(yaw)), wu(0, 1, 0), wl = cross(wu, wf);
    vec3 lf = m_local_fwd, ll = m_local_left, lu = cross(lf, ll);
    mat3 W(wf, wu, wl), L(lf, lu, ll);
    mat3 R = W * transpose(L);
    AABB la;
    for (auto& n : b.nodes) la.add(n.p);
    vec3 pivot = la.center();
    pivot.y = la.mn.y;
    for (auto& n : b.nodes) {
        n.p = R * (n.p - pivot) + pos;
        n.v = vec3(0);
    }
    b.fem.set_orientation(from_mat3(R)); // (the frame nodes turn with the vehicle)
    // lift so the lowest ground-contact node sits on the ground (terrain or static geometry)
    float lift = -1e9f;
    float top = -1e9f;
    for (auto& n : b.nodes) top = std::max(top, n.p.y);
    for (size_t i = 0; i < b.nodes.size(); i++) {
        if (!(b.info[i].flags & NF_GROUND)) continue;
        const vec3& p = b.nodes[i].p;
        float g = m_world ? ground_height_at(*m_world, p.x, p.z, top + 20.0f) : 0.0f;
        if (g - p.y > lift && getenv("BL_LIFTDBG")) fprintf(stderr, "lift: node %zu p (%.3f %.3f %.3f) ground %.3f -> lift %.3f\n", i, p.x, p.y, p.z, g, g - p.y);
        lift = std::max(lift, g - p.y);
    }
    if (lift > -1e8f)
        for (auto& n : b.nodes) n.p.y += lift + 0.03f;
    b.compute_aabb();
    b.max_speed = 0;
    b.wake();
    // drivetrain state
    Drivetrain& dr = *m_drive;
    dr.rpm = dr.idle_rpm;
    dr.gear = dr.has_engine ? 1 : 0;
    dr.clutch = dr.clutch_torque = dr.cur_acc = dr.auto_acc = 0;
    dr.shifting = dr.post_shifting = false;
    dr.dir_state = 0;
    if (m_visual) m_visual->mark_dirty();
}

void Vehicle::recover() {
    vec3 p = position();
    vec3 f = forward();
    float yaw = std::atan2(f.x, f.z) * kRad2Deg;
    if (m_world) p.y = ground_height_at(*m_world, p.x, p.z, p.y + 30.0f);
    reset(p, yaw);
}

void Vehicle::launch(vec3 v) {
    SoftBody& b = *body;
    b.set_velocity(v);
    vec3 up = this->up();
    // tread and rim nodes rotate so that the contact patch doesn't slip: v_node = v + w x r
    for (auto& w : b.wheels) {
        vec3 hub = (b.nodes[w.axle0].p + b.nodes[w.axle1].p) * 0.5f;
        vec3 omega = cross(up, v) / std::max(0.05f, w.radius);
        for (uint32_t ni : w.nodes) b.nodes[ni].v = v + cross(omega, b.nodes[ni].p - hub);
        for (uint32_t ni : w.rim) b.nodes[ni].v = v + cross(omega, b.nodes[ni].p - hub);
        w.speed = w.avg_speed = length(v);
    }
    m_drive->speed = length(v);
    last_com_vel = v;
    peak_g = 0;
    b.wake();
}

void Vehicle::set_input(const VehicleInput& in) { m_input = in; }

void Vehicle::substep(SoftBody& b, float dt) {
    m_drive->update(dt, m_input, b);
    if (assist_speed >= 0) {
        // crash-test tow: accelerate every node equally towards the target speed (compensates drag, engine braking)
        vec3 v = b.average_velocity();
        float err = assist_speed - dot(v, assist_dir);
        float acc = clampf(err * 8.0f, -30.0f, 30.0f);
        vec3* f = b.force.data();
        for (size_t i = 0; i < b.nodes.size(); i++) f[i] += assist_dir * (acc * b.nodes[i].mass);
    }
}

void Vehicle::update_frame(float dt) {
    // crash telemetry: peak deceleration of the centre of mass (filtered over the frame)
    if (dt > 0 && !body->sleeping) {
        vec3 v = body->average_velocity();
        float g = length(v - last_com_vel) / dt / 9.81f;
        if (g < 200.0f) peak_g = std::max(peak_g, g);
        last_com_vel = v;
    }
    // keep the player vehicle awake while it's being driven
    body->can_sleep = !(is_player && (m_input.throttle > 0 || m_input.brake > 0 || m_input.steer != 0)) && !ai;
    if (!body->can_sleep && body->sleeping) body->wake();
}

void Vehicle::update_visuals() {
    if (m_visual) m_visual->update(*body, m_drive->dir_state);
    if (m_sheet) m_sheet->update(*body, m_sheet_first);
    if (!body->fem.empty()) {
        if (!m_frame) {
            m_frame = std::make_unique<FrameVisual>();
            m_frame->mat = frame_tube_material();
            if (!body->fem.tris.empty()) { // (the triangle elements' plates: the shell's colour, else a red paint)
                auto m = std::make_shared<Material>(*frame_plate_material());
                m->name = name + " plates";
                const vec3 c = !m_def.fem_shells.empty() && m_def.fem_shells[0].color.x >= 0 ? m_def.fem_shells[0].color : vec3(0.78f, 0.12f, 0.10f);
                m->color = vec4(c, 1.0f);
                m->specular = 0.7f, m->gloss = 60.0f;
                m_frame->plate_mat = m;
            }
        }
        m_frame->update(*body);
    }
}

void Vehicle::draw(Renderer& r, InstanceCollector&, const DebugView& dbg) {
    if (m_visual && !dbg.hide_meshes) m_visual->draw(r, ghost);
    if (m_frame && !dbg.hide_meshes && ghost > 0.001f) {
        m_frame->upload();
        m_frame->draw(r);
    }
    if (m_sheet) {
        m_sheet->upload(m_sheet_first);
        m_sheet_first = false;
        static const bool no_panels = getenv("BL_NOPANELS") != nullptr; // (the frame alone: pictures of its tubes)
        if (!dbg.hide_meshes && ghost > 0.001f && !no_panels) m_sheet->draw(r);
    }
}

void Vehicle::make_sheet_body(const ShellMaterial& mat, float kg_m2, MaterialPtr visual, float thickness, const std::vector<std::array<int, 3>>* only,
                              const std::vector<int>* only_mat, const std::vector<SheetMaterial>* extra) {
    SoftBody& b = *body;
    if (m_sheet || b.tris.empty()) return;
    // the collision cab triangles become the sheet (their nodes are the frame's: the sheet hangs on it)
    std::vector<Triangle> cab;
    cab.swap(b.tris);
    std::set<std::array<int, 3>> keys;
    auto key = [](int a, int b, int c) {
        std::array<int, 3> k{a, b, c};
        std::sort(k.begin(), k.end());
        return k;
    };
    std::map<std::array<int, 3>, int> mat_of; // (the material of each listed triangle: 0 mat, k extra[k - 1])
    if (only)
        for (size_t i = 0; i < only->size(); i++) {
            const auto& t = (*only)[i];
            keys.insert(key(t[0], t[1], t[2]));
            if (only_mat && i < only_mat->size() && extra && (*only_mat)[i] > 0 && (*only_mat)[i] <= (int)extra->size()) mat_of[key(t[0], t[1], t[2])] = (*only_mat)[i];
        }
    for (const Triangle& t : cab) {
        if (only && !keys.count(key((int)t.a, (int)t.b, (int)t.c))) {
            b.tris.push_back(t); // (a plain collision triangle)
            continue;
        }
        const vec3 pa = b.nodes[t.a].p, pb = b.nodes[t.b].p, pc = b.nodes[t.c].p;
        if (length(cross(pb - pa, pc - pa)) < 1e-5f) continue; // (a degenerate triangle has no hinge)
        auto uv = [&](uint32_t n) {
            const vec3 p = m_spawn_nodes[n].p; // (definition space: the skin unrolled across x and up-and-over)
            return vec2(p.x * 0.25f, (p.z + p.y) * 0.3f);
        };
        b.add_shell(t.a, t.b, t.c, uv(t.a), uv(t.b), uv(t.c));
        auto it = mat_of.find(key((int)t.a, (int)t.b, (int)t.c));
        if (it != mat_of.end()) b.shells.back().mat = (uint8_t)it->second;
    }
    b.shell_mat = mat;
    b.shell_mat_extra.clear();
    if (extra)
        for (const SheetMaterial& e : *extra) b.shell_mat_extra.push_back(e.mat);
    // (the vehicle places, repairs and drives the body by the definition's node numbers: a body of the sheet alone, no
    // beam, would be put in Morton order here, and the editor's drum of 64 triangles came apart on its first frame)
    b.keep_node_order = true;
    b.finalize_shells(kg_m2, kDefaultDt, 7, true);
    // the sheet's welds on the frame (`welds`): built on the sheet as spawned, kept for the resets
    int bad_welds = 0;
    for (const auto& w : m_def.welds)
        if (w.anchor < 0 || w.node < 0 || !b.add_weld((uint32_t)w.anchor, (uint32_t)w.node, w.radius, w.brk, w.k, kDefaultDt / (float)(1 << b.shell_min_shift), w.anchor2, w.t))
            bad_welds++;
    if (bad_welds) log_warn("vehicle '%s': %d welds on no sheet node", name.c_str(), bad_welds);
    if (!b.welds.empty()) log_info("vehicle '%s': the sheet on %zu welds (%zu nodes held), membrane %.3g N/m, %d short steps", name.c_str(), b.welds.size(), b.weld_nodes.size(), b.shell_mat.membrane, 1 << b.shell_min_shift);
    m_spawn_welds = b.welds;

    m_mass = 0;
    for (const Node& n : b.nodes) m_mass += n.mass;
    m_spawn_shells = b.shells;
    m_spawn_tris = b.tris;
    m_spawn_info = b.info;
    m_spawn_base_mass = b.node_base_mass;
    // (the nodes' pristine state was taken before the sheet: the masses and flags as they are now)
    for (size_t i = 0; i < m_spawn_nodes.size() && i < b.nodes.size(); i++) {
        m_spawn_nodes[i].mass = b.nodes[i].mass;
        m_spawn_nodes[i].inv_mass = b.nodes[i].inv_mass;
    }
    m_sheet = std::make_unique<ShellVisual>();
    m_sheet->mat = visual;
    m_sheet->mats.assign(1, visual);
    if (extra)
        for (const SheetMaterial& e : *extra) m_sheet->mats.push_back(e.visual ? e.visual : visual);
    m_sheet->thickness = thickness;
    m_sheet_first = true;
}

void Vehicle::place_definition(vec3 origin) {
    SoftBody& b = *body;
    for (size_t i = 0; i < b.nodes.size() && i < m_spawn_nodes.size(); i++) {
        b.nodes[i].p = m_spawn_nodes[i].p + origin;
        b.nodes[i].v = vec3(0);
    }
    for (auto& f : b.frames) f.w = vec3(0);
    b.fem.set_orientation(quat()); // (the definition's own orientation: the frame nodes' rest one)
    b.compute_aabb();
    if (m_visual) m_visual->mark_dirty();
}

void Vehicle::set_node_position(int i, vec3 p) {
    if (i < 0 || i >= (int)body->nodes.size()) return;
    body->nodes[i].p = p;
    body->nodes[i].v = vec3(0);
    if (m_visual) m_visual->mark_dirty();
}

vec3 Vehicle::position() const { return body->nodes[m_cam_center].p; }
vec3 Vehicle::forward() const {
    vec3 f = body->nodes[m_cam_center].p - body->nodes[m_cam_back].p;
    return normalize_or(f, vec3(0, 0, 1));
}
vec3 Vehicle::left() const {
    vec3 f = forward();
    vec3 l = body->nodes[m_cam_left].p - body->nodes[m_cam_center].p;
    l = normalize_or(l - f * dot(l, f), vec3(1, 0, 0));
    if (dot(cross(f, l), vec3(0, 1, 0)) < 0 && dot(m_local_left, vec3(0, 0, 1)) < 0) l = -l;
    return l;
}
vec3 Vehicle::up() const { return normalize_or(cross(forward(), left()), vec3(0, 1, 0)); }
vec3 Vehicle::velocity() const { return body->nodes[m_cam_center].v; }
vec3 Vehicle::cockpit_pos() const {
    if (m_cinecam >= 0) return body->nodes[m_cinecam].p;
    return position() + up() * 1.3f;
}
// ground speed of the body (the driven wheels' speed includes their slip)
float Vehicle::speed_kmh() const { return dot(velocity(), forward()) * 3.6f; }
float Vehicle::steer_state() const { return m_drive ? m_drive->dir_state : 0.0f; }

float Vehicle::rpm() const { return m_drive->rpm; }
float Vehicle::max_rpm() const { return m_drive->max_rpm; }
int Vehicle::gear() const { return m_drive->gear; }
int Vehicle::num_gears() const { return m_drive->num_gears; }
float Vehicle::throttle() const { return m_drive->cur_acc; }
float Vehicle::brake() const { return m_drive->brake_in; }
bool Vehicle::has_engine() const { return m_drive->has_engine; }
int Vehicle::broken_beams() const { return body->stats.broken_beams; }

} // namespace bl

// --list helper
void beamlab_print_lists() {
    printf("Scenes:\n");
    const auto& s = bl::scene_registry();
    for (size_t i = 0; i < s.size(); i++) printf("  %2d  %-16s %s\n", (int)i, s[i].id.c_str(), s[i].name.c_str());
    printf("Vehicles:\n");
    for (const auto& e : bl::vehicle_registry()) printf("  %-45s %s\n", e.id.c_str(), e.title.c_str());
}
