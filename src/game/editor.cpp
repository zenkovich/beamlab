// The model editor: the model and its history, files, the editor's stage, the tests (drive, physics without
// gravity), the graphics preview, and the edits the tools and panels call (editor.h).
#include "game/editor.h"

#include "core/util.h"
#include "game/editor_internal.h"
#include "game/game.h"
#include "vehicle/builder.h"
#include "vehicle/vehicle.h"

#include "imgui.h"

#include <GLFW/glfw3.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <set>
#include <functional>
#include <map>
#include <set>

namespace bl {

using namespace edit_detail;

namespace {

constexpr int kUndoMax = 64;

uint64_t mix(uint64_t h, uint64_t v) { return h ^ (v + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2)); }
uint64_t mixf(uint64_t h, float f) {
    uint32_t u;
    memcpy(&u, &f, 4);
    return mix(h, u);
}
uint64_t mixv(uint64_t h, vec3 v) { return mixf(mixf(mixf(h, v.x), v.y), v.z); }
uint64_t mixs(uint64_t h, const std::string& s) { return mix(h, std::hash<std::string>{}(s)); }

} // namespace

ModelEditor::ModelEditor(Game& g) : m_game(g) {
    m_model = edit::make_cart(2.2f, 1.2f, 0.7f, 400);
    m_tri_mat = std::make_shared<Material>();
    m_tri_mat->unlit = true;
    m_tri_mat->blend = true;
    m_tri_mat->double_sided = true;
    m_tri_mat->cast_shadow = false;
    m_tri_mat->color = vec4(0.35f, 0.65f, 1.0f, 0.22f);
    m_tri_mat->name = "editor_tris";
    m_ref_mat = std::make_shared<Material>();
    m_ref_mat->double_sided = true;
    m_ref_mat->cast_shadow = false;
    m_ref_mat->color = vec4(0.72f, 0.74f, 0.8f, 1.0f);
    m_ref_mat->name = "editor_ref";
    m_views[1].ortho = 1, m_views[2].ortho = 2, m_views[3].ortho = 3;
    for (View& v : m_views) v.dist = 7.0f;
}

ModelEditor::~ModelEditor() = default;

// ------------------------------------------------------------------------------------------------ open / close
void ModelEditor::enter_stage() {
    const auto& reg = scene_registry();
    int idx = -1;
    for (int i = 0; i < (int)reg.size(); i++)
        if (reg[i].id == "editor") idx = i;
    if (idx >= 0 && m_game.scene_index != idx) m_game.load_scene(idx);
    m_scene = m_game.scene_index;
    m_preview = m_test = nullptr; // (the scene's load removed every vehicle)
    m_preview_sig = 0;
    m_preview_wait = 0;
    m_game.paused = true;
    place_on_floor();
}

void ModelEditor::place_on_floor() {
    // the model's lowest point (a node, the bottom of a wheel) on the floor, a hair above it (the faces on the ground
    // do not flicker); the work plane there. A vehicle's definition may have its ground anywhere (RoR's often below 0)
    float y = 1e30f;
    for (const edit::Node& n : m_model.nodes) y = std::min(y, n.p.y);
    for (const edit::Wheel& w : m_model.wheels)
        if (w.n1 >= 0 && w.n2 >= 0 && w.n1 < (int)m_model.nodes.size() && w.n2 < (int)m_model.nodes.size())
            y = std::min(y, std::min(m_model.nodes[w.n1].p.y, m_model.nodes[w.n2].p.y) - w.radius);
    if (y > 1e29f) y = 0;
    m_origin = vec3(0, m_game.ground_height(0, 0) + 0.005f - y, 0);
    m_work_y = y;
}

void ModelEditor::open() {
    if (m_active) return;
    m_saved_cam = m_game.cam;
    m_saved_paused = m_game.paused;
    m_prev_scene = m_game.scene_index;
    m_active = true;
    m_mode = Mode::Edit;
    enter_stage();
    const vec3 c = m_model.centroid();
    for (View& v : m_views) v.target = vec3(c.x, std::max(c.y, 0.3f), c.z);
    frame_selection();
    snprintf(m_title_buf, sizeof m_title_buf, "%s", m_model.title.c_str());
    clear_selection();
    m_status.clear();
    if (!m_model.ref_path.empty() && m_ref_loaded != m_model.ref_path) ref_load(m_model.ref_path);
    // (screenshots and scripted checks)
    if (getenv("BL_EDITOR_EMPTY")) set_model(edit::make_empty(), "");
    if (const char* tpl = getenv("BL_EDITOR_TPL")) { // (a template: index[,nx,nz,sx,sy,sz,mm])
        int n0 = m_tpl_n[0], n1 = m_tpl_n[1];
        sscanf(tpl, "%d,%d,%d,%f,%f,%f,%f", &m_tpl, &n0, &n1, &m_tpl_size.x, &m_tpl_size.y, &m_tpl_size.z, &m_tpl_mm);
        m_tpl_n[0] = n0, m_tpl_n[1] = n1;
        set_model(make_template(m_tpl), "");
    }
    if (const char* f = getenv("BL_EDITOR_FILE")) load(f); // (a model file as it is, not saved unless asked)
    if (const char* id = getenv("BL_EDITOR_VEHICLE")) { // (a copy of a vehicle, graphics included)
        if (const VehicleEntry* e = find_vehicle(id)) {
            import_vehicle_file(e->file);
            m_open_graphics_tab = true;
        }
    }
    if (getenv("BL_EDITOR_QUAD")) m_quad = true;
    if (const char* f = getenv("BL_EDITOR_FACES")) m_face_mode = std::clamp(atoi(f), 0, 3); // (scripted checks: the shapes' faces)
    if (const char* bb = getenv("BL_EDITOR_BARREL")) { // (scripted checks: a circle pulled into a drum, as by hand)
        float r = 0.3f, h = 0.9f, y = 0.3f;
        int sides = 16;
        sscanf(bb, "%f,%f,%d,%f", &r, &h, &sides, &y);
        m_circle_sides = sides;
        if (!getenv("BL_EDITOR_FACES")) m_face_mode = 2;
        int top = 0;
        for (int i = 0; i < 4; i++)
            if (m_views[i].ortho == 3) top = i;
        make_circle(vec3(0, y, 0), vec3(r, y, 0), top);
        if (!m_model.tris.empty()) {
            pushpull_region((int)m_model.tris.size() - 1, m_pp_region, m_pp_n);
            pushpull_apply(m_pp_n.y > 0 ? h : -h);
            m_pp_region.clear();
        }
        printf("editor: barrel of %zu nodes, %zu beams, %zu triangles\n", m_model.nodes.size(), m_model.beams.size(), m_model.tris.size());
    }
    if (getenv("BL_EDITOR_PHYSICS")) test_physics();
    if (const char* sk = getenv("BL_EDITOR_SKEL")) m_skel_alpha = (float)atof(sk);
    if (const char* go = getenv("BL_EDITOR_GFXONLY")) set_gfx_only(atoi(go)), m_gfx_only_grow = getenv("BL_EDITOR_GFXGROW") ? atoi(getenv("BL_EDITOR_GFXGROW")) : 0;
    if (const char* ref = getenv("BL_EDITOR_REF")) ref_load(ref);
    if (const char* t = getenv("BL_EDITOR_TOOL")) set_tool((Tool)std::clamp(atoi(t), 0, (int)Tool::Count - 1));
    if (getenv("BL_EDITOR_SELECT")) { // (the front half of the model selected)
        const float cx = m_model.centroid().x;
        for (int i = 0; i < (int)m_model.nodes.size(); i++)
            if (m_model.nodes[i].p.x < cx) m_sel.push_back(i);
        for (int i = 0; i < (int)m_model.beams.size(); i++)
            if (is_selected(m_model.beams[i].a) && is_selected(m_model.beams[i].b)) m_sel_beams.push_back(i);
        for (int i = 0; i < (int)m_model.tris.size(); i++)
            if (is_selected(m_model.tris[i].a) && is_selected(m_model.tris[i].b) && is_selected(m_model.tris[i].c)) m_sel_tris.push_back(i);
    }
    if (const char* dm = getenv("BL_EDITOR_DEFORM")) { // (the deformation demo; a number: a preset)
        enter_deform();
        if (atoi(dm) >= 0 && *dm != 'x') m_demo_amount = 0.7f, deform_preset(atoi(dm));
    }
    if (getenv("BL_EDITOR_GFXSEL") && !m_model.flexbodies.empty()) m_gfx_kind = 1, m_gfx_sel = std::clamp(atoi(getenv("BL_EDITOR_GFXSEL")), 0, (int)m_model.flexbodies.size() - 1);
}

void ModelEditor::open_vehicle(Vehicle* v) {
    if (v) {
        edit::Model m;
        std::vector<std::string> notes;
        if (edit::import_document(v->def(), m, notes)) {
            std::string text;
            if (read_text_file(v->def().path, text)) edit::read_markers(text, m);
            if (text.find(";written by the BeamLab model editor") == std::string::npos) m.title += " (edited)";
            set_model(m, "");
            m_notes = notes;
            m_dirty = true;
        } else {
            toast("This vehicle has no nodes to edit");
        }
    }
    open();
}

void ModelEditor::close() {
    if (!m_active) return;
    if (m_mode != Mode::Edit) end_mode();
    drop_preview();
    m_active = false;
    m_game.debug.hide_terrain = false;
    if (m_prev_scene >= 0 && m_prev_scene != m_scene && m_game.scene_index == m_scene) m_game.load_scene(m_prev_scene);
    m_game.cam = m_saved_cam;
    m_game.paused = m_saved_paused;
}

void ModelEditor::set_model(const edit::Model& m, const std::string& file) {
    m_model = m;
    m_file = file;
    m_undo.clear();
    m_redo.clear();
    m_dirty = false;
    m_notes.clear();
    clear_selection();
    m_hidden.assign(m_model.nodes.size(), 0);
    m_group = 0;
    m_layer = 0;
    m_gfx_kind = 0, m_gfx_sel = -1;
    m_gfx_only_base = (int)m_model.nodes.size();
    m_preview_sig = 0;
    drop_preview();
    if (m_active) place_on_floor();
    frame_selection();
    snprintf(m_title_buf, sizeof m_title_buf, "%s", m_model.title.c_str());
    snprintf(m_ref_buf, sizeof m_ref_buf, "%s", m_model.ref_path.c_str());
    if (m_model.ref_path != m_ref_loaded) {
        if (m_model.ref_path.empty()) m_ref_v.clear(), m_ref_idx.clear(), m_ref_loaded.clear();
        else ref_load(m_model.ref_path);
    }
}

void ModelEditor::push_undo() {
    m_undo.push_back(m_model);
    if ((int)m_undo.size() > kUndoMax) m_undo.erase(m_undo.begin());
    m_redo.clear();
    m_dirty = true;
}

void ModelEditor::undo() {
    if (m_undo.empty()) return;
    m_redo.push_back(m_model);
    m_model = m_undo.back();
    m_undo.pop_back();
    clear_selection();
    m_hidden.resize(m_model.nodes.size(), 0);
    m_dirty = true;
}

void ModelEditor::redo() {
    if (m_redo.empty()) return;
    m_undo.push_back(m_model);
    m_model = m_redo.back();
    m_redo.pop_back();
    clear_selection();
    m_hidden.resize(m_model.nodes.size(), 0);
    m_dirty = true;
}

// ------------------------------------------------------------------------------------------------ files
std::string ModelEditor::folder() const { return asset_path("vehicles/" + (m_model.home.empty() ? std::string("editor") : m_model.home)); }

std::string ModelEditor::save_path() const {
    // (never over a vehicle's own file: an edited copy of a mod is saved next to it under another name)
    std::string stem = m_model.slug();
    for (int k = 0; k < 100; k++) {
        const std::string path = path_join(folder(), stem + ".truck");
        std::string text;
        if (!file_exists(path) || (read_text_file(path, text) && text.find(";written by the BeamLab model editor") != std::string::npos)) return path;
        stem = m_model.slug() + "_edit" + (k ? std::to_string(k + 1) : std::string());
    }
    return path_join(folder(), m_model.slug() + "_edit.truck");
}

bool ModelEditor::save(std::string* err) {
    m_model.title = trim(m_title_buf);
    if (m_model.title.empty()) m_model.title = "Model";
    std::error_code ec;
    std::filesystem::create_directories(folder(), ec);
    if (m_model.home == "editor" || m_model.home.empty()) {
        const std::string src = path_join(folder(), "SOURCE.txt");
        if (!file_exists(src)) {
            std::ofstream f(src);
            f << "Model editor\nModels made in BeamLab's model editor (Editor menu).\n";
        }
    }
    const std::string path = save_path();
    std::ofstream f(path, std::ios::binary);
    if (!f) {
        if (err) *err = "cannot write " + path;
        return false;
    }
    f << edit::write_truck(m_model);
    f.close();
    m_file = path;
    m_dirty = false;
    refresh_vehicle_registry();
    log_info("editor: saved %s", path.c_str());
    return true;
}

bool ModelEditor::load(const std::string& path) {
    ror::Document d;
    if (!ror::parse_truck_file(path, d)) {
        toast("Cannot read " + path_stem(path));
        return false;
    }
    edit::Model m;
    std::vector<std::string> notes;
    if (!edit::import_document(d, m, notes)) {
        toast("No nodes in " + path_stem(path));
        return false;
    }
    std::string text;
    if (read_text_file(path, text)) edit::read_markers(text, m);
    set_model(m, path);
    m_notes = notes;
    return true;
}

bool ModelEditor::import_vehicle_file(const std::string& path) {
    if (!load(path)) return false;
    std::string text;
    if (read_text_file(path, text) && text.find(";written by the BeamLab model editor") == std::string::npos) m_model.title += " (edited)";
    snprintf(m_title_buf, sizeof m_title_buf, "%s", m_model.title.c_str());
    m_file.clear(); // (an import: saved as a model of its own, next to the vehicle it came from)
    m_dirty = true;
    return true;
}

// ------------------------------------------------------------------------------------------------ tests and preview
bool ModelEditor::alive(const Vehicle* v) const {
    if (!v) return false;
    for (const auto& x : m_game.vehicles)
        if (x.get() == v) return true;
    return false;
}

std::string ModelEditor::preview_text() const {
    edit::Model m = m_model;
    m.title = trim(m_title_buf).empty() ? m.title : trim(m_title_buf);
    return edit::write_truck(m, true); // (every mesh, switched off or not: the preview draws them by their state)
}

Vehicle* ModelEditor::spawn_from_model(bool sheet) {
    // the model in memory: its file name only places it in its folder (where its meshes and materials are)
    VehicleEntry e;
    e.folder = m_model.home.empty() ? "editor" : m_model.home;
    e.id = e.folder + "/.editor_preview";
    e.file = path_join(folder(), ".editor_preview.truck");
    e.title = m_model.title;
    e.text = preview_text();
    e.split_parts = true;
    if (const char* dump = getenv("BL_EDITOR_DUMP")) { // (checks: the definition the editor spawns)
        std::ofstream f(dump, std::ios::binary);
        f << e.text;
    }
    std::string err;
    auto v = Vehicle::create(e, m_game.world, m_origin, 0.0f, &err);
    if (!v) {
        m_preview_error = err.empty() ? "the model does not build" : err;
        return nullptr;
    }
    m_preview_error.clear();
    Vehicle* p = v.get();
    if (getenv("BL_EDITOR_LOG"))
        for (const std::string& w : p->load_warnings()) printf("editor: preview warning: %s\n", w.c_str());
    m_game.vehicles.push_back(std::move(v));
    if (sheet) apply_vehicle_sheet_body(p, false);
    p->place_definition(m_origin);
    return p;
}

void ModelEditor::drop_preview() {
    if (m_preview && alive(m_preview)) m_game.remove_vehicle(m_preview);
    m_preview = nullptr;
}

uint64_t ModelEditor::preview_signature() const {
    const edit::Model& m = m_model;
    uint64_t h = mixs(1469598103934665603ull, m.home);
    h = mix(h, m.nodes.size());
    for (const edit::Node& n : m.nodes) h = mixv(h, n.p);
    for (const edit::Tri& t : m.tris) h = mix(mix(mix(mix(mix(h, t.a), t.b), t.c), t.submesh + 7), (uint64_t)t.collision | (uint64_t)t.shell << 1 | (uint64_t)t.fem << 2);
    for (const edit::Wheel& w : m.wheels) {
        h = mix(mix(mix(mix(h, w.type), w.n1), w.n2), w.rays);
        h = mixf(mixf(mixf(h, w.radius), w.rim_radius), w.width);
        h = mixs(mixs(h, w.rim_mesh), w.tyre_material);
        h = mix(h, (uint64_t)w.side);
    }
    for (const edit::Flexbody& f : m.flexbodies) {
        h = mix(mix(mix(h, f.ref), f.x), f.y);
        h = mixs(mixv(mixv(h, f.offset), f.rot), f.mesh);
        for (int n : f.forset) h = mix(h, n);
    }
    for (const edit::Prop& p : m.props) h = mixs(mixs(mixv(mixv(mix(mix(mix(h, p.ref), p.x), p.y), p.offset), p.rot), p.mesh), p.extra);
    for (const edit::Submesh& s : m.submeshes) {
        h = mix(h, s.texcoords.size() + (s.backmesh ? 1000000 : 0));
        for (const auto& [n, uv] : s.texcoords) h = mixf(mixf(mix(h, n), uv.x), uv.y);
    }
    h = mix(mixv(mixs(h, m.cab_material), m.skin_color), (uint64_t)m.skin | (uint64_t)(m.shell_count() > 0) << 1);
    h = mix(h, m.managed_materials.size());
    return h | 1;
}

void ModelEditor::update_preview(float dt) {
    if (m_mode != Mode::Edit && m_mode != Mode::Deform) return;
    if (m_preview && !alive(m_preview)) m_preview = nullptr;
    if (!m_show_gfx || m_model.nodes.empty()) {
        drop_preview();
        return;
    }
    static uint64_t last_sig = 0;
    const uint64_t sig = preview_signature();
    if (sig != last_sig) {
        last_sig = sig;
        m_preview_wait = 0;
    } else {
        m_preview_wait += dt;
    }
    const bool busy = m_drag == Drag::MoveFree || m_drag == Drag::MoveAxis || m_op;
    const bool rebuild = m_mode == Mode::Deform ? !m_preview : sig != m_preview_sig && (m_preview_wait > 0.35f || !m_preview) && !busy;
    if (rebuild) {
        // (a rebuild: the meshes are bound to the nodes where they are now, as in a spawn)
        drop_preview();
        m_preview = spawn_from_model(false);
        m_preview_sig = sig;
        m_preview_rest.clear();
        if (m_preview)
            for (const phys::Node& nd : m_preview->body->nodes) m_preview_rest.push_back(nd.p);
    }
    if (!m_preview) return;
    if (m_mode == Mode::Deform) {
        // the demo: the model's nodes displaced, the wheels' own nodes with their axles
        apply_demo_to_preview();
    } else {
        // live: the nodes follow the model (the meshes skinned to them deform while a drag goes on)
        const int n = std::min((int)m_model.nodes.size(), (int)m_preview->body->nodes.size());
        for (int i = 0; i < n; i++) {
            const vec3 p = to_world(m_model.nodes[i].p);
            if (length2(m_preview->body->nodes[i].p - p) > 1e-10f) m_preview->set_node_position(i, p);
        }
    }
    m_preview->ghost = m_gfx_alpha;
    apply_part_states(m_preview);
}

void ModelEditor::test_drive() {
    if (m_mode != Mode::Edit) end_mode();
    std::string err;
    if (!save(&err)) {
        toast(err);
        return;
    }
    const std::string id = m_model.home + "/" + path_stem(m_file);
    if (!find_vehicle(id)) {
        toast("The saved model is not in the vehicle list");
        return;
    }
    drop_preview();
    Vehicle* v = m_game.spawn_vehicle(id, m_origin, 0.0f, true);
    if (!v) {
        toast("Spawn failed (see the log)");
        return;
    }
    m_test = v;
    m_mode = Mode::Drive;
    m_game.paused = false;
    m_game.debug.hide_terrain = false;
    clear_selection();
}

void ModelEditor::test_physics() {
    if (m_mode != Mode::Edit) end_mode();
    drop_preview();
    m_test = spawn_from_model(true);
    if (!m_test) {
        toast("The model does not build: " + m_preview_error);
        return;
    }
    m_saved_gravity = m_game.world.settings.gravity;
    if (getenv("BL_EDITOR_GRAVITY")) m_phys_gravity = true; // (scripted checks: with gravity, hung from a crane, slowed down)
    if (const char* sp = getenv("BL_EDITOR_SPEED")) m_phys_time = (float)atof(sp);
    m_game.world.settings.gravity = m_phys_gravity ? m_saved_gravity : vec3(0);
    if (const char* c = getenv("BL_EDITOR_CRANE")) m_game.crane_vehicle(m_test, (float)atof(c));
    m_saved_debug_beams = m_game.debug.beams;
    m_game.debug.beams = true;
    m_saved_game_tool = (int)m_game.tool;
    m_saved_time_scale = m_game.world.settings.time_scale;
    m_game.world.settings.time_scale = m_phys_time; // (the test's own speed, kept from one test to the next)
    m_mode = Mode::Physics;
    m_game.paused = false;
    m_test->body->wake();
    clear_selection();
}

void ModelEditor::set_phys_time(float ts) {
    m_phys_time = ts;
    if (m_mode == Mode::Physics) m_game.world.settings.time_scale = ts;
}

void ModelEditor::end_mode() {
    if (m_mode == Mode::Edit) return;
    if (m_mode == Mode::Deform) {
        // (back to the model's shape: the preview's nodes where the model has them, the wheels' at rest)
        m_demo_off.assign(m_model.nodes.size(), vec3(0));
        m_demo_wave = m_demo_drag = false;
        apply_demo_to_preview();
        m_gfx_alpha = m_demo_saved_alpha[0], m_skel_alpha = m_demo_saved_alpha[1];
        m_mode = Mode::Edit;
        m_status.clear();
        return;
    }
    m_game.grab_end();
    if (m_test && alive(m_test)) m_game.remove_vehicle(m_test);
    m_test = nullptr;
    if (m_mode == Mode::Physics) {
        m_game.world.settings.gravity = m_saved_gravity;
        m_game.debug.beams = m_saved_debug_beams;
        // (the game's tool back; the shots gone)
        m_game.laser_release();
        m_game.clear_projectiles();
        m_game.tool = (bl::Tool)m_saved_game_tool;
        m_game.world.settings.time_scale = m_saved_time_scale;
        m_game.cursor_valid = false;
        m_phys_held = false;
    }
    m_mode = Mode::Edit;
    m_game.paused = true;
    m_preview_sig = 0; // (a new preview)
}

// ------------------------------------------------------------------------------------------------ selection
std::vector<int>& ModelEditor::sel_of(Elem k) {
    switch (k) {
    case Elem::Beam: return m_sel_beams;
    case Elem::Shock: return m_sel_shocks;
    case Elem::Hydro: return m_sel_hydros;
    case Elem::Tri: return m_sel_tris;
    case Elem::Wheel: return m_sel_wheels;
    case Elem::Joint: return m_sel_joints;
    default: return m_none;
    }
}

bool ModelEditor::is_selected(int n) const { return std::binary_search(m_sel.begin(), m_sel.end(), n); }
bool ModelEditor::selection_empty() const { return m_sel.empty() && selected_elements() == 0; }
int ModelEditor::selected_elements() const {
    return (int)(m_sel_beams.size() + m_sel_shocks.size() + m_sel_hydros.size() + m_sel_tris.size() + m_sel_wheels.size() + m_sel_joints.size());
}

void ModelEditor::select_node(int n, bool add, bool toggle) {
    if (n < 0 || !node_pickable(n)) return;
    if (toggle) sorted_toggle(m_sel, n);
    else if (add) sorted_insert(m_sel, n);
    else {
        clear_selection();
        m_sel = {n};
    }
}

void ModelEditor::select_elem(Elem k, int i, bool add, bool toggle) {
    if (k == Elem::None || i < 0 || !elem_pickable(k, i)) return;
    if (toggle) sorted_toggle(sel_of(k), i);
    else if (add) sorted_insert(sel_of(k), i);
    else {
        clear_selection();
        sel_of(k) = {i};
    }
}

void ModelEditor::clear_selection() {
    m_sel.clear();
    m_sel_beams.clear(), m_sel_shocks.clear(), m_sel_hydros.clear(), m_sel_tris.clear(), m_sel_wheels.clear(), m_sel_joints.clear();
    m_chain = m_chain_start = -1;
    m_picks.clear();
    m_drag = Drag::None;
}

bool ModelEditor::node_shown(int n) const { return m_model.node_visible(n) && !(n < (int)m_hidden.size() && m_hidden[n]) && !gfx_filtered(n); }
bool ModelEditor::node_pickable(int n) const { return node_shown(n) && !m_model.node_locked(n); }

bool ModelEditor::elem_shown(Elem k, int i) const {
    const edit::Model& M = m_model;
    auto nh = [&](int n) { return (n < (int)m_hidden.size() && m_hidden[n]) || gfx_filtered(n); };
    switch (k) {
    case Elem::Beam: return i < (int)M.beams.size() && M.beam_visible(i) && !nh(M.beams[i].a) && !nh(M.beams[i].b);
    case Elem::Shock: return i < (int)M.shocks.size() && M.shock_visible(i) && !nh(M.shocks[i].a) && !nh(M.shocks[i].b);
    case Elem::Hydro: return i < (int)M.hydros.size() && M.hydro_visible(i) && !nh(M.hydros[i].a) && !nh(M.hydros[i].b);
    case Elem::Tri: return i < (int)M.tris.size() && M.tri_visible(i) && !nh(M.tris[i].a) && !nh(M.tris[i].b) && !nh(M.tris[i].c);
    case Elem::Wheel: return i < (int)M.wheels.size() && M.wheel_visible(i) && !nh(M.wheels[i].n1) && !nh(M.wheels[i].n2);
    case Elem::Joint: return i < (int)M.joints.size() && M.joint_visible(i) && !nh(M.joints[i].parent) && !nh(M.joints[i].child);
    default: return false;
    }
}

bool ModelEditor::elem_pickable(Elem k, int i) const {
    if (!elem_shown(k, i)) return false;
    const edit::Model& M = m_model;
    switch (k) {
    case Elem::Beam: return !M.layer_locked(M.beams[i].layer);
    case Elem::Shock: return !M.layer_locked(M.shocks[i].layer);
    case Elem::Hydro: return !M.layer_locked(M.hydros[i].layer);
    case Elem::Tri: return !M.layer_locked(M.tris[i].layer);
    case Elem::Wheel: return !M.layer_locked(M.wheels[i].layer);
    case Elem::Joint: return !M.layer_locked(M.joints[i].layer);
    default: return false;
    }
}

std::vector<int> ModelEditor::selection_with_twins() const {
    std::vector<int> out = m_sel;
    if (m_symmetry)
        for (int n : m_sel) {
            const int t = m_model.twin(n);
            if (t >= 0 && !is_selected(t) && !m_model.node_locked(t)) out.push_back(t);
        }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

std::vector<int> ModelEditor::selection_nodes_all() const {
    std::vector<int> out = m_sel;
    const edit::Model& M = m_model;
    for (int i : m_sel_beams) out.push_back(M.beams[i].a), out.push_back(M.beams[i].b);
    for (int i : m_sel_shocks) out.push_back(M.shocks[i].a), out.push_back(M.shocks[i].b);
    for (int i : m_sel_hydros) out.push_back(M.hydros[i].a), out.push_back(M.hydros[i].b);
    for (int i : m_sel_tris) out.push_back(M.tris[i].a), out.push_back(M.tris[i].b), out.push_back(M.tris[i].c);
    for (int i : m_sel_wheels) out.push_back(M.wheels[i].n1), out.push_back(M.wheels[i].n2);
    for (int i : m_sel_joints) out.push_back(M.joints[i].parent), out.push_back(M.joints[i].child);
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

void ModelEditor::delete_selection() {
    if (selection_empty()) return;
    push_undo();
    // elements first (highest index first keeps the others' indices), then the nodes with everything on them
    for (int i = (int)m_sel_beams.size() - 1; i >= 0; i--) m_model.remove_beam(m_sel_beams[i]);
    for (int i = (int)m_sel_shocks.size() - 1; i >= 0; i--) m_model.remove_shock(m_sel_shocks[i]);
    for (int i = (int)m_sel_hydros.size() - 1; i >= 0; i--) m_model.remove_hydro(m_sel_hydros[i]);
    for (int i = (int)m_sel_tris.size() - 1; i >= 0; i--) m_model.remove_tri(m_sel_tris[i]);
    for (int i = (int)m_sel_wheels.size() - 1; i >= 0; i--) m_model.remove_wheel(m_sel_wheels[i]);
    for (int i = (int)m_sel_joints.size() - 1; i >= 0; i--) m_model.remove_joint(m_sel_joints[i]);
    if (!m_sel.empty()) m_model.remove_nodes(selection_with_twins());
    m_hidden.assign(m_model.nodes.size(), 0);
    clear_selection();
    m_gfx_sel = -1;
}

void ModelEditor::duplicate_selection(vec3 offset) {
    if (m_sel.empty()) return;
    push_undo();
    const std::vector<int> ids = selection_with_twins();
    std::vector<int> map(m_model.nodes.size(), -1);
    for (int i : ids) {
        map[i] = (int)m_model.nodes.size();
        edit::Node n = m_model.nodes[i];
        n.p += is_selected(i) ? offset : mirror_z(offset);
        m_model.nodes.push_back(n);
        m_hidden.push_back(0);
    }
    const size_t nb = m_model.beams.size(), nt = m_model.tris.size();
    for (size_t k = 0; k < nb; k++) {
        const edit::Beam b = m_model.beams[k];
        if (map[b.a] >= 0 && map[b.b] >= 0 && m_model.add_beam(map[b.a], map[b.b], b.group) >= 0) m_model.beams.back().layer = b.layer;
    }
    for (size_t k = 0; k < nt; k++) {
        edit::Tri t = m_model.tris[k];
        if (map[t.a] >= 0 && map[t.b] >= 0 && map[t.c] >= 0) {
            t.a = map[t.a], t.b = map[t.b], t.c = map[t.c], t.submesh = -1, t.options.clear();
            m_model.tris.push_back(t);
        }
    }
    std::vector<int> sel;
    for (int i : m_sel) sel.push_back(map[i]);
    std::sort(sel.begin(), sel.end());
    clear_selection();
    m_sel = sel;
    m_status = "Duplicated: the copy is selected";
}

void ModelEditor::mirror_selection() {
    if (m_sel.empty()) return;
    push_undo();
    const int made = m_model.mirror(m_sel);
    m_hidden.resize(m_model.nodes.size(), 0);
    m_status = std::to_string(made) + " nodes mirrored";
}

int ModelEditor::twin_or_self(int n) const {
    const int t = m_model.twin(n);
    return t >= 0 ? t : (n >= 0 && n < (int)m_model.nodes.size() && std::fabs(m_model.nodes[n].p.z) < 1e-3f ? n : -1);
}

void ModelEditor::connect_selection(bool all_pairs) {
    if (m_sel.size() < 2) return;
    push_undo();
    const size_t nb = m_model.beams.size();
    if (all_pairs) {
        for (size_t i = 0; i < m_sel.size(); i++)
            for (size_t j = i + 1; j < m_sel.size(); j++) add_beam_sym(m_sel[i], m_sel[j]);
    } else {
        for (size_t i = 0; i + 1 < m_sel.size(); i++) add_beam_sym(m_sel[i], m_sel[i + 1]);
    }
    m_status = std::to_string(m_model.beams.size() - nb) + " beams added";
}

void ModelEditor::frame_selection() {
    vec3 c = m_model.centroid();
    float d = -1;
    const std::vector<int> nodes = selection_nodes_all();
    if (!nodes.empty()) {
        vec3 mn(1e30f), mx(-1e30f);
        for (int n : nodes) mn = vmin(mn, m_model.nodes[n].p), mx = vmax(mx, m_model.nodes[n].p);
        c = (mn + mx) * 0.5f;
        d = std::max(1.5f, length(mx - mn) * 1.3f);
    } else if (!m_model.nodes.empty()) {
        // (what is shown: with the graphics' nodes only, those)
        vec3 mn(1e30f), mx(-1e30f);
        for (int i = 0; i < (int)m_model.nodes.size(); i++)
            if (!m_gfx_only || node_shown(i)) mn = vmin(mn, m_model.nodes[i].p), mx = vmax(mx, m_model.nodes[i].p);
        if (mn.x <= mx.x) {
            c = (mn + mx) * 0.5f;
            d = std::max(m_gfx_only ? 1.0f : 3.0f, length(mx - mn) * 1.2f);
        }
    }
    for (View& v : m_views) {
        v.target = c;
        if (d > 0) v.dist = d;
    }
}

void ModelEditor::select_connected() {
    std::vector<char> in(m_model.nodes.size(), 0);
    for (int n : selection_nodes_all()) in[n] = 1;
    bool grown = true;
    while (grown) {
        grown = false;
        for (const edit::Beam& b : m_model.beams)
            if (in[b.a] != in[b.b]) in[b.a] = in[b.b] = 1, grown = true;
    }
    m_sel.clear();
    for (int i = 0; i < (int)in.size(); i++)
        if (in[i] && node_pickable(i)) m_sel.push_back(i);
    m_status = std::to_string(m_sel.size()) + " connected nodes selected";
}

void ModelEditor::select_invert() {
    std::vector<int> out;
    for (int i = 0; i < (int)m_model.nodes.size(); i++)
        if (!is_selected(i) && node_pickable(i)) out.push_back(i);
    clear_selection();
    m_sel = out;
}

void ModelEditor::select_grow() {
    std::vector<int> out = m_sel;
    for (const edit::Beam& b : m_model.beams) {
        if (is_selected(b.a) && node_pickable(b.b)) sorted_insert(out, b.b);
        if (is_selected(b.b) && node_pickable(b.a)) sorted_insert(out, b.a);
    }
    m_sel = out;
}

void ModelEditor::select_by_preset(int group) {
    clear_selection();
    for (int i = 0; i < (int)m_model.beams.size(); i++)
        if (m_model.beams[i].group == group && elem_pickable(Elem::Beam, i)) m_sel_beams.push_back(i);
    m_status = std::to_string(m_sel_beams.size()) + " beams of " + m_model.groups[group].name;
}

int ModelEditor::sel_count(SelKind k) const {
    switch (k) {
    case SelKind::Nodes: return (int)m_sel.size();
    case SelKind::Beams: return (int)m_sel_beams.size();
    case SelKind::Shells:
    case SelKind::Cab:
    case SelKind::Fem: {
        int n = 0;
        for (int i : m_sel_tris) n += tri_sel_kind(m_model.tris[i]) == k;
        return n;
    }
    case SelKind::Shocks: return (int)m_sel_shocks.size();
    case SelKind::Rods: return (int)m_sel_hydros.size();
    case SelKind::Wheels: return (int)m_sel_wheels.size();
    case SelKind::Joints: return (int)m_sel_joints.size();
    default: return 0;
    }
}

const char* ModelEditor::sel_kind_name(SelKind k, bool many) {
    static const char* one[] = {"node", "beam", "shell", "cab triangle", "shock", "rod", "wheel", "joint", "FEM triangle"};
    static const char* more[] = {"nodes", "beams", "shells", "cab triangles", "shocks", "rods", "wheels", "joints", "FEM triangles"};
    const int i = std::clamp((int)k, 0, (int)SelKind::Count - 1);
    return many ? more[i] : one[i];
}

void ModelEditor::select_filter(SelKind k, bool only) {
    if (sel_count(k) == 0 && !only) return;
    auto keep = [&](SelKind x) { return only ? x == k : x != k; };
    if (!keep(SelKind::Nodes)) m_sel.clear();
    if (!keep(SelKind::Beams)) m_sel_beams.clear();
    if (!keep(SelKind::Shocks)) m_sel_shocks.clear();
    if (!keep(SelKind::Rods)) m_sel_hydros.clear();
    if (!keep(SelKind::Wheels)) m_sel_wheels.clear();
    if (!keep(SelKind::Joints)) m_sel_joints.clear();
    m_sel_tris.erase(std::remove_if(m_sel_tris.begin(), m_sel_tris.end(),
                                    [&](int i) { return !keep(tri_sel_kind(m_model.tris[i])); }),
                     m_sel_tris.end());
    m_chain = m_chain_start = -1;
    m_picks.clear();
    const int n = sel_count(k);
    m_status = only ? std::to_string(n) + " " + sel_kind_name(k, n != 1) + " kept, the rest deselected"
                    : std::string("The ") + sel_kind_name(k, true) + " deselected";
}

void ModelEditor::hide_selection() {
    m_hidden.resize(m_model.nodes.size(), 0);
    int n = 0;
    for (int i : selection_with_twins()) m_hidden[i] = 1, n++;
    clear_selection();
    m_status = std::to_string(n) + " nodes hidden (Ctrl+Shift+H shows all)";
}

void ModelEditor::unhide_all() {
    m_hidden.assign(m_model.nodes.size(), 0);
    m_status = "All nodes shown";
}

void ModelEditor::fill_selection() {
    // 3 nodes: a triangle; 4: two triangles round the ring, the nodes ordered by angle about the centre
    if (m_sel.size() != 3 && m_sel.size() != 4) {
        m_status = "Fill needs 3 or 4 selected nodes";
        return;
    }
    push_undo();
    std::vector<int> ids = m_sel;
    const int kind = face_kind();
    if (ids.size() == 4) {
        vec3 c(0);
        for (int i : ids) c += m_model.nodes[i].p;
        c = c * 0.25f;
        const vec3 n = normalize_or(cross(m_model.nodes[ids[1]].p - m_model.nodes[ids[0]].p, m_model.nodes[ids[2]].p - m_model.nodes[ids[0]].p), vec3(0, 1, 0));
        const vec3 u = normalize_or(m_model.nodes[ids[0]].p - c, vec3(1, 0, 0)), v = cross(n, u);
        std::sort(ids.begin(), ids.end(), [&](int a, int b) {
            const vec3 da = m_model.nodes[a].p - c, db = m_model.nodes[b].p - c;
            return std::atan2(dot(da, v), dot(da, u)) < std::atan2(dot(db, v), dot(db, u));
        });
        add_tri_sym(ids[0], ids[1], ids[2], kind);
        add_tri_sym(ids[0], ids[2], ids[3], kind);
    } else {
        add_tri_sym(ids[0], ids[1], ids[2], kind);
    }
    m_status = "Filled: check the outside (the white line) and flip if needed";
}

int ModelEditor::beam_joint(int beam, int end) const {
    const edit::Model& M = m_model;
    if (beam < 0 || beam >= (int)M.beams.size()) return 0;
    const edit::Beam& b = M.beams[beam];
    const edit::BeamGroup& g = M.groups[std::clamp(b.group, 0, (int)M.groups.size() - 1)];
    const int own = end ? b.end_b : b.end_a;
    return own >= 0 ? own : end ? g.frame_end_b : g.frame_end_a;
}

void ModelEditor::set_beam_joint(int beam, int end, int type) {
    edit::Model& M = m_model;
    if (beam < 0 || beam >= (int)M.beams.size()) return;
    auto set = [&](int bi, int e) {
        edit::Beam& b = M.beams[bi];
        const edit::BeamGroup& g = M.groups[std::clamp(b.group, 0, (int)M.groups.size() - 1)];
        if (!g.is_frame()) return;
        const int preset = e ? g.frame_end_b : g.frame_end_a;
        (e ? b.end_b : b.end_a) = type < 0 || type == preset ? -1 : type;
    };
    set(beam, end);
    if (!m_symmetry) return;
    // the twin beam's end at the mirrored node
    const edit::Beam b = M.beams[beam];
    const int ta = twin_or_self(b.a), tb = twin_or_self(b.b), tn = end ? tb : ta;
    if (ta < 0 || tb < 0) return;
    for (int i = 0; i < (int)M.beams.size(); i++) {
        if (i == beam) continue;
        const edit::Beam& t = M.beams[i];
        if ((t.a == ta && t.b == tb) || (t.a == tb && t.b == ta)) set(i, t.b == tn ? 1 : 0);
    }
}

void ModelEditor::divide_selected_beams(int n) {
    edit::Model& M = m_model;
    n = std::clamp(n, 2, 16);
    std::vector<int> list = m_sel_beams;
    if (m_symmetry)
        for (int bi : m_sel_beams) {
            const int ta = twin_or_self(M.beams[bi].a), tb = twin_or_self(M.beams[bi].b);
            for (int i = 0; i < (int)M.beams.size() && ta >= 0 && tb >= 0; i++)
                if ((M.beams[i].a == ta && M.beams[i].b == tb) || (M.beams[i].a == tb && M.beams[i].b == ta)) list.push_back(i);
        }
    std::sort(list.begin(), list.end(), std::greater<int>());
    list.erase(std::unique(list.begin(), list.end()), list.end());
    if (list.empty()) {
        m_status = "Select the beams to divide";
        return;
    }
    push_undo();
    int made = 0;
    for (int bi : list) {
        const edit::Beam b = M.beams[bi];
        const vec3 pa = M.nodes[b.a].p, pb = M.nodes[b.b].p;
        int prev = b.a;
        for (int k = 1; k <= n; k++) {
            int cur = b.b;
            if (k < n) {
                cur = add_node(pa + (pb - pa) * ((float)k / n), false);
                M.nodes[cur].layer = M.nodes[b.a].layer;
            }
            edit::Beam x = b;
            x.a = prev, x.b = cur;
            x.end_a = k == 1 ? b.end_a : -1; // (the joints stay at the ends; inside, the pieces are welded)
            x.end_b = k == n ? b.end_b : -1;
            M.beams.push_back(x);
            prev = cur;
        }
        M.beams.erase(M.beams.begin() + bi);
        made++;
    }
    clear_selection();
    m_status = std::to_string(made) + " beams divided into " + std::to_string(n) + " each";
}

void ModelEditor::finish_merge(const std::vector<std::vector<int>>& groups, const std::vector<vec3>& at, const char* what) {
    // the survivors (the first of each group) selected afterwards, at their new numbers
    std::vector<int> keep;
    std::vector<char> out(m_model.nodes.size(), 0);
    for (const auto& g : groups) {
        if (g.size() < 2) continue;
        keep.push_back(g[0]);
        for (size_t k = 1; k < g.size(); k++) out[g[k]] = 1;
    }
    if (keep.empty()) {
        m_status = "Nothing to merge";
        return;
    }
    push_undo();
    const int gone = m_model.merge_nodes(groups, at);
    std::vector<int> shift(out.size() + 1, 0);
    for (size_t i = 0; i < out.size(); i++) shift[i + 1] = shift[i] + out[i];
    clear_selection();
    for (int k : keep) sorted_insert(m_sel, k - shift[k]);
    m_hidden.assign(m_model.nodes.size(), 0);
    m_status = std::to_string(gone) + " nodes merged " + what;
}

void ModelEditor::merge_selection() {
    std::vector<int> S;
    for (int n : m_sel)
        if (!m_model.node_locked(n)) S.push_back(n);
    if (S.size() < 2) {
        m_status = "Select two nodes or more to merge";
        return;
    }
    // at their centre (a fixed node stays where it is)
    auto place = [&](const std::vector<int>& g) {
        vec3 c(0);
        for (int n : g) c += m_model.nodes[n].p;
        c = c * (1.0f / (float)g.size());
        for (int n : g)
            if (m_model.nodes[n].fixed) return m_model.nodes[n].p;
        return c;
    };
    std::vector<std::vector<int>> groups{S};
    std::vector<vec3> at{place(S)};
    if (m_symmetry) {
        // their twins into one on the other side (unless the selection holds both sides already)
        std::vector<int> T;
        bool both = false;
        for (int n : S) {
            const int t = m_model.twin(n);
            if (t < 0) continue;
            if (std::binary_search(S.begin(), S.end(), t)) both = true;
            else if (!m_model.node_locked(t)) T.push_back(t);
        }
        std::sort(T.begin(), T.end());
        T.erase(std::unique(T.begin(), T.end()), T.end());
        if (!both && T.size() >= 2) groups.push_back(T), at.push_back(mirror_z(at[0]));
    }
    finish_merge(groups, at, "(the selection into one)");
}

void ModelEditor::merge_by_distance(float tol) {
    // the nodes (the selection's, or all) closer than tol to one another: each cluster into one at its centre
    std::vector<int> ids;
    if (m_sel.size() >= 2) {
        for (int n : m_sel)
            if (!m_model.node_locked(n)) ids.push_back(n);
    } else {
        for (int n = 0; n < (int)m_model.nodes.size(); n++)
            if (node_pickable(n)) ids.push_back(n);
    }
    const int k = (int)ids.size();
    std::vector<int> root(k);
    for (int i = 0; i < k; i++) root[i] = i;
    std::function<int(int)> find = [&](int i) { return root[i] == i ? i : root[i] = find(root[i]); };
    const float t2 = tol * tol;
    for (int i = 0; i < k; i++)
        for (int j = i + 1; j < k; j++)
            if (length2(m_model.nodes[ids[i]].p - m_model.nodes[ids[j]].p) <= t2) root[find(j)] = find(i);
    std::map<int, std::vector<int>> clusters;
    for (int i = 0; i < k; i++) clusters[find(i)].push_back(ids[i]);
    std::vector<std::vector<int>> groups;
    std::vector<vec3> at;
    for (auto& [r, g] : clusters) {
        if (g.size() < 2) continue;
        std::sort(g.begin(), g.end());
        vec3 c(0);
        bool fixed = false;
        for (int n : g) c += m_model.nodes[n].p;
        c = c * (1.0f / (float)g.size());
        for (int n : g)
            if (m_model.nodes[n].fixed && !fixed) c = m_model.nodes[n].p, fixed = true;
        groups.push_back(g);
        at.push_back(c);
    }
    char what[64];
    snprintf(what, sizeof what, "(closer than %.1f mm)", tol * 1000.0f);
    if (groups.empty()) m_status = std::string("No nodes closer than ") + format("%.1f mm", tol * 1000.0f);
    else finish_merge(groups, at, what);
}

void ModelEditor::merge_into(int src, int dst) {
    // src goes into dst (at dst); with symmetry src's twin into dst's twin
    if (src == dst || src < 0 || dst < 0) return;
    std::vector<std::vector<int>> groups{{dst, src}};
    std::vector<vec3> at{m_model.nodes[dst].p};
    if (m_symmetry) {
        const int ts = m_model.twin(src), td = m_model.twin(dst);
        if (ts >= 0 && td >= 0 && ts != td && ts != dst && td != src && ts != src && td != dst) groups.push_back({td, ts}), at.push_back(m_model.nodes[td].p);
    }
    finish_merge(groups, at, "(into the second)");
}

void ModelEditor::set_view(int preset) {
    // one view: that one; four: that one blown up (again: back to four)
    if (m_quad) m_maximized = m_maximized == preset ? -1 : preset;
    else m_single = preset;
    m_active_view = preset;
}

// ------------------------------------------------------------------------------------------------ edits
int ModelEditor::add_node(vec3 p, bool twin) {
    const int near = m_model.nearest_node(p, 1e-3f);
    if (near >= 0) return near;
    edit::Node n;
    n.p = p;
    n.load_bearing = true;
    n.layer = m_layer;
    m_model.nodes.push_back(n);
    m_hidden.push_back(0);
    const int id = (int)m_model.nodes.size() - 1;
    if (twin && m_symmetry && std::fabs(p.z) > 1e-3f && m_model.nearest_node(mirror_z(p), 1e-3f) < 0) {
        n.p = mirror_z(p);
        m_model.nodes.push_back(n);
        m_hidden.push_back(0);
    }
    return id;
}

int ModelEditor::node_from_pick(const Pick& pk) {
    if (pk.kind == PK_Node && pk.node >= 0) return pk.node;
    if ((pk.kind == PK_Mid || pk.kind == PK_Edge) && pk.beam >= 0 && pk.beam < (int)m_model.beams.size()) {
        // a point on a beam: the beam is split there (and its mirror twin with symmetry)
        const edit::Beam b = m_model.beams[pk.beam];
        const int m = add_node(pk.p, false);
        m_model.remove_beam(pk.beam);
        m_model.add_beam(b.a, m, b.group), m_model.beams.back().layer = b.layer;
        m_model.add_beam(m, b.b, b.group), m_model.beams.back().layer = b.layer;
        if (m_symmetry) {
            const int ta = twin_or_self(b.a), tb = twin_or_self(b.b);
            const int tb_beam = ta >= 0 && tb >= 0 && !(ta == b.a && tb == b.b) ? m_model.find_beam(ta, tb) : -1;
            if (tb_beam >= 0) {
                const edit::Beam t = m_model.beams[tb_beam];
                const int tm = add_node(mirror_z(pk.p), false);
                m_model.remove_beam(tb_beam);
                m_model.add_beam(t.a, tm, t.group), m_model.beams.back().layer = t.layer;
                m_model.add_beam(tm, t.b, t.group), m_model.beams.back().layer = t.layer;
            }
        }
        return m;
    }
    return add_node(pk.p, true);
}

void ModelEditor::add_beam_sym(int a, int b) {
    if (m_model.add_beam(a, b, m_group) >= 0) m_model.beams.back().layer = m_layer;
    if (!m_symmetry) return;
    const int x = twin_or_self(a), y = twin_or_self(b);
    if (x >= 0 && y >= 0 && m_model.add_beam(x, y, m_group) >= 0) m_model.beams.back().layer = m_layer;
}

void ModelEditor::add_tri_sym(int a, int b, int c, int kind) {
    if (a == b || b == c || a == c) return;
    auto exists = [&](int x, int y, int z) {
        std::array<int, 3> k{x, y, z};
        std::sort(k.begin(), k.end());
        for (const edit::Tri& t : m_model.tris) {
            std::array<int, 3> q{t.a, t.b, t.c};
            std::sort(q.begin(), q.end());
            if (q == k) return true;
        }
        return false;
    };
    edit::Tri t;
    t.a = a, t.b = b, t.c = c, t.shell = kind == 1, t.layer = m_layer;
    t.shell_preset = kind == 1 ? m_shell_preset : 0;
    if (kind == 2) { // (a FEM triangle makes its own collision surface)
        m_model.ensure_fem_preset();
        t.fem = true, t.collision = false;
        t.fem_preset = std::clamp(m_fem_preset, 0, (int)m_model.fem_presets.size() - 1);
    }
    if (!exists(a, b, c)) m_model.tris.push_back(t);
    if (!m_symmetry) return;
    const int x = twin_or_self(a), y = twin_or_self(b), z = twin_or_self(c);
    if (x >= 0 && y >= 0 && z >= 0 && !(x == a && y == b && z == c) && !exists(x, y, z)) {
        t.a = x, t.b = z, t.c = y; // (mirrored winding)
        m_model.tris.push_back(t);
    }
}

void ModelEditor::triangulate_selection(int mode) {
    if (m_sel.size() < 3) {
        m_status = "Triangulate needs at least 3 selected nodes";
        return;
    }
    const std::vector<std::array<int, 3>> tris = edit::triangulate(m_model, m_sel, m_model.centroid());
    if (tris.empty()) return;
    push_undo();
    if (mode == 0) {
        const size_t nb = m_model.beams.size();
        for (const auto& t : tris)
            for (int e = 0; e < 3; e++) add_beam_sym(t[e], t[(e + 1) % 3]);
        m_status = std::to_string(m_model.beams.size() - nb) + " beams laid over the selection";
    } else {
        const size_t nt = m_model.tris.size();
        for (const auto& t : tris) add_tri_sym(t[0], t[1], t[2], mode - 1);
        m_status = std::to_string(m_model.tris.size() - nt) + (mode == 3 ? " FEM" : mode == 2 ? " shell" : " cab") + " triangles laid over the selection";
    }
}

void ModelEditor::hull_selection() {
    if (m_sel.size() < 4) {
        m_status = "A collision hull needs at least 4 selected nodes (not on one plane)";
        return;
    }
    std::vector<vec3> pts;
    for (int i : m_sel) pts.push_back(m_model.nodes[i].p);
    const auto faces = convex_hull(pts);
    if (faces.empty()) {
        m_status = "The selected nodes lie on one plane: no hull";
        return;
    }
    push_undo();
    std::set<std::array<int, 3>> have;
    for (const edit::Tri& t : m_model.tris) {
        std::array<int, 3> k{t.a, t.b, t.c};
        std::sort(k.begin(), k.end());
        have.insert(k);
    }
    int added = 0;
    for (const auto& f : faces) {
        edit::Tri t;
        t.a = m_sel[f[0]], t.b = m_sel[f[1]], t.c = m_sel[f[2]]; // (counter-clockwise from outside: facing out)
        std::array<int, 3> k{t.a, t.b, t.c};
        std::sort(k.begin(), k.end());
        if (!have.insert(k).second) continue;
        t.collision = true, t.shell = false, t.options = "ch", t.layer = m_layer;
        m_model.tris.push_back(t);
        added++;
    }
    m_status = std::to_string(added) + " hull triangles round the selection (one-sided collision, not drawn in the game)";
}

void ModelEditor::move_selection(vec3 delta) {
    for (int n : m_sel) m_model.nodes[n].p += delta;
    if (m_symmetry) {
        const vec3 md = mirror_z(delta);
        for (int n : m_sel) {
            const int t = m_model.twin(n);
            if (t >= 0 && !is_selected(t) && !m_model.node_locked(t)) m_model.nodes[t].p += md;
        }
    }
}

void ModelEditor::op_begin_nodes() {
    m_op_nodes.clear(), m_op_twins.clear(), m_op_pos0.clear();
    for (int n : selection_nodes_all()) {
        if (m_model.node_locked(n)) continue;
        m_op_nodes.push_back(n);
        m_op_pos0.push_back(m_model.nodes[n].p);
    }
    for (int n : m_op_nodes) {
        int t = m_symmetry ? m_model.twin(n) : -1;
        if (t >= 0 && (std::binary_search(m_op_nodes.begin(), m_op_nodes.end(), t) || m_model.node_locked(t))) t = -1;
        m_op_twins.push_back(t);
    }
}

void ModelEditor::op_apply(const std::function<vec3(vec3)>& f) {
    for (size_t i = 0; i < m_op_nodes.size(); i++) {
        const vec3 p = f(m_op_pos0[i]);
        m_model.nodes[m_op_nodes[i]].p = p;
        if (m_op_twins[i] >= 0) m_model.nodes[m_op_twins[i]].p = mirror_z(p);
    }
}

void ModelEditor::op_restore() {
    for (size_t i = 0; i < m_op_nodes.size(); i++) {
        m_model.nodes[m_op_nodes[i]].p = m_op_pos0[i];
        if (m_op_twins[i] >= 0) m_model.nodes[m_op_twins[i]].p = mirror_z(m_op_pos0[i]);
    }
}

// ------------------------------------------------------------------------------------------------ building tools
vec3 ModelEditor::plane_normal(int view) const {
    const int o = m_views[view].ortho;
    if (o == 1) return vec3(1, 0, 0);
    if (o == 2) return vec3(0, 0, 1);
    if (o == 3) return vec3(0, 1, 0);
    return m_plane_mode == 1 ? vec3(0, 0, 1) : m_plane_mode == 2 ? vec3(1, 0, 0) : vec3(0, 1, 0);
}

void ModelEditor::make_rect(vec3 a, vec3 b, int view) {
    const vec3 n = plane_normal(view);
    vec3 u, v;
    plane_axes(n, u, v);
    const float du = dot(b - a, u), dv = dot(b - a, v);
    if (std::fabs(du) < 1e-3f || std::fabs(dv) < 1e-3f) return;
    push_undo();
    const int nx = std::max(1, m_rect_div[0]), ny = std::max(1, m_rect_div[1]);
    std::vector<int> id((size_t)(nx + 1) * (ny + 1));
    for (int j = 0; j <= ny; j++)
        for (int i = 0; i <= nx; i++) id[(size_t)j * (nx + 1) + i] = add_node(a + u * (du * i / nx) + v * (dv * j / ny), true);
    auto at = [&](int i, int j) { return id[(size_t)j * (nx + 1) + i]; };
    // shell faces hold their shape themselves (in the plane and in bending): no beams unless asked
    const bool beams = m_face_mode < 2 || m_shell_beams;
    // a rectangle across the symmetry plane is its own mirror: its faces and braces are not mirrored again
    const bool keep_sym = m_symmetry;
    bool self_sym = m_symmetry;
    for (int n : id) self_sym &= std::find(id.begin(), id.end(), twin_or_self(n)) != id.end();
    if (self_sym) m_symmetry = false;
    // faces toward the camera of the view it was drawn in
    const vec3 to_cam = m_views[view].cam.pos - to_world((a + b) * 0.5f);
    const bool flip = dot(cross(u * du, v * dv), to_cam) < 0;
    for (int j = 0; j <= ny; j++)
        for (int i = 0; i <= nx; i++) {
            if (beams && i < nx) add_beam_sym(at(i, j), at(i + 1, j));
            if (beams && j < ny) add_beam_sym(at(i, j), at(i, j + 1));
            if (i < nx && j < ny) {
                if (beams && m_rect_diagonals) add_beam_sym(at(i, j), at(i + 1, j + 1));
                if (m_face_mode) {
                    int p0 = at(i, j), p1 = at(i + 1, j), p2 = at(i + 1, j + 1), p3 = at(i, j + 1);
                    if (flip) std::swap(p1, p3);
                    add_tri_sym(p0, p1, p2, face_kind());
                    add_tri_sym(p0, p2, p3, face_kind());
                }
            }
        }
    m_symmetry = keep_sym;
    m_status = "Rectangle " + std::to_string(std::fabs(du)).substr(0, 5) + " x " + std::to_string(std::fabs(dv)).substr(0, 5) + " m";
}

void ModelEditor::make_circle(vec3 c, vec3 r, int view) {
    const vec3 n = plane_normal(view);
    vec3 d = r - c;
    d -= n * dot(d, n);
    const float rad = length(d);
    if (rad < 1e-3f) return;
    push_undo();
    const vec3 e1 = d / rad, e2 = cross(n, e1);
    const int sides = std::clamp(m_circle_sides, 3, 64);
    std::vector<int> ring;
    for (int k = 0; k < sides; k++) {
        const float a = 2.0f * kPi * k / sides;
        ring.push_back(add_node(c + (e1 * std::cos(a) + e2 * std::sin(a)) * rad, true));
    }
    const bool center = m_circle_center || m_face_mode;
    const int ci = center ? add_node(c, true) : -1;
    const bool beams = m_face_mode < 2 || m_shell_beams; // (shell and FEM faces: no beams unless asked)
    const bool keep_sym = m_symmetry;
    bool self_sym = m_symmetry;
    for (int n : ring) self_sym &= std::find(ring.begin(), ring.end(), twin_or_self(n)) != ring.end();
    if (self_sym) m_symmetry = false;
    const vec3 to_cam = m_views[view].cam.pos - to_world(c);
    const bool flip = dot(n, to_cam) < 0;
    for (int k = 0; k < sides; k++) {
        const int a = ring[k], b = ring[(k + 1) % sides];
        if (beams) add_beam_sym(a, b);
        if (beams && ci >= 0) add_beam_sym(ci, a);
        if (m_face_mode && ci >= 0) {
            if (flip) add_tri_sym(ci, b, a, face_kind());
            else add_tri_sym(ci, a, b, face_kind());
        }
    }
    if (ci < 0 && beams) // (a ring alone needs bracing: chords across)
        for (int k = 0; k < sides; k++) add_beam_sym(ring[k], ring[(k + 2) % sides]);
 m_symmetry = keep_sym;
    m_status = "Circle r " + std::to_string(rad).substr(0, 5) + " m, " + std::to_string(sides) + " sides";
}

void ModelEditor::pushpull_region(int tri, std::vector<int>& region, vec3& n) const {
    region.clear();
    const edit::Model& M = m_model;
    if (tri < 0 || tri >= (int)M.tris.size()) return;
    auto normal = [&](int i) {
        const edit::Tri& t = M.tris[i];
        return normalize_or(cross(M.nodes[t.b].p - M.nodes[t.a].p, M.nodes[t.c].p - M.nodes[t.a].p), vec3(0, 1, 0));
    };
    n = normal(tri);
    const vec3 p0 = M.nodes[M.tris[tri].a].p;
    std::map<std::pair<int, int>, std::vector<int>> edges;
    for (int i = 0; i < (int)M.tris.size(); i++) {
        if (!elem_shown(Elem::Tri, i)) continue;
        const edit::Tri& t = M.tris[i];
        const int v[3] = {t.a, t.b, t.c};
        for (int e = 0; e < 3; e++) edges[{std::min(v[e], v[(e + 1) % 3]), std::max(v[e], v[(e + 1) % 3])}].push_back(i);
    }
    std::vector<char> in(M.tris.size(), 0);
    std::vector<int> stack{tri};
    in[tri] = 1;
    while (!stack.empty()) {
        const int i = stack.back();
        stack.pop_back();
        region.push_back(i);
        const edit::Tri& t = M.tris[i];
        const int v[3] = {t.a, t.b, t.c};
        for (int e = 0; e < 3; e++)
            for (int j : edges[{std::min(v[e], v[(e + 1) % 3]), std::max(v[e], v[(e + 1) % 3])}]) {
                if (in[j]) continue;
                const edit::Tri& u = M.tris[j];
                // coplanar and facing the same way
                if (dot(normal(j), n) < 0.9998f) continue;
                if (std::fabs(dot(M.nodes[u.a].p - p0, n)) > 2e-3f || std::fabs(dot(M.nodes[u.b].p - p0, n)) > 2e-3f || std::fabs(dot(M.nodes[u.c].p - p0, n)) > 2e-3f) continue;
                in[j] = 1;
                stack.push_back(j);
            }
    }
    std::sort(region.begin(), region.end());
}

void ModelEditor::pushpull_apply(float d) {
    if (m_pp_region.empty() || std::fabs(d) < 1e-4f) return;
    edit::Model& M = m_model;
    // the face's nodes, its edges, and which edges are on its border (in one of its triangles only)
    std::set<int> nodes;
    std::map<std::pair<int, int>, int> edge_count;
    std::map<std::pair<int, int>, std::pair<int, int>> edge_dir; // (as wound in its triangle)
    for (int i : m_pp_region) {
        const edit::Tri& t = M.tris[i];
        const int v[3] = {t.a, t.b, t.c};
        for (int e = 0; e < 3; e++) {
            nodes.insert(v[e]);
            const auto key = std::make_pair(std::min(v[e], v[(e + 1) % 3]), std::max(v[e], v[(e + 1) % 3]));
            edge_count[key]++;
            edge_dir[key] = {v[e], v[(e + 1) % 3]};
        }
    }
    // is the face free (its border shared with no other triangle)? then a base face closes the solid
    bool free_face = true;
    for (const auto& [key, c] : edge_count) {
        if (c != 1) continue;
        for (int i = 0; i < (int)M.tris.size(); i++) {
            if (std::binary_search(m_pp_region.begin(), m_pp_region.end(), i)) continue;
            const edit::Tri& t = M.tris[i];
            const int v[3] = {t.a, t.b, t.c};
            for (int e = 0; e < 3; e++)
                if (std::make_pair(std::min(v[e], v[(e + 1) % 3]), std::max(v[e], v[(e + 1) % 3])) == key) free_face = false;
        }
    }
    push_undo();
    const vec3 off = m_pp_n * d;
    const edit::Tri& face = M.tris[m_pp_region[0]];
    const bool shell = face.shell, fem = face.fem, coll = face.collision;
    const int shell_preset = face.shell_preset, fem_preset = face.fem_preset;
    // a shell (or FEM) face makes a box of shells, which holds its shape itself: no beams unless asked (a cab face
    // needs them)
    const bool beams = !(shell || fem) || m_shell_beams;
    auto beam = [&](int a, int b) {
        if (beams && M.add_beam(a, b, m_group) >= 0) M.beams.back().layer = m_layer;
    };
    std::map<int, int> top;
    for (int n : nodes) {
        edit::Node x = M.nodes[n];
        x.p += off;
        x.layer = m_layer;
        M.nodes.push_back(x);
        m_hidden.push_back(0);
        top[n] = (int)M.nodes.size() - 1;
        beam(n, top[n]);
    }
    vec3 center(0);
    for (int n : nodes) center += M.nodes[n].p;
    center = center * (1.0f / (float)nodes.size()) + off * 0.5f;
    for (const auto& [key, c] : edge_count) {
        beam(top[key.first], top[key.second]); // the cap's edges
        if (c != 1) continue;
        // a side: a quad between the border edge and its copy, braced across, two triangles facing out
        const int a = edge_dir[key].first, b = edge_dir[key].second;
        beam(a, top[b]);
        edit::Tri t1, t2;
        t1.a = a, t1.b = b, t1.c = top[b];
        t2.a = a, t2.b = top[b], t2.c = top[a];
        const vec3 nn = cross(M.nodes[t1.b].p - M.nodes[t1.a].p, M.nodes[t1.c].p - M.nodes[t1.a].p);
        const vec3 mid = (M.nodes[a].p + M.nodes[top[b]].p) * 0.5f;
        if (dot(nn, mid - center) < 0) std::swap(t1.b, t1.c), std::swap(t2.b, t2.c);
        t1.shell = t2.shell = shell, t1.layer = t2.layer = m_layer;
        t1.shell_preset = t2.shell_preset = shell_preset;
        t1.fem = t2.fem = fem, t1.fem_preset = t2.fem_preset = fem_preset, t1.collision = t2.collision = coll;
        M.tris.push_back(t1), M.tris.push_back(t2);
    }
    // the cap: the face's triangles move up to the copies (facing out); a free face leaves a base behind
    std::vector<edit::Tri> base;
    for (int i : m_pp_region) {
        edit::Tri& t = M.tris[i];
        if (free_face) {
            edit::Tri b = t;
            std::swap(b.b, b.c); // (the base faces the other way)
            b.submesh = -1, b.options.clear();
            base.push_back(b);
        }
        t.a = top[t.a], t.b = top[t.b], t.c = top[t.c];
        if (d < 0) std::swap(t.b, t.c); // (pushed in: the cap faces the other way)
        t.submesh = -1, t.options.clear();
    }
    if (d < 0)
        for (edit::Tri& b : base) std::swap(b.b, b.c);
    for (const edit::Tri& b : base) M.tris.push_back(b);
    m_status = "Pushed / pulled " + std::to_string(std::fabs(d)).substr(0, 5) + " m";
}

// ------------------------------------------------------------------------------------------------ the reference mesh
mat4 ModelEditor::ref_matrix() const {
    const float cy = std::cos(m_model.ref_yaw * kDeg2Rad), sy = std::sin(m_model.ref_yaw * kDeg2Rad), s = m_model.ref_scale;
    return mat4(vec4(cy * s, 0, -sy * s, 0), vec4(0, s, 0, 0), vec4(sy * s, 0, cy * s, 0), vec4(to_world(m_model.ref_offset), 1));
}

void ModelEditor::ref_load(const std::string& path) {
    std::string p = path;
    if (!file_exists(p) && file_exists(asset_path("reference/" + path))) p = asset_path("reference/" + path);
    if (!edit::load_obj(p, m_ref_v, m_ref_idx)) {
        toast("Cannot read " + path + " (a Wavefront .obj with faces)");
        m_ref_v.clear(), m_ref_idx.clear(), m_ref_loaded.clear();
        return;
    }
    m_ref_loaded = m_model.ref_path = path;
    snprintf(m_ref_buf, sizeof m_ref_buf, "%s", path.c_str());
    ref_update_mesh();
    m_status = path_stem(path) + ": " + std::to_string(m_ref_idx.size() / 3) + " triangles";
}

void ModelEditor::ref_update_mesh() {
    std::vector<Vertex> vs;
    std::vector<uint32_t> idx;
    vs.reserve(m_ref_idx.size());
    for (size_t i = 0; i + 2 < m_ref_idx.size(); i += 3) {
        const vec3 a = m_ref_v[m_ref_idx[i]], b = m_ref_v[m_ref_idx[i + 1]], c = m_ref_v[m_ref_idx[i + 2]];
        const vec3 n = normalize_or(cross(b - a, c - a), vec3(0, 1, 0));
        for (vec3 p : {a, b, c}) {
            idx.push_back((uint32_t)vs.size());
            vs.push_back({p, n, vec2(0, 0)});
        }
    }
    if (!vs.empty()) m_ref_mesh.create(vs, idx, false);
}

bool ModelEditor::ref_hit(vec3 ro, vec3 rd, vec3& hit) const {
    if (m_ref_idx.empty()) return false;
    const mat4 M = ref_matrix();
    float best = 1e30f;
    for (size_t i = 0; i + 2 < m_ref_idx.size(); i += 3) {
        float t;
        if (ray_tri(ro, rd, M.transform_point(m_ref_v[m_ref_idx[i]]), M.transform_point(m_ref_v[m_ref_idx[i + 1]]), M.transform_point(m_ref_v[m_ref_idx[i + 2]]), t) && t < best) best = t;
    }
    if (best >= 1e30f) return false;
    hit = ro + rd * best;
    return true;
}

vec3 ModelEditor::ref_closest(vec3 p) const {
    // the nearest point on the mesh (Ericson's closest point on a triangle)
    const mat4 M = ref_matrix();
    vec3 best = p;
    float bd = 1e30f;
    for (size_t i = 0; i + 2 < m_ref_idx.size(); i += 3) {
        const vec3 a = M.transform_point(m_ref_v[m_ref_idx[i]]), b = M.transform_point(m_ref_v[m_ref_idx[i + 1]]), c = M.transform_point(m_ref_v[m_ref_idx[i + 2]]);
        const vec3 ab = b - a, ac = c - a, ap = p - a;
        const float d1 = dot(ab, ap), d2 = dot(ac, ap);
        vec3 q;
        if (d1 <= 0 && d2 <= 0) q = a;
        else {
            const vec3 bp = p - b;
            const float d3 = dot(ab, bp), d4 = dot(ac, bp);
            if (d3 >= 0 && d4 <= d3) q = b;
            else {
                const float vc = d1 * d4 - d3 * d2;
                if (vc <= 0 && d1 >= 0 && d3 <= 0) q = a + ab * (d1 / (d1 - d3));
                else {
                    const vec3 cp = p - c;
                    const float d5 = dot(ab, cp), d6 = dot(ac, cp);
                    if (d6 >= 0 && d5 <= d6) q = c;
                    else {
                        const float vb = d5 * d2 - d1 * d6;
                        if (vb <= 0 && d2 >= 0 && d6 <= 0) q = a + ac * (d2 / (d2 - d6));
                        else {
                            const float va = d3 * d6 - d5 * d4;
                            if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) q = b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
                            else {
                                const float den = 1.0f / (va + vb + vc);
                                q = a + ab * (vb * den) + ac * (vc * den);
                            }
                        }
                    }
                }
            }
        }
        const float d = length2(q - p);
        if (d < bd) bd = d, best = q;
    }
    return best;
}

void ModelEditor::snap_to_surface() {
    if (m_ref_idx.empty() || m_sel.empty()) return;
    push_undo();
    for (int n : selection_with_twins()) m_model.nodes[n].p = to_model(ref_closest(to_world(m_model.nodes[n].p)));
    m_status = "Selected nodes moved onto the reference surface";
}

} // namespace bl
