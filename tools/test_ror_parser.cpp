// Parses every RoR vehicle definition under assets/vehicles, prints statistics and validates the result
// (node references, generated wheel node ranges, and the known quirks listed in ror_format_spec.md section 6).
#include "core/util.h"
#include "game/editor_model.h"
#include "vehicle/ror_def.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <functional>
#include <string>
#include <vector>

using namespace bl;
using namespace bl::ror;

namespace {

int g_failures = 0;

#define CHECK(cond, ...)                                                    \
    do {                                                                    \
        if (!(cond)) {                                                      \
            g_failures++;                                                   \
            printf("    FAIL (%s:%d) ", __FILE__, __LINE__);                \
            printf(__VA_ARGS__);                                            \
            printf("\n");                                                   \
        }                                                                   \
    } while (0)

// Calls f(ref, allow_none, what) for every node reference in the document.
void for_each_ref(const Document& d, const std::function<void(int, bool, const char*)>& f) {
    for (auto& b : d.beams) f(b.n1, false, "beam"), f(b.n2, false, "beam");
    for (auto& s : d.shocks) f(s.n1, false, "shock"), f(s.n2, false, "shock");
    for (auto& h : d.hydros) f(h.n1, false, "hydro"), f(h.n2, false, "hydro");
    for (auto& c : d.commands) f(c.n1, false, "command"), f(c.n2, false, "command");
    for (auto& c : d.cinecams)
        for (int n : c.nodes) f(n, false, "cinecam");
    for (auto& c : d.cameras) f(c.center, false, "camera"), f(c.back, false, "camera"), f(c.left, false, "camera");
    for (auto& w : d.wheels) f(w.n1, false, "wheel"), f(w.n2, false, "wheel"), f(w.rigidity, true, "wheel rigidity"),
        f(w.arm, false, "wheel arm");
    for (auto& a : d.axles) f(a.w1a, true, "axle"), f(a.w1b, true, "axle"), f(a.w2a, true, "axle"), f(a.w2b, true, "axle");
    for (auto& sm : d.submeshes) {
        for (auto& t : sm.texcoords) f(t.node, false, "texcoord");
        for (auto& c : sm.cabs) f(c.n1, false, "cab"), f(c.n2, false, "cab"), f(c.n3, false, "cab");
    }
    for (int n : d.contacters) f(n, false, "contacter");
    for (auto& fb : d.flexbodies) {
        f(fb.ref, false, "flexbody"), f(fb.x, false, "flexbody"), f(fb.y, false, "flexbody");
        for (int n : fb.forset) f(n, false, "forset");
    }
    for (auto& p : d.props) f(p.ref, false, "prop"), f(p.x, false, "prop"), f(p.y, false, "prop");
    for (auto& r : d.ropes) f(r.root, false, "rope"), f(r.end, false, "rope");
    for (auto& t : d.ties) f(t.root, false, "tie");
    for (int n : d.fixes) f(n, false, "fix");
}

void validate_structure(const Document& d) {
    const int total = (int)d.nodes.size();
    int bad = 0;
    for_each_ref(d, [&](int r, bool allow_none, const char* what) {
        if ((r >= 0 && r < total) || (allow_none && r == -1)) return;
        if (bad++ < 5) CHECK(false, "%s references node %d (total %d)", what, r, total);
    });
    CHECK(bad == 0, "%d invalid node references", bad);

    // slots: explicit/cinecam refs in order, wheel ranges contiguous and matching first_node
    int ne = 0, nc = 0, nw = 0;
    for (int i = 0; i < total; i++) {
        const NodeSlot& s = d.nodes[i];
        if (s.kind == NodeSlot::EXPLICIT) CHECK(s.ref == ne++, "slot %d: explicit ref %d out of order", i, s.ref);
        else if (s.kind == NodeSlot::CINECAM) CHECK(s.ref == nc++, "slot %d: cinecam ref %d out of order", i, s.ref);
        else nw++;
    }
    CHECK(ne == (int)d.nodes_explicit.size(), "explicit slots %d != nodes_explicit %zu", ne, d.nodes_explicit.size());
    CHECK(nc == (int)d.cinecams.size(), "cinecam slots %d != cinecams %zu", nc, d.cinecams.size());
    int expect_first = -1, wheel_nodes = 0, prev_count = 1;
    for (size_t wi = 0; wi < d.wheels.size(); wi++) {
        const WheelDef& w = d.wheels[wi];
        // (a ring tyre has no nodes: the wheel after it starts where it would have)
        CHECK(w.first_node > expect_first || (w.first_node == expect_first && (prev_count == 0 || w.node_count() == 0)), "wheel %zu first_node %d not increasing", wi,
              w.first_node);
        prev_count = w.node_count();
        CHECK(w.first_node >= 0 && w.first_node + w.node_count() <= total, "wheel %zu range out of bounds", wi);
        if (w.first_node < 0 || w.first_node + w.node_count() > total) continue;
        for (int k = 0; k < w.node_count(); k++) {
            const NodeSlot& s = d.nodes[w.first_node + k];
            if (s.kind != NodeSlot::WHEEL || s.ref != (int)wi || s.sub != k) {
                CHECK(false, "wheel %zu slot %d mismatch", wi, w.first_node + k);
                break;
            }
        }
        expect_first = w.first_node;
        wheel_nodes += w.node_count();
    }
    CHECK(wheel_nodes == nw, "wheel slots %d != sum of wheel node counts %d", nw, wheel_nodes);
}

bool has_warning_at(const Document& d, int line) {
    std::string p = format("line %d:", line);
    for (auto& w : d.warnings)
        if (w.compare(0, p.size(), p) == 0) return true;
    return false;
}

const FlexbodyDef* flexbody_at(const Document& d, int line) {
    for (auto& fb : d.flexbodies)
        if (fb.line == line) return &fb;
    return nullptr;
}

bool beams_use_range(const Document& d, int lo, int hi) {
    for (auto& b : d.beams)
        if ((b.n1 >= lo && b.n1 <= hi) || (b.n2 >= lo && b.n2 <= hi)) return true;
    return false;
}

// Known quirks from ror_format_spec.md section 6.
void check_quirks(const std::string& rel, const Document& d) {
    const std::string file = path_filename(rel);
    if (file == "T800_Wrecker.truck") {
        CHECK(!has_warning_at(d, 5), "kenworth line 5 ('aurthor ...') must be ignored silently");
        CHECK(!d.submeshes.empty() && d.submeshes.back().cabs.empty() && d.submeshes.back().texcoords.empty(),
              "kenworth: last submesh should be empty");
    }
    if (file == "tatra815-6x6vvn.truck") {
        CHECK(has_warning_at(d, 1480), "tatra line 1480 (forset without keyword) should warn");
        const FlexbodyDef* fb = flexbody_at(d, 1479);
        CHECK(fb && fb->mesh == "tatra815-rearb.mesh" && !fb->has_forset && fb->forset.empty(),
              "tatra rearb flexbody must have an empty forset");
        int il = 0;
        for (auto& s : d.shocks)
            if (s.line == 1195 || s.line == 1196) il += s.options == "i";
        CHECK(il == 2, "tatra shocks il/ir must keep only 'i'");
        CHECK(has_warning_at(d, 1195) && has_warning_at(d, 1196), "tatra il/ir should warn");
        const FlexbodyDef* tyre = flexbody_at(d, 1488);
        CHECK(tyre && tyre->y == 201 && d.nodes[201].kind == NodeSlot::WHEEL, "tatra flexbody must use wheel node 201");
        CHECK(d.submeshes.size() == 1 && d.submeshes[0].cabs.size() == 32, "tatra: two cab headers in one submesh");
    }
    if (file == "f250_99_extcab.truck") {
        const FlexbodyDef* fb = flexbody_at(d, 2117);
        CHECK(fb && fb->forset == std::vector<int>({67, 71, 30, 6, 0}), "ford forset with trailing comma must add node 0");
    }
    if (file.find("Chevrolet") == 0 || file.find("GMC") == 0) {
        CHECK(!d.axles.empty(), "chevrolet: axles expected");
        for (auto& w : d.warnings) CHECK(w.find("axles") == std::string::npos, "unexpected axle warning: %s", w.c_str());
        int dash = 0;
        for (auto& p : d.props)
            if (p.mesh == "dashboard-small.mesh") {
                dash++;
                CHECK(p.special == PropDef::DASHBOARD && p.wheel_mesh == "dirwheel.mesh" && !p.has_wheel_offset &&
                          p.wheel_angle == 160.0f,
                      "dashboard-small.mesh must be a default dashboard prop");
            }
        (void)dash;
    }
    if (file == "Chevrolet Express_SWB_passenger.truck")
        CHECK(!d.title.empty() && d.title.back() != ' ', "title must be right-trimmed");
    if (file == "E36Lightweight.truck" || file == "E36Sedan.truck") {
        CHECK(d.nodes_explicit.size() == 372 && d.cinecams.size() == 3, "bmw: 372 nodes + 3 cinecams expected");
        CHECK(!d.wheels.empty() && d.wheels[0].first_node == 375 && d.wheels[0].type == WheelDef::MESHWHEELS2,
              "bmw: first meshwheels2 node must be 375");
        bool uses375 = false;
        for (auto& fb : d.flexbodies) uses375 |= fb.y == 375;
        CHECK(uses375, "bmw: a flexbody must reference node 375");
        for (auto& p : d.props)
            if (p.mesh == "E36_STEER.mesh") CHECK(p.special == PropDef::NONE, "E36_STEER.mesh is not special");
        CHECK(d.beams.size() > 0 && d.beams[0].bd.scale_spring == 0.36f, "bmw: set_beam_defaults_scale snapshot");
    }
    if (file == "ViperGTS.car" || file == "ViperGTS_Race.car") {
        CHECK(d.nodes_explicit.size() == 145 && d.cinecams.size() == 3, "viper: 145 nodes + 3 cinecams expected");
        CHECK(d.wheels.size() == 4 && d.wheels[0].first_node == 148 && d.nodes.size() == 276,
              "viper: 4x16-ray wheels from node 148 expected");
        CHECK(beams_use_range(d, 148, 275), "viper: beams must use generated nodes 148-275");
        size_t want = file == "ViperGTS.car" ? 6 : 2;
        CHECK(d.configs.size() == want && d.selected_config == d.configs[0], "viper: %zu configs, first selected", want);
    }
    if (file.rfind("CLK_", 0) == 0) {
        // wheels of non-selected sections are numbered but not spawned (3-4 wheel configurations per file)
        CHECK(d.wheels.size() == 4 && d.configs.size() >= 3, "clk: only the selected section's 4 wheels");
        CHECK(d.nodes.size() == d.nodes_explicit.size() + d.cinecams.size() + 4 * 4 * 14, "clk: node slots after dropping");
    }
    if (file == "1988AudiQuattro.car") {
        // "20000099999999999999999999999999999999999999" > FLT_MAX means "never breaks", not 0
        bool huge = false;
        for (auto& s : d.shocks) huge |= s.bd.brk >= 1e38f;
        CHECK(huge, "quattro: overflowing break threshold must clamp to FLT_MAX");
    }
    if (file == "BajaTrophyTruck.truck") {
        CHECK(d.engine.present && d.engine.neutral_ratio == 12.0f && d.engine.rev_ratio == 15.0f &&
                  d.engine.gears == std::vector<float>({8, 5, 2.8f, 2, 1.5f, 1}),
              "baja engine: neutral 12, 6 forward gears");
        CHECK(d.engoption.present && d.engoption.inertia == 5 && d.engoption.type == 't' && d.engoption.clutch_force == -1,
              "baja engoption");
        CHECK(d.has_brakes && d.brakes.force == 16000 && d.brakes.parking == -1, "baja brakes");
        CHECK(d.cab_material == "tracks/Trophytruck" && d.dry_mass == 2000 && d.cargo_mass == 3000, "baja globals");
    }
}

std::string temp_file(const char* name, const std::string& text) {
    std::string path = (std::filesystem::temp_directory_path() / name).string();
    FILE* f = fopen(path.c_str(), "wb");
    if (f) {
        fwrite(text.data(), 1, text.size(), f);
        fclose(f);
    }
    return path;
}

// Grammar corners not present in the assets (nodes2, wheels2, flexbodywheels, shocks3, defaults, modules ...).
const char* kSynthetic = R"(Synthetic Test Vehicle   
; comment
fileformatversion 3
author chassis 123 Some_Author
guid abc-123
description
nodes
line two
end_description
globals
1000, -5, mymat
minimass
5 l
set_default_minimass 2.5
enable_advanced_deformation
set_beam_defaults_scale 1 2
set_beam_defaults_scale 0.5, 0.6, 0.7, 0.8
set_node_defaults 10, 0.5, -1, 2, c
nodes2
na, 0, 0, 0, l 5
nb, 1, 0, 0
na, 2, 0, 0
set_node_defaults -1
nodes
2, 0, 1, 0, x
5, 0, 0, 0
3, 1, 1, 0, n 7
4, 0, 0, 1
5, 1, 0, 1
6, 0, 1, 1
7, 1, 1, 1
beams
na, nb
2, 3, vis, 3
3, 4, s, -2
4, 5, s
5, 6, rx
set_beam_defaults 100, -1
6, 7
set_beam_defaults -1, 50, 20000, 30000, 0.1, mat2, 0.5
7, na
detacher_group 3
na, 99
detacher_group end
nb, 2
SlopeBrake 15, 5, 12
0, 2
beams
0, 3
sectionconfig 0 cfgA
1, 2
cinecam
0.5, 0.5, 0.5, na, nb, 2, 3, 4, 5, 6, 7
0.5, 0.5, 0.5, 0, 1, 2, 3, 4, 5, 6, 7, 5000, 400, ;x
set_node_defaults -1, 0.8
wheels2
0.2, 0.4, 0.3, 6, 4, 5, 9999, 1, 1, 6, 30, 100000, 1000, 200000, 2000, facemat bandmat
flexbodywheels
0.4, 0.25, 0.3, 4, 6, 7, 2, 4, 2, 5, 25, 300000, 3000, 400000, 4000, r, rim.mesh, tyre.mesh
meshwheels2
0.4, 0.25, 0.3, 3, 6, 7, 9999, 7, 3, 5, 25, 300000, 3000, x, rim.mesh tyremat
wheels
0.5, 0.2, 0, 2, 3, 9999, 1, 1, 4, 10, 1000, 10, a, b
shocks
na, nb, 1000, 100, 0.5, 0.6, 1.1, iLx
shocks2
2, 3, 1, 2, 3, 4, 5, 6, 7, 8, 0.1, 0.2, 1.0, M
shocks3
4, 5, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 0.3, 0.4, 0.9, mM
hydros
6, 7, 0.1
6, 7, -0.2, ij
commands
2, 3, 0.2, 0.5, 1.5, 1, 2, cpo Desc_x 0 0 a b 0.5 no
commands2
4, 5, 0.1, 0.3, 0.6, 1.4, 3, 4
engine
1000, 3000, 500, 3.5, 4, 5, 3, 2, 1, -1, 7
engoption
1.5, c, 900
torquecurve
1000, 0.5
2000, 1.0
1,2,3
brakes
5000, 7000
guisettings
speedoMax 5
useMaxRPM 1
axles
w1(4 5), w2(6 7), d(ls) // comment
w1(4 5) w2(6 7)
cameras
0, 1, 2
set_collision_range 0.2
rollon
submesh
texcoords
0, 0.1, 0.2
1, 0.3, 0.4
cab
0, 1, 2, cx
props
backmesh
0, 1, 2, 0, 0, 0, 0, 0, 0, dashboard-rh.mesh, sw.mesh, 1, 2, 3, 200
0, 1, 2, 0, 0, 0, 0, 0, 0, beacon1.mesh, flare_x, 0.1, 0.2, 0.3
0, 1, 2, 0, 0, 0, 0, 0, 0, redbeacon.mesh
0, 1, 2, 0, 0, 0, 0, 0, 0, seat2.mesh
0, 1, 2, 0, 0, 0, 0, 0, 0, leftmirror.mesh
prop_camera_mode 3
cab
2, 3, 4
submesh
cab
4, 5, 6
end_section
flexbodies
0, 1, 2, 0, 0, 0, 0, 0, 0, body.mesh
forset 0-3, 5,
flexbody_camera_mode -1
0, 1, 2, 0, 0, 0, 0, 0, 0, body2.mesh
forset1-2-9,50-60,6
0, 1, 2, 0, 0, 0, 0, 0, 0, body3.mesh
forset
managedmaterials
set_managedmaterials_options 1
mm1 flexmesh_standard d.png - s.png
mm2 mesh_standard d2.png
mm3 bogus_type x.png
mm1 mesh_standard other.png
contacters
3
fixes
4
ropes
0, 1, i
ties
2, 1.5, 0.5, 0.05, 1.0, is, 5000
comment
beams
author x y
end_comment
0, 1
section 0 cfgA
props
0, 1, 2, 0, 0, 0, 0, 0, 0, modA.mesh
end_section
section 0 cfgB extra
props
0, 1, 2, 0, 0, 0, 0, 0, 0, modB.mesh
end_section
author chassis
flexbodies
0, 1, 2, 0, 0, 0, 0, 0, 0, bodyroot.mesh
author chassis 1 X
0, 1, 2, 0, 0, 0, 0, 0, 0, never.mesh
end
2, 3, 4
)";

// the model editor's document: every template written as a truck file, parsed back, the counts kept; an imported
// vehicle written again keeps its nodes, beams, wheels and cab
void test_editor_models() {
    printf("\nmodel editor round trip\n");
    struct T {
        const char* name;
        bl::edit::Model m;
    } tpl[] = {{"cart", bl::edit::make_cart(2.2f, 1.2f, 0.7f, 400)},
               {"box", bl::edit::make_box(3, 2, 2, vec3(2, 1, 1), 300)},
               {"plate", bl::edit::make_plate(4, 3, vec2(2, 1.5f), 100, false)},
               {"sheet", bl::edit::make_plate(6, 6, vec2(2, 2), 60, true)},
               {"cylinder", bl::edit::make_cylinder(8, 3, 0.5f, 2, 200)}};
    for (T& t : tpl) {
        const std::string text = bl::edit::write_truck(t.m);
        const std::string path = temp_file((std::string("bl_editor_") + t.name + ".truck").c_str(), text);
        Document d;
        CHECK(parse_truck_file(path, d), "%s: written model does not parse", t.name);
        for (auto& w : d.warnings) printf("    %s warn: %s\n", t.name, w.c_str());
        CHECK(d.warnings.empty(), "%s: %zu parser warnings", t.name, d.warnings.size());
        CHECK((int)d.nodes_explicit.size() == (int)t.m.nodes.size(), "%s: nodes %zu != %zu", t.name, d.nodes_explicit.size(), t.m.nodes.size());
        CHECK(d.beams.size() == t.m.beams.size(), "%s: beams %zu != %zu", t.name, d.beams.size(), t.m.beams.size());
        CHECK(d.wheels.size() == t.m.wheels.size(), "%s: wheels %zu != %zu", t.name, d.wheels.size(), t.m.wheels.size());
        size_t cabs = 0;
        for (auto& sm : d.submeshes) cabs += sm.cabs.size();
        CHECK(cabs == t.m.tris.size(), "%s: cab triangles %zu != %zu", t.name, cabs, t.m.tris.size());
        CHECK(d.engine.present == t.m.engine, "%s: engine", t.name);
        CHECK((t.m.shell_count() > 0 ? d.cab_material.rfind("sheet/", 0) == 0 : d.cab_material.rfind("color/", 0) == 0), "%s: cab material '%s'", t.name, d.cab_material.c_str());
        CHECK(d.joints.size() == t.m.joints.size(), "%s: joints %zu != %zu", t.name, d.joints.size(), t.m.joints.size());
        CHECK(d.shells.size() == (t.m.shell_count() > 0 && t.m.shell_count() < (int)t.m.tris.size() ? (size_t)t.m.shell_count() : 0), "%s: shells section", t.name);
        CHECK(bl::edit::validate(t.m).empty(), "%s: validation problems", t.name);
        // import the parsed document and write it again: the same file
        bl::edit::Model back;
        std::vector<std::string> notes;
        CHECK(bl::edit::import_document(d, back, notes), "%s: import failed", t.name);
        CHECK(back.nodes.size() == t.m.nodes.size() && back.beams.size() == t.m.beams.size() && back.tris.size() == t.m.tris.size() && back.wheels.size() == t.m.wheels.size() &&
                  back.joints.size() == t.m.joints.size() && back.shell_count() == t.m.shell_count(),
              "%s: import counts differ", t.name);
        for (size_t i = 0; i < back.nodes.size() && i < t.m.nodes.size(); i++)
            CHECK(length(back.nodes[i].p - t.m.nodes[i].p) < 1e-3f, "%s: node %zu moved", t.name, i);
        printf("  %-9s %3zu nodes %3zu beams %3zu tris %zu wheels: ok\n", t.name, t.m.nodes.size(), t.m.beams.size(), t.m.tris.size(), t.m.wheels.size());
    }
    // layers and groups survive the file as comments
    {
        bl::edit::Model m = bl::edit::make_cart(2.2f, 1.2f, 0.7f, 400);
        m.layers.push_back({"Wheels", false, true});
        m.node_groups.push_back({"front", true});
        m.node_groups.push_back({"rear", false});
        for (int i = 0; i < (int)m.nodes.size(); i++) m.nodes[i].group = m.nodes[i].p.x < 0 ? 0 : 1;
        for (auto& w : m.wheels) w.layer = 1;
        for (int i = 0; i < (int)m.beams.size(); i += 3) m.beams[i].layer = 1;
        m.nodes[3].layer = 1;
        const std::string path = temp_file("bl_editor_layers.truck", bl::edit::write_truck(m));
        Document d;
        CHECK(parse_truck_file(path, d) && d.warnings.empty(), "layered model parse (%zu warnings)", d.warnings.size());
        bl::edit::Model back;
        std::vector<std::string> notes;
        std::string text;
        CHECK(bl::edit::import_document(d, back, notes) && read_text_file(path, text), "layered model import");
        bl::edit::read_markers(text, back);
        CHECK(back.layers.size() == 2 && back.layers[1].name == "Wheels" && !back.layers[1].visible && back.layers[1].locked, "layers read back");
        CHECK(back.node_groups.size() == 2 && back.node_groups[0].name == "front" && !back.node_groups[1].visible, "groups read back");
        bool same = back.nodes.size() == m.nodes.size() && back.beams.size() == m.beams.size() && back.wheels.size() == m.wheels.size();
        for (size_t i = 0; same && i < m.nodes.size(); i++) same = back.nodes[i].layer == m.nodes[i].layer && back.nodes[i].group == m.nodes[i].group;
        for (size_t i = 0; same && i < m.beams.size(); i++) same = back.beams[i].layer == m.beams[i].layer;
        for (size_t i = 0; same && i < m.wheels.size(); i++) same = back.wheels[i].layer == m.wheels[i].layer;
        CHECK(same, "layer / group of every node, beam and wheel read back");
        printf("  layers and groups: ok\n");
    }
    // a detailed mod through the editor: its graphics (flexbodies, props, mesh wheels, submeshes, managed materials)
    // survive the round trip, bound to the same nodes
    for (const char* rel : {"toyota_ae86/ae86_levin.truck", "audi_80/Audi80Quattro.truck"}) {
        Document d;
        CHECK(parse_truck_file(path_join(asset_path("vehicles"), rel), d), "%s parse", rel);
        bl::edit::Model m;
        std::vector<std::string> notes;
        CHECK(bl::edit::import_document(d, m, notes), "ae86 import");
        CHECK(std::string(rel).rfind(m.home + "/", 0) == 0, "%s home folder '%s'", rel, m.home.c_str());
        const std::string text = bl::edit::write_truck(m);
        const std::string path = path_join(path_dir(d.path), ".bl_editor_roundtrip.truck"); // (in its folder: the meshes resolve)
        Document d2;
        CHECK(parse_truck_text(text, path, d2), "ae86 rewrite parse");
        size_t cabs = 0, cabs2 = 0, tc = 0, tc2 = 0;
        for (auto& sm : d.submeshes) cabs += sm.cabs.size(), tc += sm.texcoords.size();
        for (auto& sm : d2.submeshes) cabs2 += sm.cabs.size(), tc2 += sm.texcoords.size();
        CHECK(d2.flexbodies.size() == d.flexbodies.size() && d2.props.size() == d.props.size(), "ae86 flexbodies %zu/%zu props %zu/%zu", d2.flexbodies.size(),
              d.flexbodies.size(), d2.props.size(), d.props.size());
        CHECK(d2.wheels.size() == d.wheels.size() && d2.managed_materials.size() == d.managed_materials.size(), "ae86 wheels / managed materials");
        CHECK(cabs2 == cabs && d2.submeshes.size() == d.submeshes.size(), "ae86 cab %zu/%zu submeshes %zu/%zu", cabs2, cabs, d2.submeshes.size(), d.submeshes.size());
        bool same_wheels = true;
        for (size_t i = 0; i < d.wheels.size() && i < d2.wheels.size(); i++)
            same_wheels &= d2.wheels[i].type == d.wheels[i].type && d2.wheels[i].rim_mesh == d.wheels[i].rim_mesh && std::fabs(d2.wheels[i].radius - d.wheels[i].radius) < 1e-4f;
        CHECK(same_wheels, "ae86 wheel types and meshes");
        // a flexbody bound to the same nodes (by position: the model renumbers the nodes)
        bool same_bind = true;
        auto pos = [](const Document& doc, int n) {
            const ror::NodeSlot& s = doc.nodes[n];
            return s.kind == ror::NodeSlot::EXPLICIT ? doc.nodes_explicit[s.ref].pos : vec3(1e9f);
        };
        for (size_t i = 0; i < d.flexbodies.size() && i < d2.flexbodies.size(); i++) {
            same_bind &= length(pos(d, d.flexbodies[i].ref) - pos(d2, d2.flexbodies[i].ref)) < 1e-3f && d.flexbodies[i].mesh == d2.flexbodies[i].mesh;
            same_bind &= d2.flexbodies[i].forset.size() <= d.flexbodies[i].forset.size() && d2.flexbodies[i].forset.size() > 0;
        }
        CHECK(same_bind, "ae86 flexbodies bound to the same nodes");
        printf("  %s graphics: %zu flexbodies, %zu props, %zu wheels, %zu submeshes (%zu tris, %zu texcoords), %zu managed materials: ok\n", rel, d2.flexbodies.size(),
               d2.props.size(), d2.wheels.size(), d2.submeshes.size(), cabs2, tc2, d2.managed_materials.size());
        (void)tc;
        // meshes switched off (not in the vehicle, a comment) and hidden (the editor's view) come back from the file
        if (m.flexbodies.size() >= 3) {
            bl::edit::Model mo = m;
            mo.flexbodies[0].enabled = false, mo.flexbodies[0].hidden = true;
            mo.flexbodies[1].hidden = true;
            if (!mo.props.empty()) mo.props[0].enabled = false;
            const std::string t2 = bl::edit::write_truck(mo), tp = bl::edit::write_truck(mo, true);
            Document d3, dp;
            CHECK(parse_truck_text(t2, path, d3) && parse_truck_text(tp, path, dp), "%s switched off: parse", rel);
            CHECK(d3.flexbodies.size() == mo.flexbodies.size() - 1 && d3.props.size() + (mo.props.empty() ? 0 : 1) == mo.props.size(), "%s switched off: not in the vehicle", rel);
            CHECK(dp.flexbodies.size() == mo.flexbodies.size() && dp.props.size() == mo.props.size(), "%s preview: every mesh", rel);
            bl::edit::Model back;
            std::vector<std::string> n3;
            CHECK(bl::edit::import_document(d3, back, n3), "%s switched off: import", rel);
            bl::edit::read_markers(t2, back);
            int off = 0, hid = 0;
            for (const auto& f : back.flexbodies) off += !f.enabled, hid += f.hidden;
            bool same = back.flexbodies.size() == mo.flexbodies.size() && back.props.size() == mo.props.size();
            if (same) {
                const auto& f = back.flexbodies.back(); // (switched off: read back at the end)
                same = f.mesh == mo.flexbodies[0].mesh && length(back.ref_position(f.ref) - mo.ref_position(mo.flexbodies[0].ref)) < 1e-3f &&
                       f.forset.size() == mo.flexbodies[0].forset.size() && length(f.offset - mo.flexbodies[0].offset) < 1e-3f;
            }
            CHECK(same && off == 1 && hid == 2, "%s switched off / hidden read back (%d off, %d hidden)", rel, off, hid);
            printf("  %s: meshes switched off and hidden survive the file: ok\n", rel);
        }
    }
    // shell materials: a sheet of steel with a glass part written with set_shell_material and read back
    {
        bl::edit::Model m = bl::edit::make_plate(5, 4, vec2(2, 1.5f), 60, true);
        bl::edit::ShellPreset g;
        g.name = "Window glass", g.material = "Glass", g.kg_m2 = 10, g.thickness = 0.005f, g.color = vec3(0.6f, 0.8f, 0.9f);
        m.shell_presets.push_back(g);
        int glass = 0;
        for (size_t i = 0; i < m.tris.size(); i += 2) m.tris[i].shell_preset = 1, glass++;
        const std::string text = bl::edit::write_truck(m);
        Document d;
        CHECK(parse_truck_text(text, temp_file("bl_editor_shellmats.truck", text), d), "shell materials: parse");
        int sm1 = 0;
        for (const auto& sh : d.shells) sm1 += sh.mat == 1;
        CHECK(d.shell_materials.size() == 1 && d.shell_materials[0].material == "Glass" && std::fabs(d.shell_materials[0].kg_m2 - 10) < 1e-4f && sm1 == glass &&
                  (int)d.shells.size() == m.shell_count(),
              "shell materials written: %zu materials, %d of %d glass shells, %zu shells", d.shell_materials.size(), sm1, glass, d.shells.size());
        bl::edit::Model back;
        std::vector<std::string> notes;
        CHECK(bl::edit::import_document(d, back, notes), "shell materials: import");
        int g2 = 0;
        for (const auto& t : back.tris) g2 += t.shell && t.shell_preset == 1;
        CHECK(back.shell_presets.size() == 1 && back.shell_presets[0].material == "Glass" && g2 == glass, "shell materials read back (%d glass)", g2);
        printf("  shell materials: %d glass shells of %d: ok\n", glass, m.shell_count());
    }
    // merging nodes: a node's copy merged back into it takes its elements; what folds up and the doubles go
    {
        bl::edit::Model m = bl::edit::make_box(2, 2, 2, vec3(1, 1, 1), 100);
        const int n0 = (int)m.nodes.size(), b0 = (int)m.beams.size();
        bl::edit::Node copy = m.nodes[0];
        m.nodes.push_back(copy);
        const int c = n0;
        int far = -1;
        for (int i = 1; i < n0; i++)
            if (!m.has_beam(0, i) && far < 0) far = i;
        m.add_beam(c, 0, 0);                  // (folds up)
        m.add_beam(c, 1, 0);                  // (a double of 0-1 if the box has it)
        if (far >= 0) m.add_beam(c, far, 0);  // (goes to node 0)
        const size_t t0 = m.tris.size();
        bl::edit::Tri t;
        t.a = 0, t.b = c, t.c = 1;            // (folds up)
        m.tris.push_back(t);
        const bool had01 = m.has_beam(0, 1);
        const int gone = m.merge_nodes({{0, c}}, {m.nodes[0].p});
        bool dup = false;
        for (size_t i = 0; i < m.beams.size(); i++)
            for (size_t j = i + 1; j < m.beams.size(); j++)
                dup |= std::min(m.beams[i].a, m.beams[i].b) == std::min(m.beams[j].a, m.beams[j].b) && std::max(m.beams[i].a, m.beams[i].b) == std::max(m.beams[j].a, m.beams[j].b);
        const int expect = b0 + (had01 ? 0 : 1) + (far >= 0 ? 1 : 0);
        CHECK(gone == 1 && (int)m.nodes.size() == n0 && (int)m.beams.size() == expect && !dup && m.tris.size() == t0 && (far < 0 || m.has_beam(0, far)),
              "merge: %d gone, %zu nodes, %zu beams (expected %d), %zu tris, doubles %d", gone, m.nodes.size(), m.beams.size(), expect, m.tris.size(), (int)dup);
        printf("  merging nodes: ok\n");
    }
    // a collision hull's triangles (option h: one-sided, solid) through the file: written with c and h, read back as
    // collision cabs with h, imported as hull triangles again (the others stay plain cabs)
    {
        bl::edit::Model m = bl::edit::make_box(2, 2, 2, vec3(1, 1, 1), 100);
        const size_t plain = m.tris.size();
        for (auto f : {std::array<int, 3>{0, 1, 2}, std::array<int, 3>{0, 2, 3}}) {
            bl::edit::Tri t;
            t.a = f[0], t.b = f[1], t.c = f[2], t.options = "ch";
            m.tris.push_back(t);
        }
        const std::string path = temp_file("bl_editor_hull.truck", bl::edit::write_truck(m));
        Document d;
        CHECK(parse_truck_file(path, d) && d.warnings.empty(), "hull model parse (%zu warnings)", d.warnings.size());
        int with_h = 0, with_c = 0;
        for (auto& sm : d.submeshes)
            for (auto& c : sm.cabs) with_h += c.options.find('h') != std::string::npos, with_c += c.options.find('c') != std::string::npos;
        bl::edit::Model back;
        std::vector<std::string> notes;
        CHECK(bl::edit::import_document(d, back, notes), "hull model import");
        int hulls = 0;
        for (const auto& t : back.tris) hulls += t.hull() && t.collision;
        CHECK(with_h == 2 && with_c >= 2 && hulls == 2 && back.tris.size() == plain + 2, "hull triangles: %d with h, %d with c, %d read back as hull of %zu", with_h, with_c, hulls,
              back.tris.size());
        printf("  collision hull triangles: ok\n");
    }
    // mounts (a part held on the frame at a distance): node a, node b, break force[, stiffness[, turning damping]]
    {
        const std::string text = "Mounted\nglobals\n100, 0\nnodes\n0, 0, 0, 0\n1, 1, 0, 0\n2, 0, 1, 0\n3, 1, 1, 0\nset_frame_section Steel, tube, 0.03, 0.002\nbeams\n0, 1, F\n2, 3, F\n"
                                 "mounts\n1, 3, 5000, 2e6, 3\n0, 2, 800\nend\n";
        const std::string path = temp_file("bl_mounts.truck", text);
        Document d;
        CHECK(parse_truck_file(path, d) && d.warnings.empty(), "mounts parse (%zu warnings)", d.warnings.size());
        CHECK(d.mounts.size() == 2 && d.mounts[0].a == 1 && d.mounts[0].b == 3 && d.mounts[0].brk == 5000 && d.mounts[0].k == 2e6f && d.mounts[0].damp == 3 &&
                  d.mounts[1].brk == 800 && d.mounts[1].k == 0 && d.mounts[1].damp == 0,
              "mounts read: %zu", d.mounts.size());
        printf("  mounts: ok\n");
    }
    // the mounts' kinds (`..., kind[, parameter]`: c a clamp and its break moment, h a hinge and its second node, s a
    // stop, r a strap and its length); the editor writes them back as it read them
    {
        const std::string text = "Mounted kinds\nglobals\n100, 0\nnodes\n0, 0, 0, 0\n1, 1, 0, 0\n2, 0, 1, 0\n3, 1, 1, 0\n4, 0.5, 1.5, 0\n"
                                 "set_frame_section Steel, tube, 0.03, 0.002\nbeams\n0, 1, F\n2, 3, F\n3, 4, F\n"
                                 "mounts\n1, 3, 5000, 0, 0, c, 250\n0, 2, 800, 0, 2, h, 4\n1, 4, 0, 1e5, 0, s\n0, 4, 20000, 0, 0, r, 1.6\n1, 2, 900\nend\n";
        const std::string path = temp_file("bl_mount_kinds.truck", text);
        Document d;
        CHECK(parse_truck_file(path, d) && d.warnings.empty(), "mount kinds parse (%zu warnings)", d.warnings.size());
        const bool read_ok = d.mounts.size() == 5 && d.mounts[0].kind == 'c' && d.mounts[0].param == 250 && d.mounts[1].kind == 'h' && d.mounts[1].b2 == 4 &&
                             d.mounts[1].damp == 2 && d.mounts[2].kind == 's' && d.mounts[2].k == 1e5f && d.mounts[3].kind == 'r' && d.mounts[3].param == 1.6f &&
                             d.mounts[4].kind == 'p';
        CHECK(read_ok, "mount kinds read: %zu", d.mounts.size());
        bl::edit::Model m;
        std::vector<std::string> notes;
        Document d2;
        const bool back = bl::edit::import_document(d, m, notes) && parse_truck_file(temp_file("bl_mount_kinds2.truck", bl::edit::write_truck(m)), d2);
        bool same = back && d2.mounts.size() == d.mounts.size();
        for (size_t i = 0; same && i < d.mounts.size(); i++)
            same = d2.mounts[i].kind == d.mounts[i].kind && d2.mounts[i].param == d.mounts[i].param && d2.mounts[i].b2 == d.mounts[i].b2 &&
                   d2.mounts[i].brk == d.mounts[i].brk && d2.mounts[i].k == d.mounts[i].k;
        CHECK(same, "mount kinds: the editor's round trip changed them");
        printf("  mount kinds: ok\n");
    }
    // collision volumes (`collision_volumes`: "volume name, break rms", "anchors ...", "vertex x, y, z"); the editor writes
    // them back as it read them
    {
        const std::string text = "Volumes\nglobals\n100, 0\nnodes\n0, 0, 0, 0\n1, 1, 0, 0\n2, 0, 1, 0\n3, 0, 0, 1\nbeams\n0, 1\n0, 2\n0, 3\n"
                                 "collision_volumes\nvolume engine, 0.1\nanchors 0, 1, 2, 3\nvertex 0.1, 0.1, 0.1\nvertex 0.5, 0.1, 0.1\n"
                                 "vertex 0.1, 0.5, 0.1\nvertex 0.1, 0.1, 0.5\nvolume cabin\nanchors 1, 2, 3\nvertex 0, 0, 0\nvertex 1, 0, 0\nvertex 0, 1, 0\nvertex 0, 0, 1\nend\n";
        Document d;
        CHECK(parse_truck_file(temp_file("bl_volumes.truck", text), d) && d.warnings.empty(), "collision_volumes parse (%zu warnings)", d.warnings.size());
        const bool read_ok = d.volumes.size() == 2 && d.volumes[0].name == "engine" && d.volumes[0].break_rms == 0.1f && d.volumes[0].anchors.size() == 4 &&
                             d.volumes[0].verts.size() == 4 && d.volumes[0].verts[1].x == 0.5f && d.volumes[1].name == "cabin" && d.volumes[1].break_rms == 0.12f &&
                             d.volumes[1].anchors.size() == 3 && d.volumes[1].anchors[2] == 3;
        CHECK(read_ok, "collision volumes read: %zu", d.volumes.size());
        bl::edit::Model m;
        std::vector<std::string> notes;
        Document d2;
        const bool back = bl::edit::import_document(d, m, notes) && parse_truck_file(temp_file("bl_volumes2.truck", bl::edit::write_truck(m)), d2);
        bool same = back && d2.volumes.size() == d.volumes.size();
        for (size_t i = 0; same && i < d.volumes.size(); i++)
            same = d2.volumes[i].name == d.volumes[i].name && d2.volumes[i].break_rms == d.volumes[i].break_rms && d2.volumes[i].anchors == d.volumes[i].anchors &&
                   d2.volumes[i].verts.size() == d.volumes[i].verts.size() && length(d2.volumes[i].verts.back() - d.volumes[i].verts.back()) < 1e-5f;
        CHECK(same, "collision volumes: the editor's round trip changed them");
        printf("  collision volumes: ok\n");
    }
    // FEM triangles: `fem_tris` with `set_fem_shell material, thickness[, r, g, b]` before its triangles (a triangle
    // before any: the default, 1 mm steel); the editor writes them back as it read them
    {
        const std::string text = "Fem\nglobals\n0, 0\nnodes\n0, 0, 0, 0\n1, 1, 0, 0\n2, 0, 0, 1\n3, 1, 0, 1\n4, 1, 1, 1\nfem_tris\n0, 1, 2\n"
                                 "set_fem_shell Aluminium, 0.002, 0.2, 0.3, 0.4\n1, 3, 2\n3, 4, 2\nend\n";
        Document d;
        CHECK(parse_truck_file(temp_file("bl_fem_tris.truck", text), d) && d.warnings.empty(), "fem_tris parse (%zu warnings)", d.warnings.size());
        CHECK(d.fem_tris.size() == 3 && d.fem_shells.size() == 2 && d.fem_tris[0].shell == 0 && d.fem_tris[1].shell == 1 && d.fem_tris[2].shell == 1 &&
                  d.fem_tris[1].n1 == 1 && d.fem_tris[1].n2 == 3 && d.fem_tris[1].n3 == 2,
              "fem_tris read: %zu triangles, %zu shells", d.fem_tris.size(), d.fem_shells.size());
        if (d.fem_shells.size() == 2)
            CHECK(d.fem_shells[0].material == "Steel" && d.fem_shells[0].thickness == 0.001f && d.fem_shells[1].material == "Aluminium" &&
                      d.fem_shells[1].thickness == 0.002f && std::fabs(d.fem_shells[1].color.y - 0.3f) < 1e-6f,
                  "fem shells: %s %g, %s %g", d.fem_shells[0].material.c_str(), d.fem_shells[0].thickness, d.fem_shells[1].material.c_str(), d.fem_shells[1].thickness);
        bl::edit::Model back;
        std::vector<std::string> notes;
        CHECK(bl::edit::import_document(d, back, notes) && back.fem_count() == 3 && back.fem_presets.size() == 2 && back.tris[2].fem_preset == 1 &&
                  !back.tris[0].collision && !back.tris[0].shell,
              "fem_tris into the editor: %d FEM triangles, %zu presets", back.fem_count(), back.fem_presets.size());
        // the editor's box of FEM triangles, written and read back
        bl::edit::Model box = bl::edit::make_fem_box(3, vec3(1, 1, 1), 0.0015f);
        bl::edit::FemPreset thick = box.fem_presets[0];
        thick.name = "Thick", thick.thickness = 0.004f, thick.material = "Chromoly";
        box.fem_presets.push_back(thick);
        for (int i = 0; i < 6; i++) box.tris[i].fem_preset = 1;
        Document db;
        bl::edit::Model box2;
        CHECK(parse_truck_file(temp_file("bl_fem_box.truck", bl::edit::write_truck(box)), db) && db.warnings.empty() && bl::edit::import_document(db, box2, notes),
              "FEM box round trip: %zu warnings", db.warnings.size());
        int ones = 0;
        for (const auto& t : box2.tris) ones += t.fem && t.fem_preset == 1;
        CHECK(db.fem_tris.size() == box.tris.size() && db.beams.empty() && db.submeshes.empty() && box2.fem_count() == (int)box.tris.size() && ones == 6 &&
                  box2.fem_presets.size() == 2 && box2.fem_presets[1].material == "Chromoly" && box2.fem_presets[1].thickness == 0.004f,
              "FEM box: %zu of %zu triangles, %zu beams, %d of the thick shell", db.fem_tris.size(), box.tris.size(), db.beams.size(), ones);
        printf("  FEM triangles: ok\n");
    }
    // the presets as they were: beam presets' names and colours (two with the same numbers stay two, an unused one
    // stays), shell materials' names with spaces, FEM shells' names and a paint colour (-1); they were rebuilt from
    // the directives: named by their numbers, merged, the unused ones gone
    {
        bl::edit::Model m = bl::edit::make_box(2, 2, 2, vec3(1, 1, 1), 100);
        bl::edit::BeamGroup g0 = m.groups[0];
        m.groups.clear();
        for (int i = 0; i < 3; i++) {
            bl::edit::BeamGroup g = g0;
            g.name = i == 0 ? "Main frame" : i == 1 ? "Side rails" : "Spare, unused";
            g.color = vec4(0.1f * (i + 1), 0.5f, 1.0f - 0.2f * i, 1.0f);
            m.groups.push_back(g);
        }
        for (size_t i = 0; i < m.beams.size(); i++) m.beams[i].group = (int)(i % 2); // (the first two: the same numbers)
        bl::edit::ShellPreset sa, sb;
        sa.name = "Window glass 4 mm", sa.material = "Glass", sa.color = vec3(0.2f, 0.4f, 0.6f);
        sb.name = "Unused panel", sb.material = "Aluminium", sb.color = vec3(-1);
        m.shell_presets = {sa, sb};
        for (size_t i = 0; i < m.tris.size(); i++)
            if (i < 4) m.tris[i].shell = true, m.tris[i].shell_preset = 1;
        bl::edit::FemPreset fa, fb, fc;
        fa.name = "Roof skin", fa.thickness = 0.0008f, fa.color = vec3(-1);
        fb.name = "Floor, thick", fb.material = "Aluminium", fb.thickness = 0.002f, fb.color = vec3(0.3f, 0.3f, 0.35f);
        fc.name = "Unused reinforcement", fc.thickness = 0.0025f;
        m.fem_presets = {fa, fb, fc};
        for (size_t i = 4; i < m.tris.size(); i++) m.tris[i].fem = true, m.tris[i].collision = false, m.tris[i].fem_preset = i % 2 ? 1 : 0;
        const std::string text = bl::edit::write_truck(m);
        Document d;
        bl::edit::Model b;
        std::vector<std::string> notes;
        CHECK(parse_truck_file(temp_file("bl_presets.truck", text), d) && d.warnings.empty() && bl::edit::import_document(d, b, notes), "presets file: %zu warnings",
              d.warnings.size());
        bl::edit::read_markers(text, b);
        bool beams_ok = b.groups.size() == 3 && b.beams.size() == m.beams.size();
        for (size_t i = 0; beams_ok && i < 3; i++)
            beams_ok = b.groups[i].name == m.groups[i].name && std::fabs(b.groups[i].color.x - m.groups[i].color.x) + std::fabs(b.groups[i].color.y - m.groups[i].color.y) + std::fabs(b.groups[i].color.z - m.groups[i].color.z) + std::fabs(b.groups[i].color.w - m.groups[i].color.w) < 1e-3f && b.groups[i].spring == m.groups[i].spring;
        for (size_t i = 0; beams_ok && i < b.beams.size(); i++) beams_ok = b.beams[i].group == m.beams[i].group;
        CHECK(beams_ok, "beam presets: %zu read back (%s, %s)", b.groups.size(), b.groups.empty() ? "?" : b.groups[0].name.c_str(),
              b.groups.size() > 1 ? b.groups[1].name.c_str() : "?");
        bool shells_ok = b.shell_presets.size() == 2 && b.shell_presets[0].name == sa.name && b.shell_presets[1].name == sb.name &&
                         length(b.shell_presets[0].color - sa.color) < 1e-3f && b.shell_presets[1].color.x < 0 && b.shell_presets[0].material == "Glass";
        int shelled = 0;
        for (const auto& t : b.tris) shelled += t.shell && t.shell_preset == 1;
        CHECK(shells_ok && shelled == 4, "shell materials: %zu (%s), %d triangles of the first", b.shell_presets.size(),
              b.shell_presets.empty() ? "?" : b.shell_presets[0].name.c_str(), shelled);
        bool fem_ok = b.fem_presets.size() == 3;
        for (size_t i = 0; fem_ok && i < 3; i++)
            fem_ok = b.fem_presets[i].name == m.fem_presets[i].name && b.fem_presets[i].material == m.fem_presets[i].material &&
                     std::fabs(b.fem_presets[i].thickness - m.fem_presets[i].thickness) < 1e-6f && length(b.fem_presets[i].color - m.fem_presets[i].color) < 1e-3f;
        int ones = 0, fems = 0;
        for (const auto& t : b.tris) fems += t.fem, ones += t.fem && t.fem_preset == 1;
        int want = 0;
        for (size_t i = 4; i < m.tris.size(); i++) want += i % 2;
        CHECK(fem_ok && fems == (int)m.tris.size() - 4 && ones == want, "FEM shells: %zu (%s), %d of %d triangles, %d of the second (%d)", b.fem_presets.size(),
              b.fem_presets.empty() ? "?" : b.fem_presets[0].name.c_str(), fems, (int)m.tris.size() - 4, ones, want);
        printf("  presets' names and colours: ok\n");
    }
    // the cockpit camera (cinecam) only when asked for, and only in a file to drive: the editor's preview and physics
    // test get no node and beams of it (a shell-only shape would have had its eight beams along its edges); an older
    // editor's file (it wrote one for every model) reads back without it, a foreign vehicle's with it
    {
        bl::edit::Model m = bl::edit::make_box(2, 2, 2, vec3(1, 1, 1), 100);
        m.beams.clear();
        auto parse = [&](const char* name, const std::string& text, Document& d) { return parse_truck_file(temp_file(name, text), d); };
        Document d0, d1, dp;
        CHECK(parse("bl_editor_cam0.truck", bl::edit::write_truck(m), d0) && d0.cinecams.empty(), "no cockpit node asked: %zu cinecams", d0.cinecams.size());
        m.cinecam = true;
        const std::string with = bl::edit::write_truck(m);
        CHECK(parse("bl_editor_cam1.truck", with, d1) && d1.cinecams.size() == 1, "cockpit node: %zu cinecams", d1.cinecams.size());
        CHECK(parse("bl_editor_camp.truck", bl::edit::write_truck(m, true), dp) && dp.cinecams.empty() && dp.beams.empty(),
              "preview: %zu cinecams, %zu beams", dp.cinecams.size(), dp.beams.size());
        auto read_back = [&](const std::string& text) {
            Document d;
            bl::edit::Model b;
            std::vector<std::string> notes;
            if (!parse_truck_file(temp_file("bl_editor_camr.truck", text), d) || !bl::edit::import_document(d, b, notes)) return -1;
            bl::edit::read_markers(text, b);
            return b.cinecam ? 1 : 0;
        };
        std::string legacy = with;
        legacy.erase(legacy.find(";editor-cinecam"), legacy.find('\n', legacy.find(";editor-cinecam")) - legacy.find(";editor-cinecam") + 1);
        std::string foreign = with;
        foreign.erase(foreign.find(";written by the BeamLab model editor"), std::string(";written by the BeamLab model editor").size());
        const int r_with = read_back(with), r_legacy = read_back(legacy), r_foreign = read_back(foreign);
        CHECK(r_with == 1 && r_legacy == 0 && r_foreign == 1, "cockpit node read back: asked %d, older editor file %d, foreign %d", r_with, r_legacy, r_foreign);
        printf("  cockpit node only when asked, never in the preview: ok\n");
    }
    // frame elements: a preset of that type writes its section, its joints and the option F (no joints section), reads
    // back as the same preset; one with other joints (a ball at a, a hinge at b, elastic) too, and a beam's own joints
    {
        bl::edit::Model m = bl::edit::make_box(2, 2, 2, vec3(1, 1, 1), 100);
        bl::edit::BeamGroup g;
        g.name = "cage", g.type = bl::edit::BEAM_FRAME;
        g.frame_material = "Chromoly", g.frame_shape = 1, g.frame_outer = 0.03f, g.frame_wall = 0.0015f;
        m.groups.push_back(g);
        g.name = "strut", g.frame_shape = 0, g.frame_end_a = 1, g.frame_end_b = 5, g.frame_joint_k = 3000.0f; // (ball / elastic)
        g.frame_break = 1500.0f, g.frame_joint_damp = 2.5f; // (a mount that tears, a damped joint)
        m.groups.push_back(g);
        const int cage = (int)m.groups.size() - 2, strut = cage + 1;
        for (size_t i = 0; i < m.beams.size(); i++) m.beams[i].group = i < 4 ? cage : i < 6 ? strut : m.beams[i].group;
        m.beams[1].end_b = 2; // (its own: a hinge swinging in its vertical plane at b)
        const std::string text = bl::edit::write_truck(m);
        const std::string path = temp_file("bl_editor_frame.truck", text);
        Document d;
        CHECK(parse_truck_file(path, d), "frame model parse");
        int frames = 0;
        for (const auto& b : d.beams) frames += b.frame >= 0;
        CHECK(frames == 6 && d.frame_sections.size() == 2 && d.joints.empty(), "frame elements in the file: %d, %zu sections, %zu joints", frames, d.frame_sections.size(),
              d.joints.size());
        CHECK(d.frame_sections.size() == 2 && d.frame_sections[0].material == "Chromoly" && d.frame_sections[0].shape == "box" && d.frame_sections[0].end_a == 0 &&
                  d.frame_sections[1].end_a == 1 && d.frame_sections[1].end_b == 5 && d.frame_sections[1].joint_k == 3000.0f &&
                  d.frame_sections[1].brk == 1500.0f && d.frame_sections[1].joint_damp == 2.5f && d.frame_sections[0].brk == 0 && d.frame_sections[0].joint_damp == 0 &&
                  std::fabs(d.frame_sections[0].outer - 0.03f) < 1e-6f,
              "frame sections read back");
        CHECK(d.beams[1].end_a == 0 && d.beams[1].end_b == 2, "a beam's own joints: %d %d", d.beams[1].end_a, d.beams[1].end_b);
        bl::edit::Model back;
        std::vector<std::string> notes;
        CHECK(bl::edit::import_document(d, back, notes), "frame model import");
        int ok_cage = 0, ok_strut = 0;
        for (const auto& b : back.beams) {
            const auto& h = back.groups[b.group];
            if (!h.is_frame()) continue;
            if (h.frame_end_a == 0 && h.frame_end_b == 0 && h.frame_material == "Chromoly" && h.frame_shape == 1 && h.frame_outer == 0.03f && h.frame_wall == 0.0015f) ok_cage++;
            if (h.frame_end_a == 1 && h.frame_end_b == 5 && h.frame_joint_k == 3000.0f && h.frame_shape == 0 && h.frame_break == 1500.0f && h.frame_joint_damp == 2.5f) ok_strut++;
        }
        CHECK(ok_cage == 4 && ok_strut == 2, "frame presets read back: %d welded box members, %d ball / elastic tubes", ok_cage, ok_strut);
        CHECK(back.beams[1].end_a < 0 && back.beams[1].end_b == 2, "a beam's own joints read back: %d %d", back.beams[1].end_a, back.beams[1].end_b);
        printf("  frame elements, their joints, break force and joint damping: ok\n");
    }
    // a rigid preset writes a pair of joints per beam and reads back as a rigid preset
    {
        bl::edit::Model m = bl::edit::make_box(2, 2, 2, vec3(1, 1, 1), 100);
        bl::edit::BeamGroup g;
        g.name = "rigid", g.hold_rotation = true, g.joint_k = 12345.0f, g.type = bl::edit::BEAM_ROPE;
        m.groups.push_back(g);
        m.beams[0].group = m.beams[1].group = (int)m.groups.size() - 1;
        const std::string path = temp_file("bl_editor_rigid.truck", bl::edit::write_truck(m));
        Document d;
        CHECK(parse_truck_file(path, d) && d.joints.size() == 4, "rigid preset: %zu joints", d.joints.size());
        bl::edit::Model back;
        std::vector<std::string> notes;
        CHECK(bl::edit::import_document(d, back, notes), "rigid preset import");
        int held = 0;
        for (const auto& b : back.beams) held += back.groups[b.group].hold_rotation && back.groups[b.group].joint_k == 12345.0f && back.groups[b.group].type == bl::edit::BEAM_ROPE;
        CHECK(held == 2 && back.joints.empty(), "rigid preset read back: %d held beams, %zu loose joints", held, back.joints.size());
        printf("  rigid preset: ok\n");
    }
    // a real vehicle through the editor: the sheet car keeps its structure
    {
        Document d;
        CHECK(parse_truck_file(path_join(asset_path("vehicles"), "sheet_car/sheet_car.truck"), d), "sheet car parse");
        bl::edit::Model m;
        std::vector<std::string> notes;
        CHECK(bl::edit::import_document(d, m, notes), "sheet car import");
        CHECK(m.nodes.size() == d.nodes_explicit.size() && m.beams.size() == d.beams.size() && m.wheels.size() == 4 && m.hydros.size() == 2 && m.shell_count() == (int)m.tris.size(),
              "sheet car import counts: %zu nodes %zu beams %zu wheels %zu hydros sheet %d", m.nodes.size(), m.beams.size(), m.wheels.size(), m.hydros.size(), m.shell_count());
        const std::string path = temp_file("bl_editor_sheet_car.truck", bl::edit::write_truck(m));
        Document d2;
        CHECK(parse_truck_file(path, d2), "sheet car rewrite parse");
        CHECK(d2.nodes.size() == d.nodes.size() && d2.beams.size() == d.beams.size() && d2.wheels.size() == d.wheels.size() && d2.hydros.size() == d.hydros.size(),
              "sheet car rewrite counts");
        printf("  sheet car: %zu nodes %zu beams rewritten, %zu notes\n", m.nodes.size(), m.beams.size(), notes.size());
    }
}

void test_synthetic() {
    printf("\nsynthetic grammar test\n");
    std::string path = temp_file("bl_ror_synthetic.truck", kSynthetic);
    Document d;
    CHECK(parse_truck_file(path, d), "synthetic parse failed");
    for (auto& w : d.warnings) printf("    warn: %s\n", w.c_str());
    validate_structure(d);
    CHECK(d.title == "Synthetic Test Vehicle", "title '%s'", d.title.c_str());
    CHECK(d.file_format_version == 3 && d.guid == "abc-123" && d.authors.size() == 3 && d.authors[0] == "Some_Author",
          "header directives");
    CHECK(d.description == "nodes\nline two", "description '%s'", d.description.c_str());
    CHECK(d.dry_mass == 1000 && d.cargo_mass == 0 && d.cab_material == "mymat", "globals");
    CHECK(d.minimass == 5 && d.minimass_skip_loaded, "minimass");
    CHECK(d.nodes_explicit.size() == 8 && d.cinecams.size() == 2 && d.wheels.size() == 3 && d.nodes.size() == 56,
          "node counts: %zu explicit, %zu cinecam, %zu wheels, %zu total", d.nodes_explicit.size(), d.cinecams.size(),
          d.wheels.size(), d.nodes.size());
    if (d.nodes_explicit.size() == 8) {
        const NodeDef& na = d.nodes_explicit[0];
        CHECK(na.name == "na" && na.options == "lc" && na.loaded && na.load_weight == 5 && na.minimass == 2.5f &&
                  na.defaults.load_weight == 10 && na.defaults.friction == 0.5f && na.defaults.volume == 1 &&
                  na.defaults.surface == 2,
              "node na: '%s' %d %g", na.options.c_str(), na.loaded, na.load_weight);
        const NodeDef& nb = d.nodes_explicit[1];
        CHECK(nb.options == "c" && nb.loaded && nb.load_weight == 10, "node nb (default load weight)");
        const NodeDef& n2 = d.nodes_explicit[2];
        CHECK(n2.options == "xc" && !n2.loaded && n2.load_weight == -1 && n2.defaults.friction == 1 &&
                  n2.defaults.load_weight == -1,
              "node 2 (set_node_defaults -1 keeps options) '%s'", n2.options.c_str());
        CHECK(d.nodes_explicit[3].options == "nc" && !d.nodes_explicit[3].loaded, "node 3 weight without l");
    }
    CHECK(d.beams.size() == 10, "beams %zu", d.beams.size()); // `0, 1` after end_comment has no block
    if (d.beams.size() == 10) {
        const auto& b = d.beams;
        CHECK(b[0].n1 == 0 && b[0].n2 == 1 && !b[0].bd.user_defined && b[0].bd.scale_spring == 0.5f &&
                  b[0].bd.scale_break == 0.8f,
              "beam 0");
        CHECK(b[1].options == "vis" && b[1].support_limit == 3, "beam 1");
        CHECK(b[2].support_limit == 4 && b[3].support_limit == 4 && b[4].options == "r", "beams 2-4");
        CHECK(b[5].bd.spring == 100 && b[5].bd.damp == 12000 && b[5].bd.user_defined && b[5].bd.adv_deform &&
                  !b[5].bd.plastic_given,
              "set_beam_defaults 100,-1");
        CHECK(b[6].n2 == 0 && b[6].bd.spring == 9000000 && b[6].bd.damp == 50 && b[6].bd.material == "mat2" &&
                  b[6].bd.plastic_given && b[6].bd.diameter == 0.1f &&
                  std::fabs(b[6].bd.deform_threshold() - 14000) < 1,
              "set_beam_defaults full: deform %g", b[6].bd.deform_threshold());
        CHECK(b[7].n1 == 0 && b[7].n2 == 0 && b[7].detacher_group == 3, "detacher group / invalid ref -> 0");
        CHECK(b[8].n1 == 1 && b[8].n2 == 2 && b[8].detacher_group == 0, "detacher_group end");
        CHECK(b[9].n1 == 0 && b[9].n2 == 3, "SlopeBrake/sectionconfig swallow data lines");
    }
    if (d.cinecams.size() == 2) {
        CHECK(d.nodes[8].kind == NodeSlot::CINECAM && d.cinecams[0].spring == 8000 && d.cinecams[0].node_mass == 20,
              "cinecam 0");
        CHECK(d.cinecams[1].spring == 5000 && d.cinecams[1].damp == 400 && d.cinecams[1].node_mass == 20, "cinecam 1");
    }
    if (d.wheels.size() == 3) {
        const WheelDef& w2 = d.wheels[0];
        CHECK(w2.type == WheelDef::WHEELS2 && w2.rim_radius == 0.2f && w2.radius == 0.4f && w2.rays == 6 &&
                  w2.rigidity == -1 && w2.rim_spring == 100000 && w2.spring == 200000 && w2.damp == 2000 &&
                  w2.face_material == "facemat" && w2.first_node == 10 && w2.node_count() == 24 &&
                  w2.nd.friction == 0.8f,
              "wheels2");
        const WheelDef& fw = d.wheels[1];
        CHECK(fw.type == WheelDef::FLEXBODYWHEELS && fw.radius == 0.4f && fw.rim_radius == 0.25f && fw.rigidity == 2 &&
                  fw.braking == 4 && fw.propulsion == 2 && fw.spring == 300000 && fw.rim_spring == 400000 &&
                  fw.side == 'r' && fw.rim_mesh == "rim.mesh" && fw.tyre_material == "tyre.mesh" && fw.first_node == 34,
              "flexbodywheels");
        const WheelDef& mw = d.wheels[2];
        CHECK(mw.type == WheelDef::MESHWHEELS2 && mw.braking == 0 && mw.propulsion == 0 && mw.side == 'l' &&
                  mw.first_node == 50 && mw.node_count() == 6,
              "meshwheels2 invalid values");
    }
    CHECK(d.shocks.size() == 3, "shocks %zu", d.shocks.size());
    if (d.shocks.size() == 3) {
        CHECK(d.shocks[0].type == 1 && d.shocks[0].options == "iL" && d.shocks[0].precompression == 1.1f, "shock 1");
        CHECK(d.shocks[1].type == 2 && d.shocks[1].prog_damp_out == 8 && d.shocks[1].longbound == 0.2f &&
                  d.shocks[1].options == "M",
              "shock 2");
        CHECK(d.shocks[2].type == 3 && d.shocks[2].damp_out_fast == 10 && d.shocks[2].precompression == 0.9f &&
                  d.shocks[2].options == "mM",
              "shock 3");
    }
    CHECK(d.hydros.size() == 2 && d.hydros[0].options == "n" && d.hydros[1].options == "ij" && d.hydros[1].factor == -0.2f,
          "hydros");
    CHECK(d.commands.size() == 2, "commands %zu", d.commands.size());
    if (d.commands.size() == 2) {
        const CommandDef& c = d.commands[0];
        CHECK(c.rate_short == 0.2f && c.rate_long == 0.2f && c.options == "c" && c.description == "Desc_x" &&
                  c.affect_engine == 0.5f && !c.needs_engine && c.key_extend == 2,
              "commands: '%s'", c.options.c_str());
        const CommandDef& c2 = d.commands[1];
        CHECK(c2.rate_short == 0.1f && c2.rate_long == 0.3f && c2.shortbound == 0.6f && c2.needs_engine, "commands2");
    }
    CHECK(d.engine.present && d.engine.gears == std::vector<float>({3, 2, 1}), "engine gears");
    CHECK(d.engoption.present && d.engoption.type == 'c' && d.engoption.clutch_force == 900 && d.engoption.f4 == -1,
          "engoption");
    CHECK(d.torquecurve.model.empty() && d.torquecurve.points.size() == 2 && d.torquecurve.points[1].x == 2000,
          "torquecurve");
    CHECK(d.has_brakes && d.brakes.force == 5000 && d.brakes.parking == 7000, "brakes");
    CHECK(d.gui.speedo_max == 140 && d.gui.use_max_rpm, "guisettings");
    CHECK(d.axles.size() == 1 && d.axles[0].w1a == 4 && d.axles[0].w2b == 7 && d.axles[0].modes == "ls" &&
              d.has_axles_section,
          "axles");
    CHECK(d.cameras.size() == 1 && d.collision_range == 0.2f && d.rollon, "cameras / collision range / rollon");
    CHECK(d.submeshes.size() == 2, "submeshes %zu", d.submeshes.size());
    if (d.submeshes.size() == 2) {
        CHECK(d.submeshes[0].backmesh && d.submeshes[0].texcoords.size() == 2 && d.submeshes[0].cabs.size() == 2 &&
                  d.submeshes[0].cabs[0].options == "c",
              "submesh 0 (cab resumes after props)");
        CHECK(!d.submeshes[1].backmesh && d.submeshes[1].cabs.size() == 1, "submesh 1");
    }
    CHECK(d.props.size() == 6, "props %zu", d.props.size());
    if (d.props.size() == 6) {
        const PropDef& p = d.props[0];
        CHECK(p.special == PropDef::DASHBOARD_RH && p.wheel_mesh == "sw.mesh" && p.has_wheel_offset &&
                  p.wheel_offset == vec3(1, 2, 3) && p.wheel_angle == 200,
              "dashboard-rh prop");
        CHECK(d.props[1].special == PropDef::BEACON && d.props[1].beacon_material == "flare_x" &&
                  d.props[1].beacon_color == vec3(0.1f, 0.2f, 0.3f),
              "beacon prop");
        CHECK(d.props[2].special == PropDef::REDBEACON && d.props[3].special == PropDef::SEAT &&
                  d.props[4].special == PropDef::MIRROR_LEFT && d.props[4].camera_mode == 3,
              "special props");
        CHECK(d.props[5].mesh == "modA.mesh", "module cfgA prop appended after root props");
    }
    CHECK(d.flexbodies.size() == 4, "flexbodies %zu", d.flexbodies.size());
    if (d.flexbodies.size() == 4) {
        CHECK(d.flexbodies[0].forset == std::vector<int>({0, 1, 2, 3, 5, 0}) && d.flexbodies[0].camera_mode == -1,
              "forset 0-3, 5,");
        CHECK(d.flexbodies[1].forset == std::vector<int>({1, 2, 50, 51, 52, 53, 54, 55, 6}), "forset1-2-9,50-60,6");
        CHECK(d.flexbodies[2].forset == std::vector<int>({0}) && d.flexbodies[2].has_forset, "lone forset");
        CHECK(d.flexbodies[3].mesh == "bodyroot.mesh" && !d.flexbodies[3].has_forset, "author resets block");
    }
    CHECK(d.managed_materials.size() == 2, "managed materials %zu", d.managed_materials.size());
    if (d.managed_materials.size() == 2) {
        const ManagedMaterialDef& m = d.managed_materials[0];
        CHECK(m.name == "mm1" && m.type == "flexmesh_standard" && m.diffuse == "d.png" && m.damaged_diffuse.empty() &&
                  m.specular == "s.png" && m.double_sided,
              "managed material 0");
    }
    CHECK(d.contacters == std::vector<int>({3}) && d.fixes == std::vector<int>({4}), "contacters / fixes");
    CHECK(d.ropes.size() == 1 && d.ropes[0].options == "i" && d.ties.size() == 1 && d.ties[0].options == "is" &&
              d.ties[0].max_stress == 5000,
          "ropes / ties");
    CHECK(d.configs == std::vector<std::string>({"cfgA", "cfgB"}) && d.selected_config == "cfgA", "configs");

    Document b;
    parse_truck_file(path, b, "cfgB");
    CHECK(b.selected_config == "cfgB" && !b.props.empty() && b.props.back().mesh == "modB.mesh", "select cfgB");

    // expected warnings (by line)
    const int warn_lines[] = {16, 22, 26, 27, 37, 43, 61, 63, 65, 74, 84, 88, 92, 102, 116, 121, 129};
    for (int l : warn_lines) CHECK(has_warning_at(d, l), "expected a warning at line %d", l);
    CHECK(d.warnings.size() == 21, "synthetic: %zu warnings, expected 21", d.warnings.size());
}

// Random mutations of the asset files: must never crash and must always produce valid node references.
void test_fuzz(const std::string& root, const std::vector<std::string>& files) {
    printf("\nfuzz test\n");
    static const char* kInserts[] = {"nodes", "nodes2", "beams", "wheels", "wheels2", "flexbodywheels", "meshwheels2",
                                     "cinecam", "section 0 fz", "end_section", "submesh", "cab", "texcoords", "backmesh",
                                     "forset 0-4000000000,", "forset", "set_beam_defaults -1", "set_node_defaults 1 2 3 4 lx",
                                     "comment", "description", "end", "props", "flexbodies", "axles", "w1(1 2), d(x)",
                                     "detacher_group", "torquecurve", "engine", "minimass", "author", "x, y, z, 1, 2",
                                     "0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0", "9999, 1000, 2000000000, -5, abc"};
    uint64_t seed = 12345;
    auto rnd = [&](uint64_t n) {
        seed = seed * 6364136223846793005ULL + 1442695040888963407ULL;
        return n ? (seed >> 33) % n : 0;
    };
    const char kChars[] = ",:| \t;/-9a\n\r\xff.e";
    int runs = 0, fails_before = g_failures;
    for (const std::string& rel : files) {
        std::string orig;
        read_text_file(path_join(root, rel), orig);
        for (int it = 0; it < 8; it++) {
            std::string t = orig;
            int muts = 1 + (int)rnd(40);
            for (int m = 0; m < muts && !t.empty(); m++) {
                size_t pos = rnd(t.size());
                switch (rnd(4)) {
                case 0: t[pos] = kChars[rnd(sizeof(kChars) - 1)]; break;
                case 1: t.erase(pos, rnd(200)); break;
                case 2: t.insert(pos, std::string("\n") + kInserts[rnd(sizeof(kInserts) / sizeof(kInserts[0]))] + "\n"); break;
                case 3: t.insert(pos, t.substr(rnd(t.size()), rnd(400))); break;
                }
            }
            Document d;
            if (parse_truck_file(temp_file("bl_ror_fuzz.truck", t), d, rnd(2) ? "fz" : "")) validate_structure(d);
            runs++;
        }
    }
    printf("  %d mutated files parsed, %d failures\n", runs, g_failures - fails_before);
}

const char* special_name(PropDef::Special s) {
    static const char* n[] = {"none", "mirrorL", "mirrorR", "dash", "dashRH", "spinprop", "pale", "seat", "beacon",
                              "redbeacon", "lightbar"};
    return n[s];
}

struct Totals {
    long nodes = 0, beams = 0, shocks = 0, hydros = 0, commands = 0, wheels = 0, cabs = 0, flexbodies = 0, props = 0,
         warnings = 0, lines = 0;
};

} // namespace

int main() {
    set_log_quiet(true);
    const std::string root = asset_path("vehicles");
    std::vector<std::string> files;
    for (auto& dir : list_dir(root, false, true)) {
        for (auto& f : list_dir(path_join(root, dir), true, false)) {
            std::string ext = path_ext_lower(f);
            if (ext == ".truck" || ext == ".car" || ext == ".trailer" || ext == ".load")
                files.push_back(path_join(dir, f));
        }
    }
    printf("RoR parser test: %zu files under %s\n\n", files.size(), root.c_str());

    Totals tot;
    double total_ms = 0, worst_ms_per_kline = 0;
    std::string worst_file;
    for (const std::string& rel : files) {
        const std::string path = path_join(root, rel);
        Document d;
        bool ok = parse_truck_file(path, d);
        std::string text;
        read_text_file(path, text);
        long lines = (long)std::count(text.begin(), text.end(), '\n') + 1;

        // timing: best of several runs
        double best = 1e9;
        for (int it = 0; it < 15; it++) {
            Document tmp;
            double t0 = time_seconds();
            parse_truck_file(path, tmp);
            best = std::min(best, (time_seconds() - t0) * 1000.0);
        }
        total_ms += best;
        double per_kline = best * 1000.0 / (double)lines;
        if (per_kline > worst_ms_per_kline) worst_ms_per_kline = per_kline, worst_file = rel;

        int ncine = (int)d.cinecams.size(), nexp = (int)d.nodes_explicit.size();
        int nwheel_nodes = (int)d.nodes.size() - ncine - nexp;
        int wt[5] = {0, 0, 0, 0, 0};
        for (auto& w : d.wheels) wt[w.type]++;
        int cabs = 0, tex = 0;
        for (auto& sm : d.submeshes) cabs += (int)sm.cabs.size(), tex += (int)sm.texcoords.size();
        std::string forsets;
        for (auto& fb : d.flexbodies) forsets += format("%s%zu", forsets.empty() ? "" : ",", fb.forset.size());
        int specials[11] = {};
        for (auto& p : d.props) specials[p.special]++;
        std::string spec;
        for (int s = 1; s < 11; s++)
            if (specials[s]) spec += format(" %s:%d", special_name((PropDef::Special)s), specials[s]);

        printf("%s%s  \"%s\"  (%ld lines, %.3f ms)\n", ok ? "" : "[PARSE FAILED] ", rel.c_str(), d.title.c_str(), lines,
               best);
        printf("  nodes %zu (explicit %d, cinecam %d, wheel %d)  beams %zu  shocks %zu  hydros %zu  commands %zu\n",
               d.nodes.size(), nexp, ncine, nwheel_nodes, d.beams.size(), d.shocks.size(), d.hydros.size(),
               d.commands.size());
        printf("  wheels %zu (wheels %d, wheels2 %d, mesh %d, mesh2 %d, flexbody %d)  submeshes %zu cabs %d texcoords %d\n",
               d.wheels.size(), wt[0], wt[1], wt[2], wt[3], wt[4], d.submeshes.size(), cabs, tex);
        printf("  flexbodies %zu [forset %s]  props %zu%s  managedmats %zu  engine %s gears %zu  axles %zu  "
               "ropes %zu ties %zu fixes %zu contacters %zu\n",
               d.flexbodies.size(), forsets.empty() ? "-" : forsets.c_str(), d.props.size(),
               spec.empty() ? "" : format(" (%s)", spec.c_str() + 1).c_str(), d.managed_materials.size(),
               d.engine.present ? "yes" : "no", d.engine.gears.size(), d.axles.size(), d.ropes.size(), d.ties.size(),
               d.fixes.size(), d.contacters.size());
        if (!d.configs.empty()) {
            std::string c;
            for (auto& s : d.configs) c += (c.empty() ? "" : ", ") + s;
            printf("  configs [%s] selected '%s'\n", c.c_str(), d.selected_config.c_str());
        }
        printf("  mass %.0f/%.0f cab '%s' minimass %.2f  ffv %d  warnings %zu\n", d.dry_mass, d.cargo_mass,
               d.cab_material.c_str(), d.minimass, d.file_format_version, d.warnings.size());
        for (size_t i = 0; i < d.warnings.size() && i < 4; i++) printf("    warn: %s\n", d.warnings[i].c_str());

        validate_structure(d);
        check_quirks(rel, d);

        tot.nodes += (long)d.nodes.size();
        tot.beams += (long)d.beams.size();
        tot.shocks += (long)d.shocks.size();
        tot.hydros += (long)d.hydros.size();
        tot.commands += (long)d.commands.size();
        tot.wheels += (long)d.wheels.size();
        tot.cabs += cabs;
        tot.flexbodies += (long)d.flexbodies.size();
        tot.props += (long)d.props.size();
        tot.warnings += (long)d.warnings.size();
        tot.lines += lines;
    }

    // Module selection: each dodge_viper config swaps the wheel props.
    {
        std::string path = path_join(root, "dodge_viper/ViperGTS.car");
        Document a, b;
        parse_truck_file(path, a);
        parse_truck_file(path, b, "XXR_002");
        CHECK(b.selected_config == "XXR_002" && a.props.size() == b.props.size(), "viper config selection");
        bool differs = false;
        for (size_t i = 0; i < a.props.size() && i < b.props.size(); i++) differs |= a.props[i].mesh != b.props[i].mesh;
        CHECK(differs, "viper: selecting XXR_002 must change prop meshes");
        printf("\nviper configs: default '%s' (%zu props), XXR_002 (%zu props), wheel mesh %s -> %s\n",
               a.selected_config.c_str(), a.props.size(), b.props.size(), a.props.back().mesh.c_str(),
               b.props.back().mesh.c_str());
    }

    test_synthetic();
    test_editor_models();
    test_fuzz(root, files);

    // Title reader must agree with the full parser.
    for (const std::string& rel : files) {
        Document d;
        parse_truck_file(path_join(root, rel), d);
        CHECK(read_truck_title(path_join(root, rel)) == d.title, "read_truck_title mismatch for %s", rel.c_str());
    }

    printf("\nTOTAL %zu files, %ld lines: nodes %ld beams %ld shocks %ld hydros %ld commands %ld wheels %ld cabs %ld "
           "flexbodies %ld props %ld warnings %ld\n",
           files.size(), tot.lines, tot.nodes, tot.beams, tot.shocks, tot.hydros, tot.commands, tot.wheels, tot.cabs,
           tot.flexbodies, tot.props, tot.warnings);
    printf("parse time: %.2f ms total (best of 15), worst %.3f ms per 1000 lines (%s) -> ~%.2f ms for 5000 lines\n",
           total_ms, worst_ms_per_kline, worst_file.c_str(), worst_ms_per_kline * 5);
    size_t shipped = 0; // (the model editor's own files under vehicles/editor are not counted)
    for (const std::string& rel : files) {
        // (the model editor's files: its own folder, and edited copies saved next to the vehicles they came from)
        std::string text;
        read_text_file(path_join(root, rel), text);
        shipped += rel.rfind("editor/", 0) != 0 && text.find(";written by the BeamLab model editor") == std::string::npos;
    }
    CHECK(shipped == 121, "expected 121 vehicle files, found %zu", shipped);
    CHECK(worst_ms_per_kline * 5 < 5.0, "parsing is too slow");
    printf("%s (%d failures)\n", g_failures ? "FAILED" : "ALL CHECKS PASSED", g_failures);
    return g_failures ? 1 : 0;
}
