// Scene registry: the worlds of the World menu, by section.
#include "core/util.h"
#include "game/game.h"
#include "phys/fem_shell.h"
#include "vehicle/vehicle.h"
#include "world/ai.h"
#include "world/scenes_internal.h"
#include "world/stage.h"

#include <algorithm>
#include <cmath>

namespace bl {

using namespace phys;
namespace te = terrain_edit;


std::vector<SceneInfo> base_scenes();

// The converted RBR stages (tools/fetch_rbr_stage.py): the ones the script knows, installed or not (a stage that is not
// says how to get it), then whatever else lies in assets/stages.
void add_rbr_stages(std::vector<SceneInfo>& s) {
    struct Known {
        const char *id, *dir, *name, *what;
    };
    static const Known known[] = {
        {"rbr_verkiai", "verkiai_sss", "Verkiai SSS", "Lithuanian super special in a park, tarmac and gravel"},
        {"rbr_zarasai", "zarasai", "Zarasai island", "a super special round an island, two laps, 2 km"},
        {"rbr_haguenau", "haguenau", "Haguenau", "tarmac, 5.7 km"},
        {"rbr_junior_wheels", "junior_wheels_2", "Junior Wheels II", "a technical training stage of many road types, 5.6 km"},
        {"rbr_undva", "undva", "Undva", "narrow fast Estonian gravel through forest, 10 km"},
        {"rbr_undva_reverse", "undva_reverse", "Undva reverse", "Undva the other way"},
        {"rbr_travanca", "travanca", "Travanca do Monte", "Portuguese gravel in wooded hills"},
        {"rbr_fernet", "fernet_branca", "Fernet Branca", "Argentine gravel through the hills"},
    };
    std::vector<std::string> dirs = list_dir(asset_path("stages"), false, true);
    for (const Known& k : known) {
        const bool have = std::find(dirs.begin(), dirs.end(), k.dir) != dirs.end();
        dirs.erase(std::remove(dirs.begin(), dirs.end(), std::string(k.dir)), dirs.end());
        const std::string dir = k.dir;
        s.push_back({k.id, k.name, "Rally/RBR stages", std::string("RBR community stage (RALLY Guru): ") + k.what + (have ? "" : " - not installed (tools/fetch_rbr_stage.py)"),
                     [dir](Game& g) { scene_rbr_stage(g, dir); }});
    }
    for (const std::string& dir : dirs) {
        if (!file_exists(asset_path("stages/" + dir + "/stage.bin"))) continue;
        s.push_back({"rbr_" + dir, dir, "Rally/RBR stages", "A converted RBR stage (assets/stages/" + dir + ")", [dir](Game& g) { scene_rbr_stage(g, dir); }});
    }
}

const std::vector<SceneInfo>& scene_registry() {
    static const std::vector<SceneInfo> s = [] {
        std::vector<SceneInfo> r = base_scenes();
        const auto at = std::find_if(r.begin(), r.end(), [](const SceneInfo& x) { return x.id == "crash"; });
        std::vector<SceneInfo> rbr;
        add_rbr_stages(rbr);
        r.insert(at, rbr.begin(), rbr.end());
        return r;
    }();
    return s;
}

std::vector<SceneInfo> base_scenes() {
    std::vector<SceneInfo> s = {
        {"proving", "Proving Ground", "Drive", "Vehicle handling test course", scene_proving_ground},
        {"test_site", "Test Site", "Drive", "Vehicle test site: oval, crash lanes, press and vise, ride lanes, off-road park, circuit, city block", scene_test_site},
        {"forest", "Forest", "Drive", "Hills with trees and bushes", scene_forest},
        {"offroad", "Offroad Trail", "Drive", "Rocks, mud, sand, logs", scene_offroad},
        {"canyon", "Canyon Bridges", "Drive", "Breakable beam bridges over a chasm", scene_canyon},
        {"tape_maze", "Tape Maze", "Drive", "Gymkhana course marked with tape: lanes, hairpins, chicane, timer", scene_tape_maze},
        {"rally", "Rally Stage", "Rally", "Gravel stage: trees, tape, hay bales, crests, timer", scene_rally},
        {"crash", "Crash Test", "Crash", "Wall, barrier, poles", scene_crash},
        {"vehicle_crash", "Vehicle vs Vehicle", "Crash", "Configurable crash of two vehicles (Scene menu)", scene_vehicle_crash},
        {"frame_car", "Frame Car", "FEM cars/Frame Car", "A car on a welded space frame of FEM tubes with sheet panels: drive it, drop it, roll it, crash it",
         scene_frame_car},
        {"fc_headon", "Frame Car: Head-on", "FEM cars/Frame Car", "Two Frame Cars head-on at 50 km/h each",
         [](Game& g) { scene_frame_car_test(g, "Head-on into another", vec3(4.6f, 1.9f, 57.5f), vec3(0, 0.6f, 62.5f), "Two Frame Cars meet head-on at 50 km/h each."); }},
        {"fc_side", "Frame Car: Side impact", "FEM cars/Frame Car", "Another Frame Car into its side at 50 km/h",
         [](Game& g) {
             scene_frame_car_test(g, "into its side", vec3(6.5f, 2.8f, 53.5f), vec3(-1.5f, 0.7f, 60), "A second Frame Car hits the standing one in the side at 50 km/h.");
         }},
        {"fc_wall", "Frame Car: Wall", "FEM cars/Frame Car", "Into a concrete wall at 60 km/h",
         [](Game& g) { scene_frame_car_test(g, "Launch at the wall", vec3(6.5f, 2.4f, 139.5f), vec3(0, 0.7f, 146), "The Frame Car into a concrete wall at 60 km/h."); }},
        {"fc_pole", "Frame Car: Pole", "FEM cars/Frame Car", "Into a concrete pole at 50 km/h",
         [](Game& g) { scene_frame_car_test(g, "Launch at the pole", vec3(22, 2.4f, 141), vec3(16, 0.7f, 147), "The Frame Car into a concrete pole at 50 km/h."); }},
        {"fc_drop", "Frame Car: Drop 10 m", "FEM cars/Frame Car", "Dropped on its wheels from 10 m",
         [](Game& g) { scene_frame_car_test(g, "Drop from 10", vec3(8, 4.5f, 12), vec3(0, 1.8f, 20), "The Frame Car dropped on its wheels from 10 m."); }},
        {"fc_slab", "Frame Car: Slab", "FEM cars/Frame Car", "A 5 t concrete slab dropped on it from 2.5 m",
         [](Game& g) { scene_frame_car_test(g, "Drop a 5 t concrete slab", vec3(6, 3.6f, 14.5f), vec3(0, 1.2f, 20), "A 5 t concrete slab falls on the roof from 2.5 m."); }},
        {"fc_axe", "Frame Car: Giant axe", "FEM cars/Frame Car", "A 5 t pendulum axe swings down and cuts it in two",
         [](Game& g) { scene_frame_car_test(g, "The giant axe", vec3(-15.5f, 5.0f, 110.5f), vec3(-25, 3.0f, 120), "A 5 t axe on a pendulum swings down and cuts the car in two."); }},
        {"fc_roll", "Frame Car: Barrel roll", "FEM cars/Frame Car", "Thrown up and spun at 50 km/h: it rolls over",
         [](Game& g) { scene_frame_car_test(g, "Barrel roll", vec3(9, 4, 18), vec3(0, 1, 32), "Thrown up and spun at 50 km/h: the Frame Car rolls over."); }},
        {"buggy", "Buggy", "FEM cars/Buggy", "A desert racer on a welded tube cage (FEM) with aluminium panels: whoops, a jump, drops, rolls, crashes",
         scene_buggy},
        {"bg_headon", "Buggy: Head-on", "FEM cars/Buggy", "Two Buggies head-on at 50 km/h each",
         [](Game& g) { scene_frame_car_test(g, "Head-on into another", vec3(4.6f, 1.9f, 57.5f), vec3(0, 0.6f, 62.5f), "Two Buggies meet head-on at 50 km/h each.", scene_buggy); }},
        {"bg_side", "Buggy: Side impact", "FEM cars/Buggy", "Another Buggy into its side at 50 km/h",
         [](Game& g) {
             scene_frame_car_test(g, "into its side", vec3(6.5f, 2.8f, 53.5f), vec3(-1.5f, 0.7f, 60), "A second Buggy hits the standing one in the side at 50 km/h.", scene_buggy);
         }},
        {"bg_wall", "Buggy: Wall", "FEM cars/Buggy", "Into a concrete wall at 60 km/h",
         [](Game& g) { scene_frame_car_test(g, "Launch at the wall", vec3(6.5f, 2.4f, 139.5f), vec3(0, 0.7f, 146), "The Buggy into a concrete wall at 60 km/h.", scene_buggy); }},
        {"bg_drop", "Buggy: Drop 10 m", "FEM cars/Buggy", "Dropped on its wheels from 10 m",
         [](Game& g) { scene_frame_car_test(g, "Drop from 10", vec3(8, 4.5f, 12), vec3(0, 1.8f, 20), "The Buggy dropped on its wheels from 10 m.", scene_buggy); }},
        {"bg_slab", "Buggy: Slab", "FEM cars/Buggy", "A 5 t concrete slab dropped on it from 2.5 m",
         [](Game& g) { scene_frame_car_test(g, "Drop a 5 t concrete slab", vec3(6, 3.9f, 14.5f), vec3(0, 1.4f, 20), "A 5 t concrete slab falls on the cage from 2.5 m.", scene_buggy); }},
        {"bg_roll", "Buggy: Barrel roll", "FEM cars/Buggy", "Thrown up and spun at 50 km/h: it rolls over",
         [](Game& g) { scene_frame_car_test(g, "Barrel roll", vec3(9, 4, 18), vec3(0, 1, 32), "Thrown up and spun at 50 km/h: the Buggy rolls over.", scene_buggy); }},
        {"bg_whoops", "Buggy: Whoops", "FEM cars/Buggy", "Through the whoops at 80 km/h",
         [](Game& g) { scene_frame_car_test(g, "whoops", vec3(88, 2.5f, 30), vec3(80, 0.8f, 45), "Through ten 0.5 m whoops at 80 km/h.", scene_buggy); }},
        {"bg_jump", "Buggy: Jump", "FEM cars/Buggy", "Over the tabletop jump at 90 km/h",
         [](Game& g) { scene_frame_car_test(g, "jump", vec3(95, 4.0f, 160), vec3(80, 2.2f, 172), "Over the 2.2 m tabletop at 90 km/h.", scene_buggy); }},
        {"shell_car", "Shell Car", "FEM cars/Shell Car", "A saloon on the BMW E36's lines, all FEM: a body-in-white of members and sheets, parts on hinges, bolts, buffers",
         scene_shell_car},
        {"shell_car_m3", "Shell Car M3", "FEM cars/Shell Car", "The Shell Car under the BMW M3's (the E36 Lightweight's) graphical model: its meshes skinned to the FEM parts' nodes",
         scene_shell_car_m3},
        {"sc_headon", "Shell Car: Head-on", "FEM cars/Shell Car", "Two Shell Cars head-on at 50 km/h each",
         [](Game& g) { scene_frame_car_test(g, "Head-on into another", vec3(5.2f, 2.0f, 56.5f), vec3(0, 0.6f, 62.5f), "Two Shell Cars meet head-on at 50 km/h each.", scene_shell_car); }},
        {"sc_side", "Shell Car: Side impact", "FEM cars/Shell Car", "Another Shell Car into its side at 50 km/h",
         [](Game& g) {
             scene_frame_car_test(g, "into its side", vec3(7.0f, 3.0f, 53.0f), vec3(-1.5f, 0.7f, 60), "A second Shell Car hits the standing one in the side at 50 km/h.", scene_shell_car);
         }},
        {"sc_wall", "Shell Car: Wall", "FEM cars/Shell Car", "Into a concrete wall at 60 km/h",
         [](Game& g) { scene_frame_car_test(g, "Launch at the wall", vec3(7.0f, 2.5f, 139.0f), vec3(0, 0.7f, 146), "The Shell Car into a concrete wall at 60 km/h.", scene_shell_car); }},
        {"sc_pole", "Shell Car: Pole", "FEM cars/Shell Car", "Into a concrete pole at 50 km/h",
         [](Game& g) { scene_frame_car_test(g, "Launch at the pole", vec3(22.5f, 2.5f, 140.5f), vec3(16, 0.7f, 147), "The Shell Car into a concrete pole at 50 km/h.", scene_shell_car); }},
        {"sc_drop", "Shell Car: Drop 10 m", "FEM cars/Shell Car", "Dropped on its wheels from 10 m",
         [](Game& g) { scene_frame_car_test(g, "Drop from 10", vec3(8.5f, 4.5f, 12), vec3(0, 1.8f, 20), "The Shell Car dropped on its wheels from 10 m.", scene_shell_car); }},
        {"sc_roof", "Shell Car: On the roof", "FEM cars/Shell Car", "Turned over and dropped on its roof from 1.5 m",
         [](Game& g) { scene_frame_car_test(g, "Drop on the roof", vec3(8, 3.0f, 13), vec3(0, 0.8f, 20), "The Shell Car turned over and dropped on its roof from 1.5 m.", scene_shell_car); }},
        {"sc_side_lay", "Shell Car: On its side", "FEM cars/Shell Car", "Laid on its side from 0.3 m: the doors rest on their openings, the wheels keep their toe",
         [](Game& g) { scene_frame_car_test(g, "Lay it on its side", vec3(0, 2.2f, 13), vec3(0, 0.8f, 20), "The Shell Car laid on its side from 0.3 m.", scene_shell_car); }},
        {"sc_slab", "Shell Car: Slab", "FEM cars/Shell Car", "A 5 t concrete slab dropped on it from 2.5 m",
         [](Game& g) { scene_frame_car_test(g, "Drop a 5 t concrete slab", vec3(6.5f, 3.8f, 14), vec3(0, 1.2f, 20), "A 5 t concrete slab falls on the roof from 2.5 m.", scene_shell_car); }},
        {"sc_axe", "Shell Car: Giant axe", "FEM cars/Shell Car", "A 5 t pendulum axe swings down and cuts it in two",
         [](Game& g) { scene_frame_car_test(g, "The giant axe", vec3(-15.5f, 5.0f, 110.0f), vec3(-25, 3.0f, 120), "A 5 t axe on a pendulum swings down and cuts the car in two.", scene_shell_car); }},
        {"sc_roll", "Shell Car: Barrel roll", "FEM cars/Shell Car", "Thrown up and spun at 50 km/h: it rolls over",
         [](Game& g) { scene_frame_car_test(g, "Barrel roll", vec3(9.5f, 4, 18), vec3(0, 1, 32), "Thrown up and spun at 50 km/h: the Shell Car rolls over.", scene_shell_car); }},
        {"sc_curb", "Shell Car: Curb", "FEM cars/Shell Car", "Sideways into a curb at 40 km/h",
         [](Game& g) { scene_frame_car_test(g, "Trip over the curb", vec3(-32, 3.0f, 51), vec3(-41, 0.8f, 60), "The Shell Car slides sideways into a curb at 40 km/h.", scene_shell_car); }},
        {"sc_whoops", "Shell Car: Whoops", "FEM cars/Shell Car", "Through the whoops at 80 km/h",
         [](Game& g) { scene_frame_car_test(g, "whoops", vec3(88, 2.5f, 30), vec3(80, 0.8f, 45), "Through ten 0.5 m whoops at 80 km/h.", scene_shell_car); }},
        {"sc_jump", "Shell Car: Jump", "FEM cars/Shell Car", "Over the tabletop jump at 90 km/h",
         [](Game& g) { scene_frame_car_test(g, "jump", vec3(95, 4.0f, 160), vec3(80, 2.2f, 172), "Over the 2.2 m tabletop at 90 km/h.", scene_shell_car); }},
        {"sc_latches", "Shell Car: Latches let go", "FEM cars/Shell Car", "The hood's, the lid's and the doors' latches let go: they rest on their buffers and stays",
         [](Game& g) { scene_frame_car_test(g, "Let the latches go", vec3(7, 2.6f, -5.5f), vec3(0, 0.8f, 0), "The latches let go: the hood and the lid rest on their buffers.", scene_shell_car); }},
        {"sheet_car", "Sheet Car", "FEM cars", "A car whose body panels are a steel sheet of triangle elements: crash it into a parked one, poles or a wall",
         scene_sheet_car},
        {"lab", "Primitives", "Physics lab", "Primitive tests: collisions, deformation, breaking, joints", scene_lab},
        {"materials", "Materials Lab", "Physics lab", "Sheets, shapes and bars of 10 materials: drop, press and bending tests, drive-through panels",
         scene_materials},
        {"sheet_shapes", "Sheet Shapes", "Physics lab", "Sheets of other shapes and sizes: steel gate 6 x 4 m, half-pipe, disc, ring, triangle, L, dome",
         scene_sheet_shapes},
        {"sheet_run", "Sheet Run", "Physics lab", "Drive through three lead sheets clamped in gates (profiling of sheet fracture)", scene_sheet_run},
        {"barrels", "Steel Barrels", "Physics lab", "200 l steel drums of triangle elements: drop, roll, stack and crash them", scene_barrels},
        {"fem_shells", "FEM Shells", "Physics lab", "Triangle elements of the FEM frame: a steel sheet, a cantilever and a hollow cube", scene_fem_shells},
        {"stress_vehicles", "Stress: 16 vehicles", "Stress", "Many AI vehicles", scene_stress_vehicles},
        {"stress_derby", "Stress: Demolition derby", "Stress", "12 AI vehicles colliding", scene_stress_derby},
        {"stress_crates", "Stress: 512 crates", "Stress", "Pile of soft boxes", scene_stress_crates},
        {"stress_forest", "Stress: Windy forest", "Stress", "400 trees in wind", scene_stress_forest},
        {"stress_bridge", "Stress: Bridge convoy", "Stress", "Heavy trucks break the bridge", scene_stress_bridge},
        {"stress_barrels", "Stress: Barrel pile", "Stress", "15 steel barrels in a pile, three more thrown into it", scene_stress_barrels},
        {"editor", "Model Editor", "Tools", "The model editor's stage: a flat square, a neutral background (Editor menu, Ctrl+E)", scene_editor},
    };
    return s;
}

} // namespace bl
