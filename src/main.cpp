// BeamLab: soft-body vehicle physics prototype (Rigs of Rods style) with own renderer and physics.
#include "core/util.h"
#include "game/app.h"

#include <cstdlib>
#include <cstring>
#include <string>

static void usage() {
    printf("usage: beamlab [options]\n"
           "  --scene <id|index>        start scene (proving, forest, canyon, offroad, crash, lab, stress_*)\n"
           "  --vehicle <id>            vehicle id (folder/file stem)\n"
           "  --bench <frames>          benchmark: run N frames at fixed dt, print timings, exit\n"
           "  --screenshot <file.png>   render and save a screenshot, then exit\n"
           "  --frames <n>              frames before the screenshot (default 120)\n"
           "  --camera <preset>         chase|front|side|rear|top|orbit:<yaw>,<pitch>,<dist>\n"
           "  --size <w>x<h>            window size (default: maximized)\n"
           "  --editor                  open the model editor after the load\n"
           "  --hidden                  hidden window (offscreen)\n"
           "  --threads <n>             worker threads incl. main (default: performance cores)\n"
           "  --beams                   start with the beam debug view\n"
           "  --spawn <x,z,yaw>         override the vehicle spawn point\n"
           "  --perf                    open the performance widget\n"
           "  --drive <thr,steer[,brk]> autodrive test input for the player vehicle (prints telemetry)\n"
           "  --action <text>           run the Scene menu action whose label contains <text> (e.g. Autopilot)\n"
           "  --shoot k,speed,interval  --destroy radius   scripted tools along the camera ray\n"
           "  --crane lift[,frame[,roll,pitch]]  hang the vehicle by its top, tilted (degrees; released at that frame, -1: never)\n"
           "  --laser x0,y0,x1,y1[,frame,frames]            scripted laser sweep across the screen (fractions)\n"
           "  --list                    list scenes and vehicles\n");
}

int main(int argc, char** argv) {
    bl::AppOptions opt;
    bool list = false;
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        auto next = [&]() -> std::string { return i + 1 < argc ? argv[++i] : ""; };
        if (a == "--scene") opt.scene = next();
        else if (a == "--vehicle") opt.vehicle = next();
        else if (a == "--bench") opt.bench_frames = atoi(next().c_str());
        else if (a == "--screenshot") opt.screenshot = next();
        else if (a == "--shots") opt.shots = next();
        else if (a == "--realtime") opt.realtime = true;
        else if (a == "--frames") opt.screenshot_frames = atoi(next().c_str());
        else if (a == "--camera") opt.camera = next();
        else if (a == "--editor") opt.editor = true;
        else if (a == "--editor-test") opt.editor = opt.editor_test = true;
        else if (a == "--size") opt.size_given = sscanf(next().c_str(), "%dx%d", &opt.width, &opt.height) == 2;
        else if (a == "--hidden") opt.hidden = true;
        else if (a == "--threads") opt.threads = atoi(next().c_str());
        else if (a == "--beams") opt.debug_beams = true;
        else if (a == "--novsync") opt.no_vsync = true;
        else if (a == "--timescale") opt.time_scale = (float)atof(next().c_str());
        else if (a == "--drive") opt.drive = next();
        else if (a == "--spawn") opt.spawn = next();
        else if (a == "--perf") opt.show_perf = true;
        else if (a == "--shoot") opt.autoshoot = next();
        else if (a == "--crane") opt.crane = next();
        else if (a == "--destroy") opt.autodestroy = (float)atof(next().c_str());
        else if (a == "--laser") opt.laser = next();
        else if (a == "--action") opt.action = next();
        else if (a == "--launch") opt.launch_kmh = (float)atof(next().c_str());
        else if (a == "--record") opt.record = next();
        else if (a == "--noui") opt.no_ui = true;
        else if (a == "--list") list = true;
        else if (a == "--help" || a == "-h") {
            usage();
            return 0;
        }
    }
    if (list) {
        extern void beamlab_print_lists();
        beamlab_print_lists();
        return 0;
    }
    bl::App app;
    return app.run(opt);
}
