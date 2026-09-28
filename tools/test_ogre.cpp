// Loads every OGRE .mesh under assets/vehicles and every vehicle folder's material library, and reports
// statistics plus any material referenced by meshes / truck files that doesn't resolve.
// Usage: test_ogre [vehicles_dir] [-q]   (-q: no per-mesh lines)
#include "core/util.h"
#include "vehicle/ogre_material.h"
#include "vehicle/ogre_mesh.h"
#include "vehicle/ror_def.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <map>
#include <set>

using namespace bl;
namespace fs = std::filesystem;

namespace {

std::vector<std::string> list_files_rec(const std::string& dir, const std::set<std::string>& exts) {
    std::vector<std::string> out;
    std::error_code ec;
    for (auto it = fs::recursive_directory_iterator(dir, ec); !ec && it != fs::recursive_directory_iterator(); it.increment(ec))
        if (it->is_regular_file(ec) && exts.count(path_ext_lower(it->path().string()))) out.push_back(it->path().string());
    std::sort(out.begin(), out.end());
    return out;
}

bool is_number(const std::string& s) {
    char* end = nullptr;
    std::strtod(s.c_str(), &end);
    return end != s.c_str() && *end == 0;
}

// Crude truck-file scan: material references + managedmaterials definitions.
struct TruckScan {
    std::vector<std::pair<std::string, std::string>> refs; // material, where
    std::vector<ror::ManagedMaterialDef> managed;
};

void scan_truck(const std::string& path, TruckScan& out) {
    static const std::set<std::string> sections = {
        "globals", "nodes", "nodes2", "beams", "cameras", "cinecam", "engine", "engoption", "engturbo", "brakes",
        "hydros", "commands", "commands2", "shocks", "shocks2", "shocks3", "wheels", "wheels2", "meshwheels",
        "meshwheels2", "flexbodywheels", "managedmaterials", "props", "flexbodies", "submesh", "cab", "texcoords",
        "backmesh", "contacters", "rotators", "rotators2", "triggers", "ties", "ropes", "ropables", "fixes", "flares",
        "flares2", "flares3", "particles", "exhausts", "lockgroups", "slidenodes", "railgroups", "turboprops",
        "turboprops2", "turbojets", "pistonprops", "airbrakes", "wings", "animators", "screwprops", "fusedrag",
        "videocamera", "soundsources", "soundsources2", "guisettings", "minimass", "set_skeleton_settings", "author",
        "fileinfo", "forwardcommands", "importcommands", "rollon", "rescuer", "end", "torquecurve", "cruisecontrol",
        "speedlimiter", "axles", "interaxles", "transfercase", "tractioncontrol", "antilockbrakes", "help", "extcamera",
        "sectionconfig", "disabledefaultsounds", "materialflarebindings", "camerarail", "collisionboxes",
        "submesh_groundmodel", "globeams", "hookgroup", "hooks", "nodecollision", "wheeldetachers", "gui", "section",
        "end_section", "detacher_group", "enable_advanced_deformation", "slidenode_connect_instantly", "lockgroup_default_nolock"};
    std::string text;
    if (!read_text_file(path, text)) return;
    std::string fname = path_filename(path), section, block_end;
    bool double_sided = false;
    int line_no = 0;
    for (auto& raw : split_any(text, "\n", true)) {
        line_no++;
        std::string line = trim(raw);
        if (!block_end.empty()) {
            if (starts_with_ci(line, block_end)) block_end.clear();
            continue;
        }
        if (line.empty() || line[0] == ';' || starts_with_ci(line, "//")) continue;
        size_t sc = line.find(';');
        if (sc != std::string::npos) line = trim(line.substr(0, sc));
        auto t = split_any(line, ", \t");
        if (t.empty()) continue;
        std::string kw = to_lower(t[0]);
        std::string where = format("%s:%d %s", fname.c_str(), line_no, section.c_str());
        if (kw == "comment") { block_end = "end_comment"; continue; }
        if (kw == "description") { block_end = "end_description"; continue; }
        if (kw == "set_managedmaterials_options") { double_sided = t.size() > 1 && t[1] == "1"; continue; }
        if (kw == "set_beam_defaults") {
            if (t.size() > 6 && !is_number(t[6])) out.refs.push_back({t[6], format("%s:%d set_beam_defaults", fname.c_str(), line_no)});
            continue;
        }
        // a managed material may be named like a keyword ("engine  mesh_standard  engine.dds")
        const bool managed_line = section == "managedmaterials" && t.size() >= 3 &&
                                  (starts_with_ci(t[1], "mesh_") || starts_with_ci(t[1], "flexmesh_"));
        if (sections.count(kw) && !managed_line) { section = kw; continue; }
        if (starts_with_ci(kw, "set_") || starts_with_ci(kw, "add_")) continue;
        if (section == "globals" && t.size() >= 3) {
            out.refs.push_back({t[2], where});
        } else if ((section == "wheels" && t.size() >= 14) || (section == "wheels2" && t.size() >= 17)) {
            out.refs.push_back({t[t.size() - 2], where + " face"});
            out.refs.push_back({t.back(), where + " band"});
        } else if ((section == "meshwheels" || section == "meshwheels2") && t.size() >= 16) {
            out.refs.push_back({t.back(), where + " tyre"});
        } else if (section == "managedmaterials" && t.size() >= 3) {
            ror::ManagedMaterialDef d;
            d.name = t[0];
            d.type = t[1];
            d.diffuse = t[2];
            if (starts_with_ci(d.type, "flexmesh")) {
                if (t.size() > 3) d.damaged_diffuse = t[3];
                if (t.size() > 4) d.specular = t[4];
            } else if (t.size() > 3) {
                d.specular = t[3];
            }
            d.double_sided = double_sided;
            out.managed.push_back(d);
        }
    }
}

// Names exporters write when an object had no (real) material assigned.
bool is_placeholder_name(const std::string& name) {
    std::string l = to_lower(name);
    if (l == "material" || l == "_missing_material_" || l == "defaultmaterial" || l == "default" || l == "none") return true;
    if (l.size() >= 15 && l.compare(l.size() - 15, 15, "sketchupdefault") == 0) return true;
    if (starts_with_ci(l, "material.") || starts_with_ci(l, "material_")) return true;
    if (starts_with_ci(l, "color_") && l.size() > 6 && isdigit((unsigned char)l[6])) return true;
    return false;
}

} // namespace

int main(int argc, char** argv) {
    std::string root = asset_path("vehicles");
    bool quiet = false;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-q")) quiet = true;
        else root = argv[i];
    }
    set_log_quiet(true);

    double t0 = time_seconds();
    int n_meshes = 0, n_failed = 0, n_suspicious = 0;
    size_t tot_sub = 0, tot_vert = 0, tot_tri = 0, no_normals = 0, no_uvs = 0;
    std::map<std::string, int> versions;
    double mesh_time = 0, mat_time = 0;
    int mats_ref_total = 0, mats_unres_total = 0, mats_notex_total = 0;
    std::vector<std::string> summary;

    for (auto& veh : list_dir(root, false, true)) {
        std::string dir = path_join(root, veh);
        printf("\n==================== %s\n", veh.c_str());

        // materials
        double tm = time_seconds();
        MaterialLibrary lib;
        lib.load_dir(dir);
        TruckScan ts;
        auto trucks = list_files_rec(dir, {".truck", ".car", ".trailer", ".load", ".boat", ".airplane", ".train", ".fixed", ".machine"});
        std::string trucks_text; // lower-case concatenation, to tell used meshes from leftovers
        for (auto& tf : trucks) {
            scan_truck(tf, ts);
            std::string t;
            if (read_text_file(tf, t)) trucks_text += to_lower(t);
        }
        std::set<std::string> managed_seen;
        for (auto& d : ts.managed)
            if (managed_seen.insert(d.name).second) lib.add_managed(d, dir);
        lib.add_ror_builtins();
        mat_time += time_seconds() - tm;

        // meshes
        std::map<std::string, std::set<std::string>> refs; // material -> where
        std::set<std::string> used_by_truck;               // materials referenced by a mesh some truck file uses, or by a truck file
        for (auto& mp : list_files_rec(dir, {".mesh"})) {
            n_meshes++;
            OgreMesh m;
            std::string err;
            double t1 = time_seconds();
            bool ok = load_ogre_mesh(mp, m, &err);
            mesh_time += time_seconds() - t1;
            std::string rel = mp.substr(dir.size() + 1);
            if (!ok) {
                n_failed++;
                printf("  FAIL %-40s %s\n", rel.c_str(), err.c_str());
                continue;
            }
            versions[m.version]++;
            size_t nv = 0, nt = 0;
            bool all_n = true, all_uv = true;
            std::set<std::string> mats;
            for (auto& s : m.submeshes) {
                nv += s.vertices.size();
                nt += s.indices.size() / 3;
                all_n &= s.has_normals;
                all_uv &= s.has_uvs;
                mats.insert(s.material);
                refs[s.material].insert(rel);
                if (trucks_text.find(to_lower(path_filename(mp))) != std::string::npos) used_by_truck.insert(s.material);
            }
            tot_sub += m.submeshes.size(), tot_vert += nv, tot_tri += nt;
            no_normals += !all_n, no_uvs += !all_uv;
            vec3 e = m.bounds.valid() ? m.bounds.extent() : vec3(0.0f);
            bool sus = !m.bounds.valid() || !std::isfinite(maxc(e)) || maxc(e) > 25.0f || nt == 0;
            n_suspicious += sus;
            if (!quiet || sus) {
                std::string ml;
                for (auto& s : mats) ml += (ml.empty() ? "" : ", ") + s;
                printf("  %s%-40s v%-5s sub %2zu  vert %6zu  tri %6zu  n%c uv%c  bounds (%.2f %.2f %.2f)..(%.2f %.2f %.2f)  [%s]\n",
                       sus ? "?? " : "", rel.c_str(), m.version.c_str(), m.submeshes.size(), nv, nt, all_n ? '+' : '-',
                       all_uv ? '+' : '-', m.bounds.mn.x, m.bounds.mn.y, m.bounds.mn.z, m.bounds.mx.x, m.bounds.mx.y,
                       m.bounds.mx.z, ml.c_str());
            }
        }
        for (auto& r : ts.refs) {
            refs[r.first].insert(r.second);
            used_by_truck.insert(r.first);
        }

        // material report
        int n_ref = 0, n_unres = 0, n_notex = 0;
        std::vector<std::string> lines;
        for (auto& [name, where] : refs) {
            n_ref++;
            const MaterialDesc* d = lib.find(name);
            std::string w = *where.begin() + (where.size() > 1 ? format(" (+%zu more)", where.size() - 1) : "");
            if (!d) {
                n_unres++;
                const char* why = !used_by_truck.count(name) ? "only in leftover meshes no truck file uses"
                                  : is_placeholder_name(name) ? "exporter placeholder name, not defined by the mod"
                                                              : "not defined by the mod's .material/managedmaterials nor RoR built-ins";
                lines.push_back(format("    UNRESOLVED %-40s used by %s -- %s", name.c_str(), w.c_str(), why));
                continue;
            }
            bool tex_missing = false;
            for (auto& wm : lib.warnings)
                if (wm.find("'" + name + "'") != std::string::npos && wm.find("not found") != std::string::npos) tex_missing = true;
            if (tex_missing) {
                n_notex++;
                lines.push_back(format("    NO-TEXTURE %-50s used by %s", name.c_str(), w.c_str()));
            } else if (!quiet) {
                std::string flags;
                if (d->invisible) flags += " invisible";
                if (d->alpha_test) flags += " alpha_test";
                if (d->blend) flags += " blend";
                if (d->double_sided) flags += " 2sided";
                if (!d->lighting) flags += " unlit";
                if (d->reflective) flags += " reflective";
                if (!d->emissive_tex.empty()) flags += " emissive_tex=" + path_filename(d->emissive_tex);
                std::string tex = d->diffuse_tex.empty() ? format("colour(%.2f %.2f %.2f)", d->diffuse.x, d->diffuse.y, d->diffuse.z)
                                                         : path_filename(d->diffuse_tex);
                lines.push_back(format("    ok         %-50s %s%s", name.c_str(), tex.c_str(), flags.c_str()));
            }
        }
        printf("  materials: %zu defined, %d referenced, %d unresolved, %d with missing texture\n", lib.size(), n_ref, n_unres, n_notex);
        for (auto& l : lines) printf("%s\n", l.c_str());
        if (!lib.warnings.empty()) {
            printf("  library warnings (%zu):\n", lib.warnings.size());
            for (auto& w : lib.warnings) printf("    - %s\n", w.c_str());
        }
        mats_ref_total += n_ref, mats_unres_total += n_unres, mats_notex_total += n_notex;
        summary.push_back(format("  %-22s materials %3zu  referenced %3d  unresolved %3d  missing-texture %3d", veh.c_str(), lib.size(), n_ref,
                                 n_unres, n_notex));
    }

    printf("\n==================== summary\n");
    for (auto& s : summary) printf("%s\n", s.c_str());
    printf("meshes: %d loaded, %d failed, %d suspicious; submeshes %zu, vertices %zu, triangles %zu\n", n_meshes - n_failed,
           n_failed, n_suspicious, tot_sub, tot_vert, tot_tri);
    printf("meshes without (all) normals: %zu, without (all) uvs: %zu\n", no_normals, no_uvs);
    printf("versions:");
    for (auto& [v, c] : versions) printf("  v%s x%d", v.c_str(), c);
    printf("\nmaterials: %d referenced, %d unresolved, %d missing texture\n", mats_ref_total, mats_unres_total, mats_notex_total);
    printf("time: total %.2fs (meshes %.2fs, materials %.2fs)\n", time_seconds() - t0, mesh_time, mat_time);
    return n_failed ? 1 : 0;
}
