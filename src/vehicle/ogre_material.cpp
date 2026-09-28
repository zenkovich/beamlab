// OGRE material scripts -> MaterialDesc.
// Scripts are parsed into a small statement tree, inheritance (`material A : B`) is resolved by overlaying the
// child on its parent like OGRE's script compiler does, `set $var` / `set_texture_alias` substitutions are
// applied, and the first usable technique is reduced to the handful of properties a forward renderer needs.
#include "vehicle/ogre_material.h"

#include "core/util.h"
#include "vehicle/ror_def.h"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <map>
#include <unordered_set>

namespace bl {

namespace {

// ------------------------------------------------------------------ script tree

struct Node {
    std::vector<std::string> args; // args[0] = keyword
    std::string key;               // lower-case keyword
    std::vector<Node> kids;
    bool block = false;            // followed by a { } block
    int line = 0;
    const std::string& name() const { static const std::string e; return args.size() > 1 ? args[1] : e; }
};

// Tokenizes and parses one script. Newlines end statements, a '{' attaches a block to the preceding statement.
Node parse_script(const std::string& text, const std::string& file, std::vector<std::string>& warnings) {
    Node root;
    root.block = true;
    std::vector<Node*> st{&root};
    std::vector<std::string> cur;
    int line = 1, cur_line = 1, ignored = 0; // ignored: blocks nested deeper than kMaxDepth
    constexpr size_t kMaxDepth = 32;
    Node* last = nullptr; // last statement of the current block (may own a following '{')
    auto flush = [&] {
        if (cur.empty()) return;
        Node n;
        n.args = std::move(cur);
        n.key = to_lower(n.args[0]);
        n.line = cur_line;
        st.back()->kids.push_back(std::move(n));
        last = &st.back()->kids.back();
        cur.clear();
    };
    auto add_token = [&](std::string t) {
        if (cur.empty()) cur_line = line;
        cur.push_back(std::move(t));
    };
    size_t i = 0, n = text.size();
    while (i < n) {
        char c = text[i];
        if (c == '\n') {
            flush();
            line++, i++;
        } else if (c == '/' && i + 1 < n && text[i + 1] == '/') {
            while (i < n && text[i] != '\n') i++;
        } else if (c == '/' && i + 1 < n && text[i + 1] == '*') {
            for (i += 2; i < n && !(text[i] == '*' && i + 1 < n && text[i + 1] == '/'); i++)
                if (text[i] == '\n') line++;
            i += 2;
        } else if ((unsigned char)c <= ' ') {
            i++;
        } else if (c == '{') {
            flush();
            if (ignored || st.size() >= kMaxDepth) { // garbage input: keep the tree (and recursion) shallow
                ignored++, i++;
                continue;
            }
            Node* owner = last;
            if (!owner || owner->block) {
                st.back()->kids.emplace_back();
                owner = &st.back()->kids.back();
                owner->line = line;
            }
            if (owner->key == "material" && st.size() > 1) {
                // A material can't be nested: the previous one is missing a '}'. Close it (OGRE would drop the rest).
                warnings.push_back(format("%s:%d: missing '}' before material '%s'", path_filename(file).c_str(), line,
                                          owner->name().c_str()));
                Node moved = std::move(*owner);
                st.back()->kids.pop_back();
                st.resize(1);
                root.kids.push_back(std::move(moved));
                owner = &root.kids.back();
            }
            owner->block = true;
            st.push_back(owner);
            last = nullptr;
            i++;
        } else if (c == '}') {
            flush();
            if (ignored) ignored--;
            else if (st.size() > 1) st.pop_back();
            last = nullptr;
            i++;
        } else if (c == '"') {
            size_t b = ++i;
            while (i < n && text[i] != '"' && text[i] != '\n') i++;
            add_token(text.substr(b, i - b));
            if (i < n && text[i] == '"') i++;
        } else if (c == ':') {
            add_token(":");
            i++;
        } else {
            size_t b = i;
            while (i < n && (unsigned char)text[i] > ' ' && text[i] != '{' && text[i] != '}' && text[i] != '"' &&
                   text[i] != ':')
                i++;
            add_token(text.substr(b, i - b));
        }
    }
    flush();
    return root;
}

const Node* find_kid(const Node& n, const char* key) {
    for (auto& k : n.kids)
        if (k.key == key) return &k;
    return nullptr;
}

// OGRE-style inheritance: properties of the base come first (so the child's win), child objects override the
// base object with the same class and name, unnamed objects are matched by position.
Node overlay(const Node& base, const Node& child) {
    Node out = base;
    out.args = child.args;
    out.key = child.key;
    out.line = child.line;
    std::map<std::string, int> unnamed_seen;
    for (auto& c : child.kids) {
        Node* match = nullptr;
        if (c.block) {
            if (!c.name().empty()) {
                for (auto& k : out.kids)
                    if (k.block && k.key == c.key && k.name() == c.name()) { match = &k; break; }
            } else {
                int want = unnamed_seen[c.key]++, idx = 0;
                for (auto& k : out.kids)
                    if (k.block && k.key == c.key && k.name().empty() && idx++ == want) { match = &k; break; }
            }
        }
        if (match) *match = overlay(*match, c);
        else out.kids.push_back(c);
    }
    return out;
}

void collect_vars(const Node& n, std::map<std::string, std::string>& vars) {
    for (auto& k : n.kids) {
        if (k.key == "set" && k.args.size() >= 3 && k.args[1].size() > 1 && k.args[1][0] == '$') {
            std::string v = k.args[2];
            for (size_t i = 3; i < k.args.size(); i++) v += " " + k.args[i];
            vars[k.args[1]] = v;
        }
        collect_vars(k, vars);
    }
}

void substitute_vars(Node& n, const std::map<std::string, std::string>& vars) {
    for (auto& a : n.args) {
        if (a.size() > 1 && a[0] == '$') {
            auto it = vars.find(a);
            if (it != vars.end()) a = it->second;
        }
    }
    for (auto& k : n.kids) substitute_vars(k, vars);
}

bool parse_float(const std::string& s, float& v) {
    if (s.empty()) return false;
    char* end = nullptr;
    float f = std::strtof(s.c_str(), &end);
    if (end == s.c_str()) return false;
    if (*end && *end != ',' && *end != 'f') return false;
    v = f;
    return true;
}

bool is_integer(const std::string& s) {
    if (s.empty()) return false;
    for (char c : s)
        if (c < '0' || c > '9') return false;
    return true;
}

// Reads up to `n` floats from args[first...]; returns the number read.
int read_floats(const Node& s, size_t first, float* out, int n) {
    int got = 0;
    for (size_t i = first; i < s.args.size() && got < n; i++) {
        if (!parse_float(s.args[i], out[got])) break;
        got++;
    }
    return got;
}

// ------------------------------------------------------------------ technique evaluation

struct UnitInfo {
    std::string name, alias, tex;
    bool env = false, cubic = false;
    int coord = 0;
    int manual = 0;            // colour_op_ex with src_manual: 1 = replaces the colour, 2 = modulates it
    vec3 manual_rgb{1, 1, 1};
    float manual_alpha = -1.0f; // alpha_op_ex source1 src_manual
};

enum class Blend { Opaque, Alpha, Add, Modulate };

struct PassInfo {
    vec4 diffuse{1, 1, 1, 1};
    vec3 ambient{1, 1, 1}, specular{0, 0, 0}, emissive{0, 0, 0};
    float shininess = 0.0f;
    Blend blend = Blend::Opaque;
    bool alpha_test = false;
    float alpha_ref = 0.5f;
    bool depth_write = true, lighting = true, cull_none = false, colour_write = true;
    std::vector<UnitInfo> units;

    bool has_2d_texture() const {
        for (auto& u : units)
            if (!u.tex.empty() && !u.env && !u.cubic) return true;
        return false;
    }
};

Blend parse_blend(const Node& s) {
    if (s.args.size() < 2) return Blend::Opaque;
    std::string a = to_lower(s.args[1]);
    if (s.args.size() == 2) {
        if (a == "add") return Blend::Add;
        if (a == "modulate") return Blend::Modulate;
        if (a == "replace") return Blend::Opaque;
        return Blend::Alpha; // alpha_blend, colour_blend
    }
    std::string b = to_lower(s.args[2]);
    if (a == "one" && b == "zero") return Blend::Opaque;
    if (b == "one") return Blend::Add; // one one, src_alpha one, ...
    if ((a == "dest_colour" && b == "zero") || (a == "zero" && b == "src_colour")) return Blend::Modulate;
    return Blend::Alpha;
}

// First frame of `anim_texture`: short form "name.ext N duration" -> "name_0.ext", long form lists frames.
std::string anim_first_frame(const Node& s) {
    if (s.args.size() < 2) return {};
    float dummy;
    if (s.args.size() == 4 && is_integer(s.args[2]) && parse_float(s.args[3], dummy)) {
        const std::string& f = s.args[1];
        size_t dot = f.find_last_of('.');
        if (dot == std::string::npos) return f + "_0";
        return f.substr(0, dot) + "_0" + f.substr(dot);
    }
    return s.args[1];
}

UnitInfo eval_unit(const Node& u, const std::map<std::string, std::string>& aliases) {
    UnitInfo ui;
    ui.name = u.name();
    ui.alias = ui.name;
    bool skip = false;
    for (auto& s : u.kids) {
        if (s.key == "texture" && s.args.size() >= 2) {
            ui.tex = s.args[1];
            for (size_t i = 2; i < s.args.size(); i++)
                if (iequals(s.args[i], "cubic")) ui.cubic = true;
        } else if (s.key == "anim_texture") {
            ui.tex = anim_first_frame(s);
        } else if (s.key == "cubic_texture" && s.args.size() >= 2) {
            ui.tex = s.args[1];
            ui.cubic = true;
        } else if (s.key == "env_map" && s.args.size() >= 2) {
            ui.env = !iequals(s.args[1], "off") && !iequals(s.args[1], "false");
        } else if (s.key == "texture_alias" && s.args.size() >= 2) {
            ui.alias = s.args[1];
            for (size_t i = 2; i < s.args.size(); i++) ui.alias += " " + s.args[i];
        } else if (s.key == "tex_coord_set" && s.args.size() >= 2) {
            ui.coord = std::atoi(s.args[1].c_str());
        } else if (s.key == "content_type" && s.args.size() >= 2 && !iequals(s.args[1], "named")) {
            skip = true; // shadow / compositor input
        } else if ((s.key == "colour_op_ex" || s.key == "color_op_ex") && s.args.size() >= 7) {
            // colour_op_ex <op> <src1> <src2> [factor] <r g b>: constant colours (RoR tracks/black etc.)
            std::string op = to_lower(s.args[1]), s1 = to_lower(s.args[2]), s2 = to_lower(s.args[3]);
            float c[3];
            size_t first = op == "blend_manual" ? 5 : 4;
            if (read_floats(s, first, c, 3) == 3) {
                if ((op == "source1" && s1 == "src_manual") || (op == "source2" && s2 == "src_manual")) ui.manual = 1;
                else if (op == "modulate" && (s1 == "src_manual" || s2 == "src_manual")) ui.manual = 2;
                ui.manual_rgb = {c[0], c[1], c[2]};
            }
        } else if (s.key == "alpha_op_ex" && s.args.size() >= 5 && iequals(s.args[1], "source1") && iequals(s.args[2], "src_manual")) {
            float a;
            if (parse_float(s.args[4], a)) ui.manual_alpha = a;
        }
    }
    if (!ui.alias.empty()) {
        auto it = aliases.find(ui.alias);
        if (it != aliases.end()) ui.tex = it->second;
    }
    if (skip) ui.tex.clear();
    return ui;
}

PassInfo eval_pass(const Node& p, const std::map<std::string, std::string>& aliases) {
    PassInfo pi;
    for (auto& s : p.kids) {
        const std::string& k = s.key;
        float f[5];
        if (k == "texture_unit") {
            pi.units.push_back(eval_unit(s, aliases));
        } else if (k == "diffuse" && s.args.size() >= 2) {
            int got = read_floats(s, 1, f, 4);
            if (got >= 3) pi.diffuse = {f[0], f[1], f[2], got >= 4 ? f[3] : 1.0f};
            // "vertexcolour": tracks vertex colours, which we don't import -> keep white
        } else if (k == "ambient") {
            if (read_floats(s, 1, f, 3) == 3) pi.ambient = {f[0], f[1], f[2]};
        } else if (k == "emissive" || k == "self_illumination") {
            if (read_floats(s, 1, f, 3) == 3) pi.emissive = {f[0], f[1], f[2]};
        } else if (k == "specular") {
            int got = read_floats(s, 1, f, 5);
            if (got >= 4) {
                pi.specular = {f[0], f[1], f[2]};
                pi.shininess = f[got - 1]; // "r g b shininess" or "r g b a shininess"
            } else if (s.args.size() >= 3 && iequals(s.args[1], "vertexcolour") && parse_float(s.args[2], f[0])) {
                pi.specular = {1, 1, 1};
                pi.shininess = f[0];
            }
        } else if (k == "scene_blend" || k == "separate_scene_blend") {
            pi.blend = parse_blend(s);
        } else if (k == "alpha_rejection" && s.args.size() >= 2) {
            std::string fn = to_lower(s.args[1]);
            if (fn != "always_pass" && fn != "always_fail") {
                pi.alpha_test = true;
                pi.alpha_ref = s.args.size() >= 3 && parse_float(s.args[2], f[0]) ? saturate(f[0] / 255.0f) : 0.5f;
            }
        } else if (k == "depth_write" && s.args.size() >= 2) {
            pi.depth_write = !iequals(s.args[1], "off") && !iequals(s.args[1], "false");
        } else if (k == "lighting" && s.args.size() >= 2) {
            pi.lighting = !iequals(s.args[1], "off") && !iequals(s.args[1], "false");
        } else if ((k == "cull_hardware" || k == "cull_software") && s.args.size() >= 2) {
            if (iequals(s.args[1], "none")) pi.cull_none = true;
        } else if (k == "colour_write" && s.args.size() >= 2) {
            pi.colour_write = !iequals(s.args[1], "off") && !iequals(s.args[1], "false");
        }
    }
    for (auto& u : pi.units) {
        if (u.manual == 1) { // constant colour, lighting bypassed
            pi.diffuse = vec4(u.manual_rgb, pi.diffuse.w);
            pi.lighting = false;
        } else if (u.manual == 2) {
            pi.diffuse = vec4(vec3(pi.diffuse.x, pi.diffuse.y, pi.diffuse.z) * u.manual_rgb, pi.diffuse.w);
            pi.ambient = pi.ambient * u.manual_rgb;
        }
        if (u.manual_alpha >= 0.0f) pi.diffuse.w = u.manual_alpha;
    }
    return pi;
}

bool looks_like_non_diffuse(const std::string& s) {
    static const char* words[] = {"spec", "norm", "bump", "env", "refl", "detail", "mask", "emis", "glow", "light", "cube"};
    std::string l = to_lower(s);
    for (auto w : words)
        if (l.find(w) != std::string::npos) return true;
    return false;
}

// Base colour texture of a pass: first plain 2D unit, preferring UV set 0 and units not named like spec/normal maps.
const UnitInfo* pick_diffuse_unit(const PassInfo& p) {
    const UnitInfo* fallback = nullptr;
    for (auto& u : p.units) {
        if (u.tex.empty() || u.env || u.cubic) continue;
        if (!fallback) fallback = &u;
        if (u.coord == 0 && !looks_like_non_diffuse(u.name) && !looks_like_non_diffuse(u.alias)) return &u;
    }
    return fallback;
}

std::string resolve_texture(const std::string& dir, const std::string& name) {
    if (name.empty()) return {};
    std::string f = find_file_ci(dir, name);
    if (f.empty()) {
        // RoR mods often reference .tga/.dds that were shipped converted to another format.
        static const char* exts[] = {".dds", ".png", ".jpg", ".jpeg", ".tga", ".bmp"};
        std::string stem = path_stem(name);
        for (auto e : exts)
            if (!(f = find_file_ci(dir, stem + e)).empty()) break;
    }
    if (f.empty()) return {};
    std::error_code ec;
    std::string abs = std::filesystem::absolute(f, ec).string();
    return ec ? f : abs;
}

struct Source {
    Node node;
    std::string parent, file;
    bool abstract = false;
};

// Built-in RoR (resources/materials/ror.material) and OGRE materials that vehicles reference by name.
// Their textures ship with RoR, not with the mods, so they are approximated by colours.
enum : unsigned { B_INVIS = 1, B_UNLIT = 2, B_BLEND = 4, B_NODEPTH = 8, B_2SIDED = 16, B_REFLECT = 32, B_FLARE = 64 };
struct Builtin {
    const char* name;
    vec3 colour;
    float alpha;
    unsigned flags;
};
constexpr vec3 kFace{0.62f, 0.62f, 0.64f}, kBand{0.12f, 0.12f, 0.12f}, kChrome{0.8f, 0.8f, 0.82f};
constexpr unsigned kFlare = B_FLARE | B_BLEND | B_UNLIT | B_NODEPTH;
const Builtin kBuiltins[] = {
    {"tracks/trans", {1, 1, 1}, 1, B_INVIS},
    {"tracks/transred", {0.5f, 0, 0}, 0.8f, B_INVIS},
    {"tracks/invisible", {0, 0, 0}, 1, B_INVIS},
    {"gavrilinvis", {1, 1, 1}, 1, B_INVIS},
    {"tracks/beam", {0.45f, 0.42f, 0.38f}, 1, 0}, // RustySteel.dds
    {"tracks/beamblack", {0.03f, 0.03f, 0.03f}, 1, 0},
    {"tracks/wheelface", kFace, 1, 0},
    {"tracks/wheelfaceb", kFace, 1, 0},
    {"tracks/wheelband", kBand, 1, 0},
    {"tracks/wheelband1", kBand, 1, 0},
    {"tracks/wheelband2", kBand, 1, 0},
    {"tracks/dafrwheelface", kFace, 1, 0},
    {"tracks/daffwheelface", kFace, 1, 0},
    {"tracks/dafwheelband", kBand, 1, 0},
    {"tracks/dodgewheelface", kFace, 1, 0},
    {"tracks/dodgewheelband", kBand, 1, 0},
    {"tracks/an12wheelface", kFace, 1, 0},
    {"tracks/an12wheelband", kBand, 1, 0},
    {"tracks/ttwinwheelface", kFace, 1, 0},
    {"tracks/ttwinwheelband", kBand, 1, 0},
    {"tracks/tatrawheelface", kFace, 1, 0},
    {"tracks/tatrawheelband", kBand, 1, 0},
    {"driversseat", {0.35f, 0.33f, 0.3f}, 1, 0},
    {"seat", {0.35f, 0.33f, 0.3f}, 1, 0},
    {"dashboard", {0.1f, 0.1f, 0.11f}, 1, 0},
    {"renderdash", {0.06f, 0.06f, 0.07f}, 1, B_UNLIT}, // render target of the RoR dashboard
    {"dirwheel", {0.1f, 0.1f, 0.1f}, 1, 0},
    {"prop", {0.2f, 0.2f, 0.2f}, 1, 0},
    {"metal", {0.5f, 0.5f, 0.5f}, 1, 0},
    {"mirror", kChrome, 1, B_REFLECT},
    {"tracks/mirror", kChrome, 1, B_REFLECT},
    {"tracks/Chrome", kChrome, 1, B_REFLECT},
    {"tracks/chrome", kChrome, 1, B_REFLECT},
    {"tracks/black", {0, 0, 0}, 1, B_UNLIT},
    {"tracks/white", {1, 1, 1}, 1, 0},
    {"tracks/simple", {0.61f, 0.61f, 0.61f}, 1, B_2SIDED},
    {"tracks/graytrans", {0.5f, 0.5f, 0.5f}, 0.3f, B_BLEND},
    {"tracks/spinprop", {0.3f, 0.3f, 0.3f}, 0.5f, B_BLEND | B_NODEPTH | B_2SIDED},
    {"beacon", {1.0f, 0.55f, 0.05f}, 1, B_BLEND | B_2SIDED},
    {"redbeacon", {1.0f, 0.1f, 0.05f}, 1, B_BLEND | B_2SIDED},
    {"lightbar", {1.0f, 0.2f, 0.1f}, 1, B_BLEND | B_2SIDED},
    {"tracks/flare", {1.0f, 0.95f, 0.8f}, 1, kFlare},
    {"tracks/whiteflare", {1.0f, 1.0f, 1.0f}, 1, kFlare},
    {"tracks/redflare", {1.0f, 0.1f, 0.05f}, 1, kFlare},
    {"tracks/brakeflare", {1.0f, 0.1f, 0.05f}, 1, kFlare},
    {"tracks/brightredflare", {1.0f, 0.15f, 0.1f}, 1, kFlare},
    {"tracks/greenflare", {0.1f, 1.0f, 0.2f}, 1, kFlare},
    {"tracks/blinkflare", {1.0f, 0.55f, 0.05f}, 1, kFlare},
    {"tracks/aimflare", {1.0f, 0.2f, 0.2f}, 1, kFlare},
    {"tracks/beaconflare", {1.0f, 0.55f, 0.05f}, 1, kFlare},
    {"tracks/orangebeaconflare", {1.0f, 0.55f, 0.05f}, 1, kFlare},
    {"tracks/redbeaconflare", {1.0f, 0.1f, 0.05f}, 1, kFlare},
    {"tracks/bluebeaconflare", {0.1f, 0.3f, 1.0f}, 1, kFlare},
    {"tracks/brightblueflare", {0.2f, 0.4f, 1.0f}, 1, kFlare},
    {"BaseWhite", {1, 1, 1}, 1, 0},
    {"BaseWhiteNoLighting", {1, 1, 1}, 1, B_UNLIT},
};

bool is_builtin_name(const std::string& n) {
    for (auto& b : kBuiltins)
        if (n == b.name) return true;
    return false;
}

} // namespace

// ------------------------------------------------------------------ MaterialLibrary

void MaterialLibrary::load_dir(const std::string& dir) {
    namespace fs = std::filesystem;
    std::vector<std::string> files;
    std::error_code ec;
    for (auto it = fs::recursive_directory_iterator(dir, ec); !ec && it != fs::recursive_directory_iterator(); it.increment(ec)) {
        if (it->is_regular_file(ec) && path_ext_lower(it->path().string()) == ".material") files.push_back(it->path().string());
    }
    std::sort(files.begin(), files.end());

    // 1) parse every script and collect material definitions (first definition of a name wins, like OGRE)
    std::unordered_map<std::string, Source> sources;
    std::vector<std::string> order;
    for (auto& f : files) {
        std::string text;
        if (!read_text_file(f, text)) {
            warnings.push_back("cannot read " + f);
            continue;
        }
        Node root = parse_script(text, f, warnings);
        for (auto& obj : root.kids) {
            size_t k = 0;
            bool abstract = false;
            if (obj.key == "abstract" && obj.args.size() >= 2 && iequals(obj.args[1], "material")) abstract = true, k = 1;
            else if (obj.key != "material") continue; // particle systems, programs, import ...
            if (obj.args.size() < k + 2) continue;
            Source src;
            src.abstract = abstract;
            src.file = f;
            std::string name = obj.args[k + 1];
            for (size_t i = k + 2; i + 1 < obj.args.size(); i++)
                if (obj.args[i] == ":") src.parent = obj.args[i + 1];
            if (!obj.block) {
                warnings.push_back(format("%s:%d: material '%s' has no body", path_filename(f).c_str(), obj.line, name.c_str()));
                continue;
            }
            if (sources.count(name)) {
                warnings.push_back(format("%s:%d: duplicate material '%s' ignored", path_filename(f).c_str(), obj.line, name.c_str()));
                continue;
            }
            src.node = obj;
            sources.emplace(name, std::move(src));
            order.push_back(name);
        }
    }

    // 2) resolve inheritance
    std::function<bool(const std::string&, Node&, int)> resolve = [&](const std::string& name, Node& out, int depth) {
        auto it = sources.find(name);
        if (it == sources.end() || depth > 16) return false;
        const Source& s = it->second;
        Node base;
        if (!s.parent.empty() && resolve(s.parent, base, depth + 1)) out = overlay(base, s.node);
        else out = s.node;
        return true;
    };

    // 3) reduce to MaterialDesc
    for (auto& name : order) {
        const Source& s = sources[name];
        if (s.abstract) continue;
        Node tree;
        resolve(name, tree, 0);
        bool parent_missing = !s.parent.empty() && !sources.count(s.parent);
        if (parent_missing)
            warnings.push_back(format("material '%s': parent '%s' not found", name.c_str(), s.parent.c_str()));

        std::map<std::string, std::string> vars, aliases;
        collect_vars(tree, vars);
        if (!vars.empty()) substitute_vars(tree, vars);
        std::vector<std::pair<std::string, std::string>> alias_list;
        for (auto& k : tree.kids)
            if (k.key == "set_texture_alias" && k.args.size() >= 3) {
                aliases[k.args[1]] = k.args[2];
                alias_list.push_back({k.args[1], k.args[2]});
            }

        MaterialDesc m;
        m.name = name;

        // technique: first one without a (non-default) scheme
        const Node* tech = nullptr;
        for (auto& k : tree.kids) {
            if (k.key != "technique") continue;
            const Node* sch = find_kid(k, "scheme");
            if (!sch || sch->args.size() < 2 || iequals(sch->args[1], "default")) { tech = &k; break; }
            if (!tech) tech = &k;
        }
        std::vector<PassInfo> passes;
        if (tech)
            for (auto& k : tech->kids)
                if (k.key == "pass") passes.push_back(eval_pass(k, aliases));

        // visible passes (colour_write off = depth-only / hidden)
        std::vector<const PassInfo*> vis;
        for (auto& p : passes)
            if (p.colour_write) vis.push_back(&p);
        if (!passes.empty() && vis.empty()) m.invisible = true;

        std::string tex_name;
        if (!vis.empty()) {
            const PassInfo* base = vis[0];
            size_t mi = 0; // main pass: first with a plain texture (skips ambient-only first passes)
            for (size_t i = 0; i < vis.size(); i++)
                if (vis[i]->has_2d_texture()) { mi = i; break; }
            const PassInfo* mp = vis[mi];

            if (const UnitInfo* u = pick_diffuse_unit(*mp)) tex_name = u->tex;
            m.diffuse = mp->diffuse;
            m.ambient = mp->ambient;
            m.specular = mp->specular;
            m.shininess = mp->shininess;
            m.emissive = mp->emissive;
            m.lighting = mp->lighting;
            m.depth_write = base->depth_write;
            m.alpha_test = mp->alpha_test || base->alpha_test;
            m.alpha_ref = mp->alpha_test ? mp->alpha_ref : base->alpha_ref;
            Blend blend = base->blend;
            if (mi > 0 && base->blend == Blend::Opaque) blend = Blend::Opaque; // textured layer over an opaque pass
            else if (mi > 0) blend = mp->blend;
            m.blend = blend != Blend::Opaque;
            if (blend == Blend::Add && m.emissive.x + m.emissive.y + m.emissive.z == 0.0f) m.emissive = vec3(1.0f);
            m.double_sided = base->cull_none || mp->cull_none;

            for (size_t i = 0; i < vis.size(); i++) {
                const PassInfo& p = *vis[i];
                for (auto& u : p.units)
                    if (!u.tex.empty() && (u.env || u.cubic)) m.reflective = true;
                if (i <= mi) continue;
                bool overlay_light = p.blend == Blend::Add || (!p.lighting && p.blend == Blend::Alpha);
                if (!overlay_light) continue;
                bool env = false;
                for (auto& u : p.units) env |= u.env || u.cubic;
                const UnitInfo* u = pick_diffuse_unit(p);
                if (u && !env && m.emissive_tex.empty()) {
                    m.emissive_tex = resolve_texture(dir, u->tex);
                    if (m.emissive_tex.empty())
                        warnings.push_back(format("material '%s': emissive texture '%s' not found", name.c_str(), u->tex.c_str()));
                }
                // specular-only additive pass (common exporter output): borrow its highlight
                if (!u && p.blend == Blend::Add && m.specular.x + m.specular.y + m.specular.z == 0.0f &&
                    p.specular.x + p.specular.y + p.specular.z > 0.0f) {
                    m.specular = p.specular;
                    m.shininess = p.shininess;
                }
            }
        }
        // Unknown parent (e.g. an engine base material): use the most diffuse-looking texture alias.
        if (tex_name.empty() && !alias_list.empty()) {
            tex_name = alias_list[0].second;
            for (auto& a : alias_list)
                if (to_lower(a.first).find("diff") != std::string::npos || to_lower(a.first).find("albedo") != std::string::npos) {
                    tex_name = a.second;
                    break;
                }
        }
        if (!tex_name.empty()) {
            m.diffuse_tex = resolve_texture(dir, tex_name);
            if (m.diffuse_tex.empty())
                warnings.push_back(format("material '%s': texture '%s' not found", name.c_str(), tex_name.c_str()));
        }

        auto ex = m_materials.find(name);
        if (ex != m_materials.end() && !is_builtin_name(name)) {
            warnings.push_back(format("material '%s' already registered, script definition ignored", name.c_str()));
            continue;
        }
        m_materials[name] = std::move(m);
    }
}

void MaterialLibrary::add_ror_builtins() {
    for (auto& b : kBuiltins) {
        if (m_materials.count(b.name)) continue; // vehicle scripts / managed materials take precedence
        MaterialDesc m;
        m.name = b.name;
        m.diffuse = vec4(b.colour, b.alpha);
        m.ambient = b.colour;
        m.invisible = b.flags & B_INVIS;
        m.lighting = !(b.flags & B_UNLIT);
        m.blend = b.flags & B_BLEND;
        m.depth_write = !(b.flags & B_NODEPTH);
        m.double_sided = b.flags & B_2SIDED;
        if (b.flags & B_REFLECT) {
            m.reflective = true;
            m.specular = vec3(0.8f);
            m.shininess = 64.0f;
        }
        if (b.flags & B_FLARE) { // flare billboards: alpha_blend + alpha_rejection greater 2
            m.emissive = b.colour;
            m.alpha_test = true;
            m.alpha_ref = 2.0f / 255.0f;
        }
        m_materials[b.name] = std::move(m);
    }
}

void MaterialLibrary::add_managed(const ror::ManagedMaterialDef& def, const std::string& dir) {
    if (def.name.empty()) return;
    MaterialDesc m;
    m.name = def.name;
    std::string type = to_lower(def.type);
    if (type != "mesh_standard" && type != "mesh_transparent" && type != "flexmesh_standard" && type != "flexmesh_transparent")
        warnings.push_back(format("managed material '%s': unknown type '%s'", def.name.c_str(), def.type.c_str()));
    if (type.find("transparent") != std::string::npos) {
        // RoR's managed/*_transparent templates: scene_blend alpha_blend + alpha_rejection greater 0.
        // (Glass textures carry alpha ~0.1-0.3, so a 0.5 alpha test alone would delete the windows.)
        m.alpha_test = true;
        m.alpha_ref = 1.0f / 255.0f;
        m.blend = true;
    }
    m.double_sided = def.double_sided;
    if (!def.diffuse.empty() && def.diffuse != "-") {
        m.diffuse_tex = resolve_texture(dir, def.diffuse);
        if (m.diffuse_tex.empty())
            warnings.push_back(format("managed material '%s': texture '%s' not found", def.name.c_str(), def.diffuse.c_str()));
    }
    if (!def.specular.empty() && def.specular != "-" && !resolve_texture(dir, def.specular).empty()) {
        m.specular = vec3(0.3f); // specular map present: modest uniform highlight
        m.shininess = 32.0f;
    }
    if (m_materials.count(def.name))
        warnings.push_back(format("managed material '%s' replaces a script material", def.name.c_str()));
    m_materials[def.name] = std::move(m);
}

const MaterialDesc* MaterialLibrary::find(const std::string& name) const {
    auto it = m_materials.find(name);
    if (it != m_materials.end()) return &it->second;
    for (auto& kv : m_materials) // content often gets the case wrong
        if (iequals(kv.first, name)) return &kv.second;
    return nullptr;
}

} // namespace bl
