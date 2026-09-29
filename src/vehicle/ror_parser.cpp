// Rigs of Rods vehicle definition parser (see ror_def.h and ror_format_spec.md).
// Mirrors RoR's RigDef::Parser line/block state machine with a hand-written tokenizer and keyword lookup (no regex).
// One pass over the lines builds per-module element lists; node references are resolved at the end, because
// numeric references may point to nodes generated later in the file (cinecams, wheels).
#include "vehicle/ror_def.h"
#include "phys/frame_fem.h"

#include "core/util.h"

#include <algorithm>
#include <cctype>
#include <cfloat>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string_view>
#include <unordered_map>

namespace bl::ror {

namespace {

using sv = std::string_view;

constexpr int kMaxTokens = 100;       // RoR LINE_MAX_ARGS
constexpr size_t kMaxLineLen = 1999;  // RoR reads at most this many chars per line; the rest continues as a new line
constexpr int kPendingRef = 1 << 30;  // node refs >= this index Parser::pending_ (name lookups resolved at the end)
constexpr int kMaxNumRef = 1 << 29;   // numeric refs are clamped here (they resolve to node 0 anyway)
constexpr int kMaxRays = 1000;
constexpr int kMaxWarnings = 500;
const char* const kRootModule = "_Root_";

// Option letters accepted per element (RoR GetArg*Options).
const char* const kNodeOpts = "nlmfchxyebpL";
const char* const kBeamOpts = "virsF";
const char* const kShockOpts = "iLRmnv";
const char* const kShock2Opts = "ismMnv";
const char* const kShock3Opts = "imMnv";
const char* const kHydroOpts = "jsaeruvxyghni";
const char* const kCommandOpts = "nirfcpo";
const char* const kCabOpts = "ncbpusrDFSh";
const char* const kTieOpts = "nvis";

// ------------------------------------------------------------------ keywords

enum class Kw : uint8_t {
    NONE,
    // blocks with a handler here
    AXLES, BEAMS, BRAKES, CAB, CAMERAS, CINECAM, COMMANDS, COMMANDS2, CONTACTERS, ENGINE, ENGOPTION, FIXES, JOINTS, SHELLS, WELDS, MOUNTS, FEM_TRIS,
    FLEXBODIES, FLEXBODYWHEELS, GLOBALS, GUISETTINGS, HYDROS, MANAGEDMATERIALS, MESHWHEELS, MESHWHEELS2, MINIMASS,
    NODES, NODES2, PROPS, ROPES, SHOCKS, SHOCKS2, SHOCKS3, TEXCOORDS, TIES, TORQUECURVE, WHEELDETACHERS, WHEELS,
    WHEELS2,
    // blocks whose data is skipped (they still become the current block)
    AIRBRAKES, ANIMATORS, ASSETPACKS, CAMERARAIL, COLLISIONBOXES, CUSTOMDASHBOARDINPUTS, ENGTURBO, EXHAUSTS, FLARES,
    FLARES2, FLARES3, FLAREGROUPS_NO_IMPORT, FUSEDRAG, HELP, HOOKS, INTERAXLES, LOCKGROUPS, MATERIALFLAREBINDINGS,
    PARTICLES, PISTONPROPS, RAILGROUPS, ROPABLES, ROTATORS, ROTATORS2, SCREWPROPS, SCRIPTS, SLIDENODES, SOUNDSOURCES,
    SOUNDSOURCES2, TRANSFERCASE, TRIGGERS, TURBOJETS, TURBOPROPS, TURBOPROPS2, VIDEOCAMERA, WINGS,
    SECTIONCONFIG, SLOPEBRAKE, SET_SHADOWS, // no RoR handler: they swallow the following data lines
    // text blocks
    COMMENT, DESCRIPTION,
    // flag directives
    DISABLEDEFAULTSOUNDS, ENABLE_ADVANCED_DEFORMATION, FORWARDCOMMANDS, HIDEINCHOOSER, IMPORTCOMMANDS,
    LOCKGROUP_DEFAULT_NOLOCK, RESCUER, ROLLON, SLIDENODE_CONNECT_INSTANTLY,
    // directives with arguments
    ADD_ANIMATION, ANTILOCKBRAKES, AUTHOR, BACKMESH, CRUISECONTROL, DEFAULT_SKIN, DETACHER_GROUP, EXTCAMERA,
    FILEFORMATVERSION, FILEINFO, FLEXBODY_CAMERA_MODE, FORSET, FORVERT, GUID, PROP_CAMERA_MODE, SECTION,
    SET_BEAM_DEFAULTS, SET_BEAM_DEFAULTS_SCALE, SET_COLLISION_RANGE, SET_DEFAULT_MINIMASS, SET_INERTIA_DEFAULTS,
    SET_MANAGEDMATERIALS_OPTIONS, SET_NODE_DEFAULTS, SET_SHELL_MATERIAL, SET_FRAME_SECTION, SET_FEM_SHELL, SET_SKELETON_SETTINGS, SPEEDLIMITER, SUBMESH, SUBMESH_GROUNDMODEL,
    TRACTIONCONTROL,
    // terminators
    END, END_COMMENT, END_DESCRIPTION, END_SECTION,
    // obsolete, ignored
    ENVMAP, HOOKGROUP, NODECOLLISION, RIGIDIFIERS,
};

struct KwInfo {
    const char* name; // lower case
    Kw kw;
    bool inline_args; // `kw<sep>args` (else the keyword must be alone on its line)
};

const KwInfo kKeywords[] = {
    {"add_animation", Kw::ADD_ANIMATION, true}, {"airbrakes", Kw::AIRBRAKES, false},
    {"animators", Kw::ANIMATORS, false}, {"antilockbrakes", Kw::ANTILOCKBRAKES, true},
    {"assetpacks", Kw::ASSETPACKS, false}, {"author", Kw::AUTHOR, true}, {"axles", Kw::AXLES, false},
    {"backmesh", Kw::BACKMESH, false}, {"beams", Kw::BEAMS, false}, {"brakes", Kw::BRAKES, false},
    {"cab", Kw::CAB, false}, {"camerarail", Kw::CAMERARAIL, false}, {"cameras", Kw::CAMERAS, false},
    {"cinecam", Kw::CINECAM, false}, {"collisionboxes", Kw::COLLISIONBOXES, false},
    {"commands", Kw::COMMANDS, false}, {"commands2", Kw::COMMANDS2, false}, {"comment", Kw::COMMENT, false},
    {"contacters", Kw::CONTACTERS, false}, {"cruisecontrol", Kw::CRUISECONTROL, true},
    {"customdashboardinputs", Kw::CUSTOMDASHBOARDINPUTS, false}, {"default_skin", Kw::DEFAULT_SKIN, true},
    {"description", Kw::DESCRIPTION, false}, {"detacher_group", Kw::DETACHER_GROUP, true},
    {"disabledefaultsounds", Kw::DISABLEDEFAULTSOUNDS, false},
    {"enable_advanced_deformation", Kw::ENABLE_ADVANCED_DEFORMATION, false}, {"end", Kw::END, false},
    {"end_comment", Kw::END_COMMENT, false}, {"end_description", Kw::END_DESCRIPTION, false},
    {"end_section", Kw::END_SECTION, false}, {"engine", Kw::ENGINE, false}, {"engoption", Kw::ENGOPTION, false},
    {"engturbo", Kw::ENGTURBO, false}, {"envmap", Kw::ENVMAP, false}, {"exhausts", Kw::EXHAUSTS, false},
    {"extcamera", Kw::EXTCAMERA, true}, {"fileformatversion", Kw::FILEFORMATVERSION, true},
    {"fileinfo", Kw::FILEINFO, true}, {"fixes", Kw::FIXES, false}, {"flares", Kw::FLARES, false},
    {"flares2", Kw::FLARES2, false}, {"flares3", Kw::FLARES3, false},
    {"flaregroups_no_import", Kw::FLAREGROUPS_NO_IMPORT, false}, {"flexbodies", Kw::FLEXBODIES, false},
    {"flexbody_camera_mode", Kw::FLEXBODY_CAMERA_MODE, true}, {"flexbodywheels", Kw::FLEXBODYWHEELS, false},
    {"forset", Kw::FORSET, true}, {"forvert", Kw::FORVERT, true}, {"forwardcommands", Kw::FORWARDCOMMANDS, false},
    {"fusedrag", Kw::FUSEDRAG, false}, {"globals", Kw::GLOBALS, false}, {"guid", Kw::GUID, true},
    {"guisettings", Kw::GUISETTINGS, false}, {"help", Kw::HELP, false}, {"hideinchooser", Kw::HIDEINCHOOSER, false},
    {"hookgroup", Kw::HOOKGROUP, false}, {"hooks", Kw::HOOKS, false}, {"hydros", Kw::HYDROS, false},
    {"importcommands", Kw::IMPORTCOMMANDS, false}, {"interaxles", Kw::INTERAXLES, false},
    {"joints", Kw::JOINTS, false}, {"welds", Kw::WELDS, false}, {"mounts", Kw::MOUNTS, false}, {"fem_tris", Kw::FEM_TRIS, false}, {"lockgroups", Kw::LOCKGROUPS, false}, {"lockgroup_default_nolock", Kw::LOCKGROUP_DEFAULT_NOLOCK, false},
    {"managedmaterials", Kw::MANAGEDMATERIALS, false}, {"materialflarebindings", Kw::MATERIALFLAREBINDINGS, false},
    {"meshwheels", Kw::MESHWHEELS, false}, {"meshwheels2", Kw::MESHWHEELS2, false},
    {"minimass", Kw::MINIMASS, false}, {"nodecollision", Kw::NODECOLLISION, false}, {"nodes", Kw::NODES, false},
    {"nodes2", Kw::NODES2, false}, {"particles", Kw::PARTICLES, false}, {"pistonprops", Kw::PISTONPROPS, false},
    {"prop_camera_mode", Kw::PROP_CAMERA_MODE, true}, {"props", Kw::PROPS, false},
    {"railgroups", Kw::RAILGROUPS, false}, {"rescuer", Kw::RESCUER, false}, {"rigidifiers", Kw::RIGIDIFIERS, false},
    {"rollon", Kw::ROLLON, false}, {"ropables", Kw::ROPABLES, false}, {"ropes", Kw::ROPES, false},
    {"rotators", Kw::ROTATORS, false}, {"rotators2", Kw::ROTATORS2, false}, {"screwprops", Kw::SCREWPROPS, false},
    {"scripts", Kw::SCRIPTS, false}, {"section", Kw::SECTION, true}, {"sectionconfig", Kw::SECTIONCONFIG, true},
    {"set_beam_defaults", Kw::SET_BEAM_DEFAULTS, true}, {"set_beam_defaults_scale", Kw::SET_BEAM_DEFAULTS_SCALE, true},
    {"set_collision_range", Kw::SET_COLLISION_RANGE, true}, {"set_default_minimass", Kw::SET_DEFAULT_MINIMASS, true},
    {"set_inertia_defaults", Kw::SET_INERTIA_DEFAULTS, true},
    {"set_managedmaterials_options", Kw::SET_MANAGEDMATERIALS_OPTIONS, true},
    {"set_node_defaults", Kw::SET_NODE_DEFAULTS, true}, {"set_shadows", Kw::SET_SHADOWS, false},
    {"set_shell_material", Kw::SET_SHELL_MATERIAL, true}, {"set_frame_section", Kw::SET_FRAME_SECTION, true}, {"set_fem_shell", Kw::SET_FEM_SHELL, true}, {"set_skeleton_settings", Kw::SET_SKELETON_SETTINGS, true}, {"shells", Kw::SHELLS, false}, {"shocks", Kw::SHOCKS, false},
    {"shocks2", Kw::SHOCKS2, false}, {"shocks3", Kw::SHOCKS3, false},
    {"slidenode_connect_instantly", Kw::SLIDENODE_CONNECT_INSTANTLY, false}, {"slidenodes", Kw::SLIDENODES, false},
    {"slopebrake", Kw::SLOPEBRAKE, true}, {"soundsources", Kw::SOUNDSOURCES, false},
    {"soundsources2", Kw::SOUNDSOURCES2, false}, {"speedlimiter", Kw::SPEEDLIMITER, true},
    {"submesh", Kw::SUBMESH, false}, {"submesh_groundmodel", Kw::SUBMESH_GROUNDMODEL, true},
    {"texcoords", Kw::TEXCOORDS, false}, {"ties", Kw::TIES, false}, {"torquecurve", Kw::TORQUECURVE, false},
    {"tractioncontrol", Kw::TRACTIONCONTROL, true}, {"transfercase", Kw::TRANSFERCASE, false},
    {"triggers", Kw::TRIGGERS, false}, {"turbojets", Kw::TURBOJETS, false}, {"turboprops", Kw::TURBOPROPS, false},
    {"turboprops2", Kw::TURBOPROPS2, false}, {"videocamera", Kw::VIDEOCAMERA, false},
    {"wheeldetachers", Kw::WHEELDETACHERS, false}, {"wheels", Kw::WHEELS, false}, {"wheels2", Kw::WHEELS2, false},
    {"wings", Kw::WINGS, false},
};

const std::unordered_map<sv, const KwInfo*>& keyword_map() {
    static const std::unordered_map<sv, const KwInfo*> m = [] {
        std::unordered_map<sv, const KwInfo*> r;
        for (const KwInfo& k : kKeywords) r.emplace(k.name, &k);
        return r;
    }();
    return m;
}

const char* kw_name(Kw kw) {
    for (const KwInfo& k : kKeywords)
        if (k.kw == kw) return k.name;
    return "-";
}

inline bool is_blank(char c) { return c == ' ' || c == '\t'; }
inline bool is_sep(char c) { return c == ' ' || c == '\t' || c == ',' || c == ':' || c == '|'; }
inline bool is_digit(char c) { return c >= '0' && c <= '9'; }

// Equivalent of RoR's IDENTIFY_KEYWORD regex: block keywords must be alone on the line (trailing blanks allowed),
// inline keywords need at least one separator after them, `forset` needs nothing. Case-insensitive.
Kw identify(sv line) {
    char c0 = line[0];
    if (!((c0 >= 'a' && c0 <= 'z') || (c0 >= 'A' && c0 <= 'Z'))) return Kw::NONE;
    if (starts_with_ci(line, "forset")) return Kw::FORSET;
    size_t w = 0;
    while (w < line.size() && !is_sep(line[w])) w++;
    if (w > 32) return Kw::NONE;
    char buf[32];
    for (size_t i = 0; i < w; i++) buf[i] = (char)tolower((unsigned char)line[i]);
    const auto& m = keyword_map();
    auto it = m.find(sv(buf, w));
    if (it == m.end()) return Kw::NONE;
    if (it->second->inline_args) return w < line.size() ? it->second->kw : Kw::NONE;
    for (size_t i = w; i < line.size(); i++)
        if (!is_blank(line[i])) return Kw::NONE;
    return it->second->kw;
}

int tokenize(sv line, sv* out) {
    int n = 0;
    size_t i = 0, len = line.size();
    while (n < kMaxTokens) {
        while (i < len && is_sep(line[i])) i++;
        if (i >= len) break;
        size_t s = i;
        while (i < len && !is_sep(line[i])) i++;
        out[n++] = line.substr(s, i - s);
    }
    return n;
}

sv trim_blanks(sv s) {
    while (!s.empty() && (unsigned char)s.front() <= ' ') s.remove_prefix(1);
    while (!s.empty() && (unsigned char)s.back() <= ' ') s.remove_suffix(1);
    return s;
}

// ------------------------------------------------------------------ scalars

// Ogre parseReal (istream >> float): numeric prefix; garbage or overflow gives 0.
float parse_real(sv s) {
    static const double kPow10[] = {1e0,  1e1,  1e2,  1e3,  1e4,  1e5,  1e6,  1e7,  1e8,  1e9,  1e10, 1e11,
                                    1e12, 1e13, 1e14, 1e15, 1e16, 1e17, 1e18, 1e19, 1e20, 1e21, 1e22};
    size_t i = 0, n = s.size();
    bool neg = false;
    if (i < n && (s[i] == '+' || s[i] == '-')) neg = s[i++] == '-';
    uint64_t mant = 0;
    int exp10 = 0, sig = 0;
    bool any = false;
    for (; i < n && is_digit(s[i]); i++, any = true) {
        if (sig < 19) {
            mant = mant * 10 + (uint64_t)(s[i] - '0');
            if (mant) sig++;
        } else {
            exp10++;
        }
    }
    if (i < n && s[i] == '.') {
        for (i++; i < n && is_digit(s[i]); i++, any = true) {
            if (sig < 19) {
                mant = mant * 10 + (uint64_t)(s[i] - '0');
                if (mant) sig++;
                exp10--;
            }
        }
    }
    if (!any) return 0.0f;
    if (i < n && (s[i] == 'e' || s[i] == 'E')) {
        size_t j = i + 1;
        bool eneg = false;
        if (j < n && (s[j] == '+' || s[j] == '-')) eneg = s[j++] == '-';
        if (j < n && is_digit(s[j])) {
            int e = 0;
            for (; j < n && is_digit(s[j]); j++)
                if (e < 100000) e = e * 10 + (s[j] - '0');
            exp10 += eneg ? -e : e;
        }
    }
    double v = (double)mant;
    if (v != 0.0 && exp10 != 0) {
        if (exp10 > 0 && exp10 <= 22) v *= kPow10[exp10];
        else if (exp10 < 0 && exp10 >= -22) v /= kPow10[-exp10];
        else v *= std::pow(10.0, (double)exp10);
    }
    // Out of float range: RoR's Windows builds (MSVC stream parse via double) yield a huge value, and mods use
    // e.g. "99999999999999999999999999999999999999999" to mean "never deforms / never breaks".
    if (!(v <= (double)FLT_MAX)) v = FLT_MAX;
    return (float)(neg ? -v : v);
}

// Integer prefix (sign + digits). Returns false if there are no digits. `used` = chars consumed.
bool scan_int(sv s, long long& out, size_t& used) {
    size_t i = 0, n = s.size();
    bool neg = false;
    if (i < n && (s[i] == '+' || s[i] == '-')) neg = s[i++] == '-';
    size_t d0 = i;
    long long v = 0;
    for (; i < n && is_digit(s[i]); i++)
        if (v < (1LL << 40)) v = v * 10 + (s[i] - '0');
    used = i;
    out = neg ? -v : v;
    return i > d0;
}

// Ogre parseBool: case-insensitive prefix true/yes/1/on.
bool parse_bool(sv s) {
    return starts_with_ci(s, "true") || starts_with_ci(s, "yes") || starts_with_ci(s, "1") || starts_with_ci(s, "on");
}

// utf8::replace_invalid: invalid sequences become '?'.
bool is_ascii(sv s) {
    for (char c : s)
        if ((unsigned char)c >= 0x80) return false;
    return true;
}

std::string utf8_fix(sv s) {
    std::string out;
    out.reserve(s.size());
    size_t i = 0, n = s.size();
    while (i < n) {
        unsigned char c = (unsigned char)s[i];
        if (c < 0x80) {
            out += (char)c;
            i++;
            continue;
        }
        int len = (c >= 0xC2 && c <= 0xDF) ? 2 : (c >= 0xE0 && c <= 0xEF) ? 3 : (c >= 0xF0 && c <= 0xF4) ? 4 : 0;
        bool ok = len > 0 && i + len <= n;
        for (int k = 1; ok && k < len; k++) ok = ((unsigned char)s[i + k] & 0xC0) == 0x80;
        if (ok && len >= 3) {
            unsigned char c1 = (unsigned char)s[i + 1];
            if (c == 0xE0 && c1 < 0xA0) ok = false;      // overlong
            if (c == 0xED && c1 >= 0xA0) ok = false;     // surrogate
            if (c == 0xF0 && c1 < 0x90) ok = false;      // overlong
            if (c == 0xF4 && c1 >= 0x90) ok = false;     // > U+10FFFF
        }
        if (ok) {
            out.append(s.data() + i, len);
            i += len;
        } else {
            out += '?';
            i++;
            while (i < n && ((unsigned char)s[i] & 0xC0) == 0x80) i++;
        }
    }
    return out;
}

bool has_char(const std::string& s, char c) { return s.find(c) != std::string::npos; }

// ------------------------------------------------------------------ parser

class Parser {
public:
    explicit Parser(Document& doc) : doc_(doc) {
        mods_.emplace_back();
        mods_[0].name = kRootModule;
    }

    void run(const std::string& text);
    void finish(const std::string& config);

private:
    // Elements of one module (root or `section`). Node-creating elements (nodes, cinecam, wheels) are global.
    struct Module {
        std::string name;
        Document d;
        bool has_globals = false, has_minimass = false, has_collision = false, has_speedo = false;
    };
    struct PendingName {
        std::string name;
        int num, line, mod;
    };
    struct RefNote {
        int value, line, mod;
    };
    struct WheelDetacher {
        int wheel, group, line, mod;
    };

    Document& doc_;
    std::vector<Module> mods_;
    std::vector<int> node_mod_, wheel_mod_; // module that created each node slot / wheel
    int cur_ = 0;
    Kw block_ = Kw::NONE;
    int line_no_ = 0;
    bool have_title_ = false;
    sv line_;
    std::string fixed_;
    sv tok_[kMaxTokens];
    int ntok_ = 0;
    int suppressed_ = 0;

    // sticky state
    BeamDefaults bd_;
    NodeDefaults nd_;
    bool adv_deform_ = false;
    int detacher_ = 0;
    int shell_mat_ = 0; // (BeamLab) the shells' material in effect: 0 the globals', k shell_materials[k - 1]
    int frame_sec_ = -1; // (BeamLab) the frame elements' section in effect (frame_sections of the module; -1: none yet)
    int fem_shell_ = -1; // (BeamLab) the triangle elements' shell in effect (fem_shells of the module; -1: none yet)
    float default_minimass_ = -1.0f;
    bool mm_double_sided_ = false;
    bool has_submesh_ = false;
    SubmeshDef submesh_;

    // node references
    bool any_named_ = false;
    std::unordered_map<std::string, int> names_;
    std::vector<PendingName> pending_;
    std::vector<RefNote> far_refs_;
    std::vector<WheelDetacher> wheel_detachers_;

    Document& md() { return mods_[cur_].d; }

    void warn_at(int line, const char* fmt, ...) __attribute__((format(printf, 3, 4)));
    bool need(int n);

    float f(int i) const { return parse_real(tok_[i]); }
    int ival(int i);
    int node_num(sv t) const;
    int ref(sv t);
    int ref(int i) { return ref(tok_[i]); }
    std::string flags(sv t, const char* valid);

    void process_line(sv raw);
    void keyword(Kw kw);
    void data_line();
    void begin_block(Kw kw);
    void flush_submesh();
    void change_module(const std::string& name);

    void dir_set_beam_defaults();
    void dir_set_beam_defaults_scale();
    void dir_set_node_defaults();
    void dir_forset();

    void parse_node();
    void parse_beam();
    void parse_shock();
    void parse_hydro();
    void parse_command();
    void parse_cinecam();
    void parse_wheel();
    void parse_engine();
    void parse_torquecurve();
    void parse_axle();
    bool parse_axle_item(sv item, AxleDef& a, sv ids[4], int& nids);
    void parse_flexbody();
    void parse_prop();
    void parse_managed_material();
    void parse_guisetting();

    void merge(const Module& m);
    void resolve_refs();
    void expand_forsets();
    void drop_inactive_modules(int sel);
    template <class F> void for_each_ref(F&& fix);
};

void Parser::warn_at(int line, const char* fmt, ...) {
    if ((int)doc_.warnings.size() >= kMaxWarnings) {
        suppressed_++;
        return;
    }
    char buf[640];
    int n = line > 0 ? snprintf(buf, sizeof(buf), "line %d: ", line) : 0;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf + n, sizeof(buf) - (size_t)n, fmt, ap);
    va_end(ap);
    doc_.warnings.emplace_back(buf);
}

bool Parser::need(int n) {
    if (ntok_ >= n) return true;
    Kw ctx = block_;
    Kw kw = identify(line_);
    if (kw != Kw::NONE) ctx = kw;
    warn_at(line_no_, "(%s) not enough arguments (got %d, need %d), line skipped", kw_name(ctx), ntok_, n);
    return false;
}

// strtol semantics (RoR GetArgLong): garbage -> 0, trailing characters are accepted with a warning.
int Parser::ival(int i) {
    long long v;
    size_t used;
    if (!scan_int(tok_[i], v, used)) {
        warn_at(line_no_, "argument %d '%.*s' is not a valid integer, using 0", i + 1, (int)tok_[i].size(),
                tok_[i].data());
        return 0;
    }
    if (used != tok_[i].size())
        warn_at(line_no_, "integer argument %d '%.*s' has invalid trailing characters", i + 1, (int)tok_[i].size(),
                tok_[i].data());
    if (v > INT32_MAX || v < INT32_MIN) return 0;
    return (int)v;
}

// Ogre parseInt: numeric prefix, garbage -> 0, negative -> abs.
int Parser::node_num(sv t) const {
    long long v;
    size_t used;
    if (!scan_int(t, v, used)) return 0;
    if (v > INT32_MAX || v < INT32_MIN) return 0; // istream overflow -> default
    if (v < 0) v = -v;
    return (int)std::min<long long>(v, kMaxNumRef);
}

int Parser::ref(sv t) {
    if (any_named_) {
        pending_.push_back({std::string(t), node_num(t), line_no_, cur_});
        return kPendingRef + (int)pending_.size() - 1;
    }
    int v = node_num(t);
    if (v >= (int)doc_.nodes.size()) far_refs_.push_back({v, line_no_, cur_});
    return v;
}

// Valid option letters of token `t` (deduplicated, in order); unknown letters produce one warning.
std::string Parser::flags(sv t, const char* valid) {
    std::string out, bad;
    for (char c : t) {
        if (c && strchr(valid, c)) {
            if (!has_char(out, c)) out += c;
        } else {
            bad += c;
        }
    }
    if (!bad.empty())
        warn_at(line_no_, "(%s) ignoring invalid option(s) '%s' in '%.*s'", kw_name(block_), bad.c_str(), (int)t.size(),
                t.data());
    return out;
}

void Parser::run(const std::string& text) {
    const char* p = text.data();
    const char* end = p + text.size();
    while (p < end) {
        const char* nl = (const char*)memchr(p, '\n', (size_t)(end - p));
        const char* le = nl ? nl : end;
        line_no_++;
        if (const char* z = (const char*)memchr(p, 0, (size_t)(le - p))) le = z; // RoR uses strnlen
        else if (le > p && le[-1] == '\r') le--;
        const size_t len = (size_t)(le - p);
        for (size_t off = 0; off < len; off += kMaxLineLen) process_line(sv(p + off, std::min(len - off, kMaxLineLen)));
        p = nl ? nl + 1 : end;
    }
}

void Parser::process_line(sv raw) {
    size_t b = 0;
    while (b < raw.size() && is_blank(raw[b])) b++;
    sv line = raw.substr(b);
    if (line.empty() || line[0] == ';' || line[0] == '/') return;
    if (!is_ascii(line)) {
        fixed_ = utf8_fix(line);
        line = fixed_;
    }
    line_ = line;

    if (!have_title_) {
        doc_.title = std::string(trim_blanks(line));
        have_title_ = true;
        return;
    }

    // Text blocks: only a terminator ends them (RoR would also react to other keywords here; see spec 1.7).
    if (block_ == Kw::DESCRIPTION || block_ == Kw::COMMENT) {
        Kw kw = identify(line);
        if (kw == Kw::END || kw == Kw::END_DESCRIPTION || kw == Kw::END_COMMENT) {
            begin_block(Kw::NONE);
        } else if (block_ == Kw::DESCRIPTION) {
            if (!doc_.description.empty()) doc_.description += '\n';
            doc_.description.append(line.data(), line.size());
        }
        return;
    }

    ntok_ = tokenize(line, tok_);
    Kw kw = identify(line);
    if (kw != Kw::NONE) keyword(kw);
    else data_line();
}

void Parser::begin_block(Kw kw) {
    if (kw == Kw::NONE || kw == Kw::CAMERARAIL) flush_submesh();
    block_ = kw;
}

void Parser::flush_submesh() {
    if (!has_submesh_) return;
    md().submeshes.push_back(std::move(submesh_));
    submesh_ = SubmeshDef();
    has_submesh_ = false;
}

void Parser::change_module(const std::string& name) {
    begin_block(Kw::NONE); // flushes the staged submesh into the module being left
    // (the frame section in effect goes on in the next module, as the beam defaults do)
    const bool carry = frame_sec_ >= 0 && frame_sec_ < (int)md().frame_sections.size();
    const Document::FrameSectionDef sec = carry ? md().frame_sections[frame_sec_] : Document::FrameSectionDef();
    const bool carry_shell = fem_shell_ >= 0 && fem_shell_ < (int)md().fem_shells.size();
    const Document::FemShellDef shell = carry_shell ? md().fem_shells[fem_shell_] : Document::FemShellDef();
    auto enter = [&](int m) {
        cur_ = m;
        frame_sec_ = -1;
        fem_shell_ = -1;
        if (carry_shell) {
            md().fem_shells.push_back(shell);
            fem_shell_ = (int)md().fem_shells.size() - 1;
        }
        if (carry) {
            md().frame_sections.push_back(sec);
            frame_sec_ = (int)md().frame_sections.size() - 1;
        }
    };
    for (size_t i = 0; i < mods_.size(); i++) {
        if (mods_[i].name == name) {
            enter((int)i);
            return;
        }
    }
    mods_.emplace_back();
    mods_.back().name = name;
    enter((int)mods_.size() - 1);
}

void Parser::keyword(Kw kw) {
    switch (kw) {
    // flags: the block is unchanged
    case Kw::DISABLEDEFAULTSOUNDS:
    case Kw::FORWARDCOMMANDS:
    case Kw::HIDEINCHOOSER:
    case Kw::IMPORTCOMMANDS:
    case Kw::LOCKGROUP_DEFAULT_NOLOCK:
    case Kw::RESCUER:
    case Kw::SLIDENODE_CONNECT_INSTANTLY:
        return;
    case Kw::ENABLE_ADVANCED_DEFORMATION:
        adv_deform_ = true;
        return;
    case Kw::ROLLON:
        doc_.rollon = true;
        return;

    case Kw::SET_FRAME_SECTION: {
        // (BeamLab) the section of the frame elements (beams with option F) that follow
        Document::FrameSectionDef d;
        if (ntok_ > 1) d.material = std::string(tok_[1]);
        if (ntok_ > 2) d.shape = std::string(tok_[2]);
        if (ntok_ > 3) d.outer = f(3);
        if (ntok_ > 4) d.wall = f(4);
        if (ntok_ > 5) {
            const std::string e(tok_[5]);
            if (e == "pinned1") d.end_a = phys::FJ_BALL;
            else if (e == "pinned2") d.end_b = phys::FJ_BALL;
            else if (e.find('/') != std::string::npos) d.end_a = phys::frame_joint(e.substr(0, e.find('/'))), d.end_b = phys::frame_joint(e.substr(e.find('/') + 1));
            else d.end_a = d.end_b = phys::frame_joint(e);
        }
        if (ntok_ > 6 && f(6) > 0) d.joint_k = f(6);
        if (ntok_ > 7 && f(7) > 0) d.brk = f(7);
        if (ntok_ > 8 && f(8) > 0) d.joint_damp = f(8);
        if (!(d.outer > 0)) {
            warn_at(line_no_, "set_frame_section: outer size %g, using 0.04", d.outer);
            d.outer = 0.04f;
        }
        md().frame_sections.push_back(d);
        frame_sec_ = (int)md().frame_sections.size() - 1;
        return;
    }
    case Kw::SET_FEM_SHELL: {
        // (BeamLab) the shell of the triangle elements (fem_tris) that follow: material, thickness m[, r, g, b]
        Document::FemShellDef d;
        if (ntok_ > 1) d.material = std::string(tok_[1]);
        if (ntok_ > 2) d.thickness = f(2);
        if (ntok_ > 5) d.color = vec3(f(3), f(4), f(5));
        if (!(d.thickness > 0)) {
            warn_at(line_no_, "set_fem_shell: thickness %g, using 0.001", d.thickness);
            d.thickness = 0.001f;
        }
        md().fem_shells.push_back(d);
        fem_shell_ = (int)md().fem_shells.size() - 1;
        return;
    }
    case Kw::SET_SHELL_MATERIAL: {
        // (BeamLab) the material of the shells that follow; alone (or "default"): the globals' again
        if (ntok_ < 2 || tok_[1] == "default") {
            shell_mat_ = 0;
            return;
        }
        Document::ShellMaterialDef m;
        m.name = std::string(tok_[1]);
        if (ntok_ > 2) m.material = std::string(tok_[2]);
        if (ntok_ > 3) m.kg_m2 = f(3);
        if (ntok_ > 4) m.thickness = f(4);
        if (ntok_ > 7) m.color = vec3(f(5), f(6), f(7));
        if (ntok_ > 8) m.max_level = ival(8);
        auto& list = md().shell_materials;
        int idx = -1;
        for (int i = 0; i < (int)list.size(); i++)
            if (list[i].name == m.name) idx = i;
        if (idx < 0) {
            list.push_back(m);
            idx = (int)list.size() - 1;
        } else {
            list[idx] = m;
        }
        shell_mat_ = idx + 1;
        return;
    }
    // directives with arguments: the block is unchanged unless noted
    case Kw::ADD_ANIMATION:
    case Kw::CRUISECONTROL:
    case Kw::DEFAULT_SKIN:
    case Kw::EXTCAMERA:
    case Kw::SET_INERTIA_DEFAULTS:
    case Kw::SET_SKELETON_SETTINGS:
    case Kw::SUBMESH_GROUNDMODEL:
        return;
    case Kw::SPEEDLIMITER:
        if (ntok_ >= 2) doc_.speed_limit = f(1);
        return;
    case Kw::TRACTIONCONTROL: {
        // TractionControl force, wheelslip [, fade [, pulse [, mode: ON|OFF ...]]]
        auto& t = doc_.traction;
        t.present = true;
        int n = 1;
        float v[4] = {1.0f, 0.25f, 0.0f, 2000.0f};
        for (int i = 1; i < ntok_ && n <= 4; i++) {
            if (!tok_[i].empty() && (isdigit((unsigned char)tok_[i][0]) || tok_[i][0] == '.' || tok_[i][0] == '-')) v[n++ - 1] = parse_real(tok_[i]);
            else break;
        }
        t.regulation = std::clamp(v[0], 1.0f, 20.0f);
        t.wheelslip = std::max(0.0f, v[1]);
        t.fade = v[2];
        t.pulse = v[3];
        for (int i = 1; i + 1 < ntok_; i++)
            if (iequals(tok_[i], "mode")) t.on = iequals(tok_[i + 1], "ON");
        return;
    }
    case Kw::ANTILOCKBRAKES: {
        // AntiLockBrakes force, min_speed_kmh [, pulse [, mode: ON|OFF ...]]
        auto& a = doc_.antilock;
        a.present = true;
        int n = 1;
        float v[3] = {1.0f, 0.0f, 2000.0f};
        for (int i = 1; i < ntok_ && n <= 3; i++) {
            if (!tok_[i].empty() && (isdigit((unsigned char)tok_[i][0]) || tok_[i][0] == '.' || tok_[i][0] == '-')) v[n++ - 1] = parse_real(tok_[i]);
            else break;
        }
        a.regulation = std::clamp(v[0], 1.0f, 20.0f);
        a.min_speed_kmh = v[1];
        a.pulse = v[2];
        for (int i = 1; i + 1 < ntok_; i++)
            if (iequals(tok_[i], "mode")) a.on = iequals(tok_[i + 1], "ON");
        return;
    }
    case Kw::FORVERT:
        warn_at(line_no_, "'forvert' is not supported, ignored");
        return;
    case Kw::AUTHOR:
        if (!need(2)) return;
        doc_.authors.emplace_back(ntok_ > 3 ? tok_[3] : tok_[1]);
        block_ = Kw::NONE;
        return;
    case Kw::FILEINFO:
        if (!need(2)) return;
        block_ = Kw::NONE;
        return;
    case Kw::FILEFORMATVERSION:
        if (!need(2)) return;
        doc_.file_format_version = ival(1);
        block_ = Kw::NONE;
        return;
    case Kw::GUID:
        if (!need(2)) return;
        doc_.guid = std::string(tok_[1]);
        return;
    case Kw::DETACHER_GROUP:
        if (!need(2)) return;
        detacher_ = tok_[1] == "end" ? 0 : ival(1);
        return;
    case Kw::SET_BEAM_DEFAULTS:
        dir_set_beam_defaults();
        return;
    case Kw::SET_BEAM_DEFAULTS_SCALE:
        dir_set_beam_defaults_scale();
        return;
    case Kw::SET_NODE_DEFAULTS:
        dir_set_node_defaults();
        return;
    case Kw::SET_DEFAULT_MINIMASS:
        if (!need(2)) return;
        default_minimass_ = f(1);
        return;
    case Kw::SET_COLLISION_RANGE: {
        if (!need(2)) return;
        float r = f(1);
        md().collision_range = r >= 0 ? r : -1.0f;
        mods_[cur_].has_collision = true;
        return;
    }
    case Kw::SET_MANAGEDMATERIALS_OPTIONS:
        if (!need(2)) return;
        mm_double_sided_ = tok_[1][0] != '0';
        return;
    case Kw::FLEXBODY_CAMERA_MODE:
    case Kw::PROP_CAMERA_MODE: {
        if (!need(2)) return;
        bool fb = kw == Kw::FLEXBODY_CAMERA_MODE;
        if (fb ? md().flexbodies.empty() : md().props.empty()) {
            warn_at(line_no_, "'%s' must come after a %s, ignored", kw_name(kw), fb ? "flexbody" : "prop");
            return;
        }
        (fb ? md().flexbodies.back().camera_mode : md().props.back().camera_mode) = ival(1);
        return;
    }
    case Kw::FORSET:
        dir_forset();
        return;
    case Kw::BACKMESH:
        if (has_submesh_) submesh_.backmesh = true;
        else warn_at(line_no_, "'backmesh' must come after 'submesh', ignored");
        return;
    case Kw::SUBMESH:
        begin_block(Kw::NONE);
        has_submesh_ = true;
        return;
    case Kw::SECTION: {
        if (!need(3)) return;
        std::string name(tok_[2]);
        if (name == mods_[cur_].name) {
            warn_at(line_no_, "attempt to re-enter the current module '%s', ignored", name.c_str());
            return;
        }
        change_module(name);
        return;
    }
    case Kw::END_SECTION:
        if (cur_ == 0) {
            warn_at(line_no_, "misplaced 'end_section' (already in root module), ignored");
            return;
        }
        change_module(kRootModule);
        return;

    case Kw::END:
    case Kw::END_COMMENT:
    case Kw::END_DESCRIPTION:
        begin_block(Kw::NONE);
        return;

    case Kw::ENVMAP:
    case Kw::HOOKGROUP:
    case Kw::NODECOLLISION:
    case Kw::RIGIDIFIERS:
        return;

    default: // every block keyword, including handler-less sectionconfig / SlopeBrake / set_shadows
        begin_block(kw);
        return;
    }
}

void Parser::data_line() {
    switch (block_) {
    case Kw::NODES:
    case Kw::NODES2: parse_node(); return;
    case Kw::BEAMS: parse_beam(); return;
    case Kw::SHOCKS:
    case Kw::SHOCKS2:
    case Kw::SHOCKS3: parse_shock(); return;
    case Kw::HYDROS: parse_hydro(); return;
    case Kw::COMMANDS:
    case Kw::COMMANDS2: parse_command(); return;
    case Kw::CINECAM: parse_cinecam(); return;
    case Kw::WHEELS:
    case Kw::WHEELS2:
    case Kw::MESHWHEELS:
    case Kw::MESHWHEELS2:
    case Kw::FLEXBODYWHEELS: parse_wheel(); return;
    case Kw::ENGINE: parse_engine(); return;
    case Kw::TORQUECURVE: parse_torquecurve(); return;
    case Kw::AXLES: parse_axle(); return;
    case Kw::FLEXBODIES: parse_flexbody(); return;
    case Kw::PROPS: parse_prop(); return;
    case Kw::MANAGEDMATERIALS: parse_managed_material(); return;
    case Kw::GUISETTINGS: parse_guisetting(); return;
    case Kw::CAMERAS: {
        if (!need(3)) return;
        CameraDef c;
        c.center = ref(0);
        c.back = ref(1);
        c.left = ref(2);
        md().cameras.push_back(c);
        return;
    }
    case Kw::GLOBALS: {
        if (!need(2)) return;
        Module& m = mods_[cur_];
        m.has_globals = true;
        m.d.dry_mass = f(0);
        m.d.cargo_mass = std::max(f(1), 0.0f); // RoR doesn't special-case negatives (assets use -1)
        if (ntok_ > 2) m.d.cab_material = std::string(tok_[2]);
        return;
    }
    case Kw::ENGOPTION: {
        if (!need(1)) return;
        EngoptionDef e;
        e.present = true;
        e.inertia = f(0);
        if (ntok_ > 1) {
            char c = tok_[1][0];
            e.type = (c == 't' || c == 'c' || c == 'e') ? c : 't';
        }
        float* fields[] = {&e.clutch_force, &e.f4, &e.f5, &e.post_shift_time, &e.stall_rpm, &e.idle_rpm,
                           &e.max_idle_mix, &e.min_idle_mix, &e.braking_torque};
        for (int i = 0; i < 9 && i + 2 < ntok_; i++) *fields[i] = f(i + 2);
        md().engoption = e;
        return;
    }
    case Kw::BRAKES: {
        if (!need(1)) return;
        md().brakes.force = f(0);
        md().brakes.parking = ntok_ > 1 ? f(1) : -1.0f;
        md().has_brakes = true;
        return;
    }
    case Kw::TEXCOORDS: {
        if (!need(3)) return;
        if (!has_submesh_) {
            warn_at(line_no_, "texcoords must come after 'submesh', line skipped");
            return;
        }
        submesh_.texcoords.push_back({ref(0), f(1), f(2)});
        return;
    }
    case Kw::CAB: {
        if (!need(3)) return;
        if (!has_submesh_) {
            warn_at(line_no_, "cab must come after 'submesh', line skipped");
            return;
        }
        CabDef c;
        c.n1 = ref(0);
        c.n2 = ref(1);
        c.n3 = ref(2);
        if (ntok_ > 3) c.options = flags(tok_[3], kCabOpts);
        submesh_.cabs.push_back(std::move(c));
        return;
    }
    case Kw::CONTACTERS:
        if (!need(1)) return;
        md().contacters.push_back(ref(0));
        return;
    case Kw::JOINTS: { // (BeamLab) parent, child [, stiffness N/m [, break force N [, ref x node, ref y node]]]
        if (!need(2)) return;
        Document::JointDef j;
        j.parent = ref(0);
        j.child = ref(1);
        if (ntok_ > 2) j.k = f(2);
        if (ntok_ > 3) j.brk = f(3);
        if (ntok_ > 5) j.ref_x = ref(4), j.ref_y = ref(5);
        j.line = line_no_;
        md().joints.push_back(j);
        return;
    }
    case Kw::WELDS: { // (BeamLab) anchor, sheet node, radius m, break force N[, stiffness N/m]
        if (!need(2)) return;
        Document::WeldDef w;
        w.anchor = ref(0);
        w.node = ref(1);
        if (ntok_ > 2) w.radius = f(2);
        if (ntok_ > 3) w.brk = f(3);
        if (ntok_ > 4) w.k = f(4);
        if (ntok_ > 6) w.anchor2 = ref(5), w.t = f(6);
        md().welds.push_back(w);
        return;
    }
    case Kw::MOUNTS: { // (BeamLab) node a, node b, break force N[, stiffness N/m[, turning damping N m s/rad[, kind[, parameter]]]]
        if (!need(2)) return;
        Document::MountDef m;
        m.a = ref(0);
        m.b = ref(1);
        if (ntok_ > 2) m.brk = f(2);
        if (ntok_ > 3) m.k = f(3);
        if (ntok_ > 4) m.damp = f(4);
        if (ntok_ > 5) {
            const char c = tok_[5].empty() ? 'p' : (char)std::tolower((unsigned char)tok_[5][0]);
            if (c == 'p' || c == 'c' || c == 'h' || c == 's' || c == 'r') m.kind = c;
            else warn_at(line_no_, "mount kind '%.*s' unknown, using a point", (int)tok_[5].size(), tok_[5].data());
            if (ntok_ > 6) {
                if (m.kind == 'h') m.b2 = ref(6);
                else m.param = f(6);
            }
            if (m.kind == 'h' && m.b2 < 0) warn_at(line_no_, "a hinge mount needs its second node, using a point"), m.kind = 'p';
        }
        md().mounts.push_back(m);
        return;
    }
    case Kw::FEM_TRIS: { // (BeamLab) triangle elements of the FEM frame: n1, n2, n3
        if (!need(3)) return;
        if (fem_shell_ < 0 || fem_shell_ >= (int)md().fem_shells.size()) {
            md().fem_shells.push_back(Document::FemShellDef());
            fem_shell_ = (int)md().fem_shells.size() - 1;
        }
        md().fem_tris.push_back({ref(0), ref(1), ref(2), fem_shell_});
        return;
    }
    case Kw::SHELLS: // (BeamLab) the cab triangles that are triangle elements: n1, n2, n3
        if (!need(3)) return;
        md().shells.push_back({ref(0), ref(1), ref(2), shell_mat_});
        return;
    case Kw::FIXES:
        if (ntok_ > 0) md().fixes.push_back(ref(0));
        return;
    case Kw::SLIDENODES: {
        // slide, rail [, rail ...] [, S<spring> B<break> T<tolerance> R<rate> G<group> D<dist> C<a|f|s|n>]
        if (!need(2)) return;
        SlideNodeDef s;
        s.node = ref(0);
        int i = 1;
        for (; i < ntok_; i++) {
            sv t = tok_[i];
            if (t.empty() || isalpha((unsigned char)t[0])) break;
            s.rail.push_back(ref(t));
        }
        for (; i < ntok_; i++) {
            sv t = tok_[i];
            if (t.size() < 2) continue;
            float v = parse_real(t.substr(1));
            switch (toupper((unsigned char)t[0])) {
            case 'S': s.spring = v; break;
            case 'B': s.break_force = v; break;
            case 'T': s.tolerance = v; break;
            case 'R': s.attach_rate = v; break;
            case 'G': s.railgroup = (int)v; break;
            case 'D': s.attach_dist = v; break;
            default: break;
            }
        }
        md().slidenodes.push_back(std::move(s));
        return;
    }
    case Kw::RAILGROUPS: {
        if (!need(3)) return;
        RailGroupDef g;
        g.id = (int)f(0);
        for (int i = 1; i < ntok_; i++) g.nodes.push_back(ref(tok_[i]));
        md().railgroups.push_back(std::move(g));
        return;
    }
    case Kw::ROPES: {
        if (!need(2)) return;
        RopeDef r;
        r.root = ref(0);
        r.end = ref(1);
        if (ntok_ > 2 && tok_[2][0] == 'i') r.options = "i";
        r.bd = bd_;
        md().ropes.push_back(std::move(r));
        return;
    }
    case Kw::TIES: {
        if (!need(5)) return;
        TieDef t;
        t.root = ref(0);
        t.max_reach = f(1);
        t.auto_shorten_rate = f(2);
        t.min_length = f(3);
        t.max_length = f(4);
        if (ntok_ > 5) t.options = flags(tok_[5], kTieOpts);
        if (ntok_ > 6) t.max_stress = f(6);
        md().ties.push_back(std::move(t));
        return;
    }
    case Kw::MINIMASS: {
        if (!need(1)) return;
        Module& m = mods_[cur_];
        m.has_minimass = true;
        m.d.minimass = f(0);
        m.d.minimass_skip_loaded = false;
        if (ntok_ > 1) {
            char c = tok_[1][0];
            if (c == 'l') m.d.minimass_skip_loaded = true;
            else if (c != 'n')
                warn_at(line_no_, "invalid minimass option '%.*s', using 'n'", (int)tok_[1].size(), tok_[1].data());
        }
        block_ = Kw::NONE; // single-line block
        return;
    }
    case Kw::WHEELDETACHERS:
        if (!need(2)) return;
        wheel_detachers_.push_back({ival(0), ival(1), line_no_, cur_});
        return;
    default: // no block, or a block this loader doesn't need
        return;
    }
}

// ------------------------------------------------------------------ directives

void Parser::dir_set_beam_defaults() {
    if (!need(2)) return;
    BeamDefaults d = bd_;
    const BeamDefaults def;
    d.adv_deform = adv_deform_;
    d.user_defined = true;
    d.spring = f(1);
    if (ntok_ > 2) d.damp = f(2);
    if (ntok_ > 3) d.deform = f(3);
    if (ntok_ > 4) d.brk = f(4);
    if (ntok_ > 5) d.diameter = f(5);
    if (ntok_ > 6) d.material = std::string(tok_[6]);
    if (ntok_ > 7) {
        d.plastic = f(7);
        if (d.plastic >= 0) d.plastic_given = true;
    }
    if (d.spring < 0) d.spring = def.spring;
    if (d.damp < 0) d.damp = def.damp;
    if (d.deform < 0) d.deform = def.deform;
    if (d.brk < 0) d.brk = def.brk;
    if (d.diameter < 0) d.diameter = def.diameter;
    if (d.plastic < 0) d.plastic = def.plastic;
    bd_ = std::move(d);
}

void Parser::dir_set_beam_defaults_scale() {
    if (!need(5)) return;
    bd_.scale_spring = f(1);
    bd_.scale_damp = f(2);
    bd_.scale_deform = f(3);
    bd_.scale_break = f(4);
}

void Parser::dir_set_node_defaults() {
    if (!need(2)) return;
    const NodeDefaults def;
    float lw = f(1);
    float fr = ntok_ > 2 ? f(2) : -1.0f;
    float vol = ntok_ > 3 ? f(3) : -1.0f;
    float surf = ntok_ > 4 ? f(4) : -1.0f;
    nd_.load_weight = lw < 0 ? def.load_weight : lw;
    nd_.friction = fr < 0 ? def.friction : fr;
    nd_.volume = vol < 0 ? def.volume : vol;
    nd_.surface = surf < 0 ? def.surface : surf;
    if (ntok_ > 5) nd_.options = flags(tok_[5], kNodeOpts);
}

// RoR ProcessForsetLine, quirks preserved: separators after `forset` are optional, items are `a` or `a-b`
// (inclusive) split on ',', values use strtoul (garbage/empty item -> node 0, so a trailing ',' adds node 0).
// Ranges are stored as flattened (first,last) pairs and expanded in expand_forsets() once the node count is known.
void Parser::dir_forset() {
    if (md().flexbodies.empty()) {
        warn_at(line_no_, "ignoring 'forset': no matching flexbody");
        return;
    }
    FlexbodyDef& fb = md().flexbodies.back();
    fb.has_forset = true;
    auto strtoul_like = [](sv s) -> int {
        size_t i = 0;
        while (i < s.size() && isspace((unsigned char)s[i])) i++;
        long long v;
        size_t used;
        if (!scan_int(s.substr(i), v, used)) return 0;
        return (v < 0 || v > INT32_MAX) ? INT32_MAX : (int)v; // negative wraps to a huge (invalid) index in RoR
    };
    sv s = line_;
    size_t p = 6;
    while (p < s.size() && (s[p] == ' ' || s[p] == ':' || s[p] == ',')) p++;
    for (;;) {
        size_t e = p;
        while (e < s.size() && s[e] != '-' && s[e] != ',') e++;
        int a = strtoul_like(s.substr(std::min(p, s.size()), e - std::min(p, s.size())));
        int b = a;
        char endwas = e < s.size() ? s[e] : 0;
        if (endwas == '-') {
            p = e + 1;
            e = p;
            while (e < s.size() && s[e] != ',') e++;
            b = strtoul_like(s.substr(std::min(p, s.size()), e - std::min(p, s.size())));
            endwas = e < s.size() ? s[e] : 0;
        }
        fb.forset.push_back(a);
        fb.forset.push_back(b);
        if (!endwas) break;
        p = e + 1;
    }
}

// ------------------------------------------------------------------ elements

void Parser::parse_node() {
    if (!need(4)) return;
    NodeDef n;
    if (block_ == Kw::NODES2) {
        any_named_ = true; // set even if the line is dropped below (as RoR does)
        n.name = std::string(tok_[0]);
        if (names_.count(n.name)) {
            warn_at(line_no_, "duplicate node name '%s', node ignored", n.name.c_str());
            return;
        }
    } else {
        int id = ival(0);
        if (id != (int)doc_.nodes.size()) {
            warn_at(line_no_, "lost sync in node numbers: got node %d, expected %d, node ignored", id,
                    (int)doc_.nodes.size());
            return;
        }
    }
    n.pos = vec3(f(1), f(2), f(3));
    std::string own = ntok_ > 4 ? flags(tok_[4], kNodeOpts) : std::string();
    bool own_weight = false;
    float weight = 0;
    if (ntok_ > 5) {
        if (has_char(own, 'l')) {
            own_weight = true;
            weight = f(5);
        } else {
            warn_at(line_no_, "node has a load weight but no 'l' option, weight ignored");
        }
    }
    n.options = own;
    for (char c : nd_.options)
        if (!has_char(n.options, c)) n.options += c;
    bool l = has_char(n.options, 'l');
    n.loaded = l || nd_.load_weight >= 0;
    n.load_weight = (l && own_weight) ? weight : (nd_.load_weight >= 0 ? nd_.load_weight : -1.0f);
    n.defaults = nd_;
    n.minimass = default_minimass_;
    n.detacher_group = detacher_;
    n.line = line_no_;

    int slot = (int)doc_.nodes.size();
    if (!n.name.empty()) names_.emplace(n.name, slot);
    doc_.nodes.push_back({NodeSlot::EXPLICIT, (int)doc_.nodes_explicit.size(), 0});
    node_mod_.push_back(cur_);
    doc_.nodes_explicit.push_back(std::move(n));
}

void Parser::parse_beam() {
    if (!need(2)) return;
    BeamDef b;
    b.n1 = ref(0);
    b.n2 = ref(1);
    if (ntok_ > 2) b.options = flags(tok_[2], kBeamOpts);
    if (ntok_ > 3 && has_char(b.options, 's')) {
        int k = ival(3); // RoR reads the support limit as an integer
        b.support_limit = k > 0 ? (float)k : 4.0f;
    }
    b.bd = bd_;
    b.detacher_group = detacher_;
    b.line = line_no_;
    if (has_char(b.options, 'F')) { // (BeamLab) a frame element: the section in effect (a 40 x 2 mm steel tube if none)
        if (frame_sec_ < 0 || frame_sec_ >= (int)md().frame_sections.size()) {
            md().frame_sections.push_back(Document::FrameSectionDef());
            frame_sec_ = (int)md().frame_sections.size() - 1;
        }
        b.frame = frame_sec_;
        // (its own joints: `n1, n2, F, joint a[, joint b]`)
        if (ntok_ > 3) b.end_a = phys::frame_joint(std::string(tok_[3])), b.end_b = b.end_a;
        if (ntok_ > 4) b.end_b = phys::frame_joint(std::string(tok_[4]));
    }
    md().beams.push_back(std::move(b));
}

void Parser::parse_shock() {
    ShockDef s;
    s.type = block_ == Kw::SHOCKS ? 1 : block_ == Kw::SHOCKS2 ? 2 : 3;
    const int min = s.type == 1 ? 7 : s.type == 2 ? 13 : 15;
    if (!need(min)) return;
    s.n1 = ref(0);
    s.n2 = ref(1);
    if (s.type == 1) {
        s.spring = f(2);
        s.damp = f(3);
    } else if (s.type == 2) {
        s.spring_in = f(2);
        s.damp_in = f(3);
        s.prog_spring_in = f(4);
        s.prog_damp_in = f(5);
        s.spring_out = f(6);
        s.damp_out = f(7);
        s.prog_spring_out = f(8);
        s.prog_damp_out = f(9);
    } else {
        s.spring_in = f(2);
        s.damp_in = f(3);
        s.damp_in_slow = f(4);
        s.split_vel_in = f(5);
        s.damp_in_fast = f(6);
        s.spring_out = f(7);
        s.damp_out = f(8);
        s.damp_out_slow = f(9);
        s.split_vel_out = f(10);
        s.damp_out_fast = f(11);
    }
    s.shortbound = f(min - 3);
    s.longbound = f(min - 2);
    s.precompression = f(min - 1);
    if (ntok_ > min) s.options = flags(tok_[min], s.type == 1 ? kShockOpts : s.type == 2 ? kShock2Opts : kShock3Opts);
    s.bd = bd_;
    s.detacher_group = detacher_;
    s.line = line_no_;
    md().shocks.push_back(std::move(s));
}

void Parser::parse_hydro() {
    if (!need(3)) return;
    HydroDef h;
    h.n1 = ref(0);
    h.n2 = ref(1);
    h.factor = f(2);
    if (ntok_ > 3) h.options = flags(tok_[3], kHydroOpts);
    if (h.options.empty()) h.options = "n"; // no flags -> steering input
    h.bd = bd_;
    h.detacher_group = detacher_;
    h.line = line_no_;
    md().hydros.push_back(std::move(h));
}

void Parser::parse_command() {
    const bool c2 = block_ == Kw::COMMANDS2;
    const int min = c2 ? 8 : 7;
    if (!need(min)) return;
    CommandDef c;
    int p = 0;
    c.n1 = ref(p++);
    c.n2 = ref(p++);
    c.rate_short = f(p++);
    c.rate_long = c2 ? f(p++) : c.rate_short;
    c.shortbound = f(p++);
    c.longbound = f(p++);
    c.key_contract = ival(p++);
    c.key_extend = ival(p++);
    if (ntok_ > p) {
        // only one of c/p/o survives: the first one in the string
        std::string o = flags(tok_[p++], kCommandOpts), out;
        char winner = 0;
        for (char ch : o) {
            bool mode = ch == 'c' || ch == 'p' || ch == 'o';
            if (mode && winner) {
                warn_at(line_no_, "command already has mode '%c', ignoring flag '%c'", winner, ch);
                continue;
            }
            if (mode) winner = ch;
            out += ch;
        }
        c.options = out;
    }
    if (ntok_ > p) c.description = std::string(tok_[p++]);
    if (ntok_ > p) p += 4; // inertia: start/stop delay, start/stop function
    if (ntok_ > p) c.affect_engine = f(p++);
    if (ntok_ > p) c.needs_engine = parse_bool(tok_[p++]);
    c.bd = bd_;
    c.detacher_group = detacher_;
    c.line = line_no_;
    md().commands.push_back(std::move(c));
}

void Parser::parse_cinecam() {
    if (!need(11)) return;
    CinecamDef c;
    c.pos = vec3(f(0), f(1), f(2));
    for (int i = 0; i < 8; i++) c.nodes[i] = ref(3 + i);
    if (ntok_ > 11) c.spring = f(11);
    if (ntok_ > 12) c.damp = f(12);
    if (ntok_ > 13 && f(13) > 0) c.node_mass = f(13); // garbage (e.g. a trailing ";comment") parses as 0
    c.bd = bd_;
    c.nd = nd_;
    doc_.nodes.push_back({NodeSlot::CINECAM, (int)doc_.cinecams.size(), 0});
    node_mod_.push_back(cur_);
    doc_.cinecams.push_back(std::move(c));
}

void Parser::parse_wheel() {
    WheelDef w;
    int min = 16;
    switch (block_) {
    case Kw::WHEELS: w.type = WheelDef::WHEELS; min = 14; break;
    case Kw::WHEELS2: w.type = WheelDef::WHEELS2; min = 17; break;
    case Kw::MESHWHEELS: w.type = WheelDef::MESHWHEELS; break;
    case Kw::MESHWHEELS2: w.type = WheelDef::MESHWHEELS2; break;
    default: w.type = WheelDef::FLEXBODYWHEELS; break;
    }
    if (!need(min)) return;
    const bool legacy = w.type == WheelDef::WHEELS;
    w.rays = ival(legacy ? 2 : 3);
    if (w.rays < 1 || w.rays > kMaxRays) {
        warn_at(line_no_, "(%s) invalid ray count %d, wheel ignored", kw_name(block_), w.rays);
        return;
    }
    if (legacy) {
        w.radius = f(0);
        w.width = f(1);
    } else if (w.type == WheelDef::WHEELS2) {
        w.rim_radius = f(0); // rim radius first, unlike the other types
        w.radius = f(1);
        w.width = f(2);
    } else {
        w.radius = f(0);
        w.rim_radius = f(1);
        w.width = f(2);
    }
    int i = legacy ? 3 : 4;
    w.n1 = ref(i);
    w.n2 = ref(i + 1);
    w.rigidity = tok_[i + 2] == "9999" ? -1 : ref(i + 2);
    w.braking = ival(i + 3);
    if (w.braking < 0 || w.braking > 4) {
        warn_at(line_no_, "invalid braking value %d, using 0", w.braking);
        w.braking = 0;
    }
    w.propulsion = ival(i + 4);
    if (w.propulsion < 0 || w.propulsion > 2) {
        warn_at(line_no_, "invalid propulsion value %d, using 0", w.propulsion);
        w.propulsion = 0;
    }
    w.arm = ref(i + 5);
    w.mass = f(i + 6);
    i += 7;
    auto side = [&](int k) {
        char c = tok_[k][0];
        if (c == 'l' || c == 'r') return c;
        warn_at(line_no_, "invalid wheel side '%.*s', using 'l'", (int)tok_[k].size(), tok_[k].data());
        return 'l';
    };
    switch (w.type) {
    case WheelDef::WHEELS:
        w.spring = f(i);
        w.damp = f(i + 1);
        w.face_material = std::string(tok_[i + 2]);
        w.band_material = std::string(tok_[i + 3]);
        break;
    case WheelDef::WHEELS2:
        w.rim_spring = f(i);
        w.rim_damp = f(i + 1);
        w.spring = f(i + 2);
        w.damp = f(i + 3);
        w.face_material = std::string(tok_[i + 4]);
        w.band_material = std::string(tok_[i + 5]);
        break;
    case WheelDef::MESHWHEELS:
    case WheelDef::MESHWHEELS2:
        w.spring = f(i);
        w.damp = f(i + 1);
        w.side = side(i + 2);
        w.rim_mesh = std::string(tok_[i + 3]);
        w.tyre_material = std::string(tok_[i + 4]);
        break;
    case WheelDef::FLEXBODYWHEELS:
        w.spring = f(i);
        w.damp = f(i + 1);
        w.rim_spring = f(i + 2);
        w.rim_damp = f(i + 3);
        w.side = side(i + 4);
        if (ntok_ > i + 5) w.rim_mesh = std::string(tok_[i + 5]);
        if (ntok_ > i + 6) w.tyre_material = std::string(tok_[i + 6]);
        break;
    }
    w.nd = nd_;
    w.bd = bd_;
    w.line = line_no_;
    w.first_node = (int)doc_.nodes.size();
    const int wi = (int)doc_.wheels.size();
    const int count = w.node_count();
    for (int k = 0; k < count; k++) doc_.nodes.push_back({NodeSlot::WHEEL, wi, k});
    node_mod_.insert(node_mod_.end(), (size_t)count, cur_);
    doc_.wheels.push_back(std::move(w));
    wheel_mod_.push_back(cur_);
}

void Parser::parse_engine() {
    if (!need(6)) return;
    EngineDef e;
    e.present = true;
    e.shift_down_rpm = f(0);
    e.shift_up_rpm = f(1);
    e.torque = f(2);
    e.diff_ratio = f(3);
    e.rev_ratio = f(4);
    e.neutral_ratio = f(5);
    for (int i = 6; i < ntok_; i++) {
        float r = f(i);
        if (r < 0) break; // optional -1 terminator
        e.gears.push_back(r);
    }
    if (e.gears.empty()) {
        warn_at(line_no_, "engine has no forward gear, line skipped");
        return;
    }
    md().engine = std::move(e);
}

// Split on ',' only (empty items dropped): 1 item = predefined model, 2 items = (rpm, fraction) sample.
void Parser::parse_torquecurve() {
    sv items[3];
    int n = 0;
    for (size_t s = 0; s <= line_.size();) {
        size_t e = line_.find(',', s);
        if (e == sv::npos) e = line_.size();
        if (e > s) {
            if (n == 3) break;
            items[n++] = line_.substr(s, e - s);
        }
        s = e + 1;
    }
    TorqueCurveDef& tc = md().torquecurve;
    if (n == 1) {
        tc.model = std::string(trim_blanks(items[0]));
    } else if (n == 2) {
        tc.points.push_back(vec2(parse_real(trim_blanks(items[0])), parse_real(trim_blanks(items[1]))));
    } else {
        warn_at(line_no_, "(torquecurve) too many arguments, line skipped");
    }
}

// One ','-separated item of an axles line: `[w1|w2(ID ID)] [d([olsv]*)] [;comment | //comment]`.
bool Parser::parse_axle_item(sv s, AxleDef& a, sv ids[4], int& nids) {
    size_t i = 0, n = s.size();
    auto blanks = [&] {
        while (i < n && is_blank(s[i])) i++;
    };
    auto id = [&](sv& out) {
        size_t b = i;
        while (i < n && (isalnum((unsigned char)s[i]) || s[i] == '_' || s[i] == '-')) i++;
        out = s.substr(b, i - b);
        return i > b;
    };
    blanks();
    if (i + 2 < n && s[i] == 'w' && (s[i + 1] == '1' || s[i + 1] == '2') && s[i + 2] == '(') {
        int w = s[i + 1] - '1';
        i += 3;
        sv id1, id2;
        if (!id(id1)) return false;
        size_t b = i;
        blanks();
        if (i == b || !id(id2)) return false;
        if (i >= n || s[i] != ')') return false;
        i++;
        blanks();
        ids[w * 2] = id1;
        ids[w * 2 + 1] = id2;
        nids |= 1 << w;
    }
    if (i + 1 < n && s[i] == 'd' && s[i + 1] == '(') {
        i += 2;
        size_t b = i;
        while (i < n && (s[i] == 'o' || s[i] == 'l' || s[i] == 's' || s[i] == 'v')) i++;
        if (i >= n || s[i] != ')') return false;
        a.modes.append(s.data() + b, i - b);
        i++;
    }
    blanks();
    return i == n || s[i] == ';' || (s[i] == '/' && i + 1 < n && s[i + 1] == '/');
}

void Parser::parse_axle() {
    AxleDef a;
    sv ids[4];
    int have = 0;
    for (size_t s = 0; s <= line_.size();) {
        size_t e = line_.find(',', s);
        if (e == sv::npos) e = line_.size();
        sv item = line_.substr(s, e - s);
        if (!item.empty() && !parse_axle_item(item, a, ids, have)) {
            warn_at(line_no_, "(axles) invalid property '%.*s', line skipped", (int)item.size(), item.data());
            return;
        }
        s = e + 1;
    }
    if (have & 1) {
        a.w1a = ref(ids[0]);
        a.w1b = ref(ids[1]);
    }
    if (have & 2) {
        a.w2a = ref(ids[2]);
        a.w2b = ref(ids[3]);
    }
    md().axles.push_back(std::move(a));
}

void Parser::parse_flexbody() {
    if (!need(10)) return;
    FlexbodyDef fb;
    fb.ref = ref(0);
    fb.x = ref(1);
    fb.y = ref(2);
    fb.offset = vec3(f(3), f(4), f(5));
    fb.rot = vec3(f(6), f(7), f(8));
    fb.mesh = std::string(tok_[9]);
    fb.line = line_no_;
    md().flexbodies.push_back(std::move(fb));
}

void Parser::parse_prop() {
    if (!need(10)) return;
    PropDef p;
    p.ref = ref(0);
    p.x = ref(1);
    p.y = ref(2);
    p.offset = vec3(f(3), f(4), f(5));
    p.rot = vec3(f(6), f(7), f(8));
    p.mesh = std::string(tok_[9]);
    p.line = line_no_;
    // special props: case-sensitive, first match wins (RoR IdentifySpecialProp)
    const sv m = tok_[9];
    auto has = [&](const char* s) { return m.find(s) != sv::npos; };
    auto starts = [&](const char* s) { return m.substr(0, strlen(s)) == s; };
    if (has("leftmirror")) p.special = PropDef::MIRROR_LEFT;
    else if (has("rightmirror")) p.special = PropDef::MIRROR_RIGHT;
    else if (has("dashboard-rh")) p.special = PropDef::DASHBOARD_RH;
    else if (has("dashboard")) p.special = PropDef::DASHBOARD;
    else if (starts("spinprop")) p.special = PropDef::SPINPROP;
    else if (starts("pale")) p.special = PropDef::PALE;
    else if (starts("seat")) p.special = PropDef::SEAT;
    else if (starts("beacon")) p.special = PropDef::BEACON;
    else if (starts("redbeacon")) p.special = PropDef::REDBEACON;
    else if (starts("lightb")) p.special = PropDef::LIGHTBAR;

    if (p.special == PropDef::DASHBOARD || p.special == PropDef::DASHBOARD_RH) {
        p.wheel_mesh = ntok_ > 10 ? std::string(tok_[10]) : std::string("dirwheel.mesh");
        p.wheel_offset = vec3(p.special == PropDef::DASHBOARD ? -0.67f : 0.67f, -0.61f, 0.24f);
        if (ntok_ > 13) {
            p.wheel_offset = vec3(f(11), f(12), f(13));
            p.has_wheel_offset = true;
        }
        if (ntok_ > 14) p.wheel_angle = f(14);
    } else if (p.special == PropDef::BEACON) {
        p.beacon_material = "tracks/beaconflare";
        if (ntok_ >= 14) {
            p.beacon_material = std::string(tok_[10]);
            p.beacon_color = vec3(f(11), f(12), f(13));
        }
    } else if (p.special == PropDef::REDBEACON) {
        p.beacon_material = "tracks/redbeaconflare";
        p.beacon_color = vec3(1, 0, 0);
    }
    md().props.push_back(std::move(p));
}

void Parser::parse_managed_material() {
    if (!need(2)) return;
    ManagedMaterialDef m;
    m.name = std::string(tok_[0]);
    m.type = std::string(tok_[1]);
    m.double_sided = mm_double_sided_;
    const bool mesh = m.type == "mesh_standard" || m.type == "mesh_transparent";
    const bool flex = m.type == "flexmesh_standard" || m.type == "flexmesh_transparent";
    if (!mesh && !flex) {
        warn_at(line_no_, "invalid managed material type '%s', line skipped", m.type.c_str());
        return;
    }
    if (!need(3)) return;
    auto tex = [&](int i) { return tok_[i][0] == '-' ? std::string() : std::string(tok_[i]); };
    m.diffuse = std::string(tok_[2]);
    if (mesh) {
        if (ntok_ > 3) m.specular = tex(3);
    } else {
        if (ntok_ > 3) m.damaged_diffuse = tex(3);
        if (ntok_ > 4) m.specular = tex(4);
    }
    md().managed_materials.push_back(std::move(m));
}

void Parser::parse_guisetting() {
    if (!need(2)) return;
    Module& m = mods_[cur_];
    if (tok_[0] == "speedoMax") {
        float v = f(1);
        m.has_speedo = true;
        m.d.gui.speedo_max = (v > 10 && v < 32000) ? v : 140.0f;
        if (!(v > 10 && v < 32000)) warn_at(line_no_, "invalid speedoMax %g (allowed 10..32000), using 140", v);
    } else if (tok_[0] == "useMaxRPM") {
        m.d.gui.use_max_rpm = true;
    }
}

// ------------------------------------------------------------------ finish

void Parser::merge(const Module& m) {
    const Document& s = m.d;
    auto app = [](auto& dst, const auto& src) { dst.insert(dst.end(), src.begin(), src.end()); };
    {
        // (a module's frame sections follow the ones before: its frame elements' indices move with them)
        const int off = (int)doc_.frame_sections.size();
        for (auto b : s.beams) {
            if (b.frame >= 0) b.frame += off;
            doc_.beams.push_back(std::move(b));
        }
        app(doc_.frame_sections, s.frame_sections);
    }
    app(doc_.shocks, s.shocks);
    app(doc_.hydros, s.hydros);
    app(doc_.commands, s.commands);
    app(doc_.cameras, s.cameras);
    app(doc_.axles, s.axles);
    app(doc_.submeshes, s.submeshes);
    app(doc_.contacters, s.contacters);
    app(doc_.flexbodies, s.flexbodies);
    app(doc_.props, s.props);
    app(doc_.ropes, s.ropes);
    app(doc_.ties, s.ties);
    app(doc_.fixes, s.fixes);
    app(doc_.joints, s.joints);
    app(doc_.welds, s.welds);
    app(doc_.mounts, s.mounts);
    {
        // (a module's triangle elements' shells follow the ones before)
        const int off = (int)doc_.fem_shells.size();
        for (auto t : s.fem_tris) {
            t.shell += off;
            doc_.fem_tris.push_back(t);
        }
        app(doc_.fem_shells, s.fem_shells);
    }
    {
        // (a module's shell materials follow the ones before: its shells' indices move with them)
        const int off = (int)doc_.shell_materials.size();
        for (auto sh : s.shells) {
            if (sh.mat > 0) sh.mat += off;
            doc_.shells.push_back(sh);
        }
        app(doc_.shell_materials, s.shell_materials);
    }
    app(doc_.slidenodes, s.slidenodes);
    app(doc_.railgroups, s.railgroups);
    for (const ManagedMaterialDef& mm : s.managed_materials) {
        bool dup = false;
        for (const ManagedMaterialDef& o : doc_.managed_materials) dup = dup || o.name == mm.name;
        if (dup) warn_at(0, "duplicate managed material '%s' ignored (first definition wins)", mm.name.c_str());
        else doc_.managed_materials.push_back(mm);
    }
    if (m.has_globals) {
        doc_.dry_mass = s.dry_mass;
        doc_.cargo_mass = s.cargo_mass;
    }
    if (!s.cab_material.empty()) doc_.cab_material = s.cab_material;
    if (m.has_minimass) {
        doc_.minimass = s.minimass;
        doc_.minimass_skip_loaded = s.minimass_skip_loaded;
    }
    if (m.has_collision) doc_.collision_range = s.collision_range;
    if (m.has_speedo) doc_.gui.speedo_max = s.gui.speedo_max;
    if (s.gui.use_max_rpm) doc_.gui.use_max_rpm = true;
    if (s.engine.present) doc_.engine = s.engine;
    if (s.engoption.present) doc_.engoption = s.engoption;
    if (!s.torquecurve.model.empty() || !s.torquecurve.points.empty()) doc_.torquecurve = s.torquecurve;
    if (s.has_brakes) {
        doc_.brakes = s.brakes;
        doc_.has_brakes = true;
    }
}

void Parser::resolve_refs() {
    const int total = (int)doc_.nodes.size();
    auto fix = [&](int& r) {
        if (r < 0) return; // -1: no node (rigidity 9999, axle wheel not given)
        if (r >= kPendingRef) {
            const PendingName& p = pending_[(size_t)(r - kPendingRef)];
            auto it = names_.find(p.name);
            if (it != names_.end()) {
                r = it->second;
                return;
            }
            r = p.num;
            if (r >= total) {
                warn_at(p.line, "node '%s' is not a named node and index %d doesn't exist (%d nodes), using node 0",
                        p.name.c_str(), r, total);
                r = 0;
            }
            return;
        }
        if (r >= total) r = 0; // reported from far_refs_ below
    };
    auto active = [&](int mod) { return mod == 0 || (mod < (int)mods_.size() && mods_[mod].name == doc_.selected_config); };
    for (const RefNote& n : far_refs_)
        if (n.value >= total && active(n.mod))
            warn_at(n.line, "node %d doesn't exist (%d nodes), using node 0", n.value, total);

    for_each_ref(fix);
}

// Every node reference of the document (after merge).
template <class F> void Parser::for_each_ref(F&& fix) {
    Document& d = doc_;
    for (auto& b : d.beams) fix(b.n1), fix(b.n2);
    for (auto& s : d.shocks) fix(s.n1), fix(s.n2);
    for (auto& h : d.hydros) fix(h.n1), fix(h.n2);
    for (auto& c : d.commands) fix(c.n1), fix(c.n2);
    for (auto& c : d.cinecams)
        for (int& n : c.nodes) fix(n);
    for (auto& c : d.cameras) fix(c.center), fix(c.back), fix(c.left);
    for (auto& w : d.wheels) fix(w.n1), fix(w.n2), fix(w.rigidity), fix(w.arm);
    for (auto& a : d.axles) fix(a.w1a), fix(a.w1b), fix(a.w2a), fix(a.w2b);
    for (auto& sm : d.submeshes) {
        for (auto& t : sm.texcoords) fix(t.node);
        for (auto& c : sm.cabs) fix(c.n1), fix(c.n2), fix(c.n3);
    }
    for (int& n : d.contacters) fix(n);
    for (auto& fb : d.flexbodies) fix(fb.ref), fix(fb.x), fix(fb.y);
    for (auto& p : d.props) fix(p.ref), fix(p.x), fix(p.y);
    for (auto& r : d.ropes) fix(r.root), fix(r.end);
    for (auto& t : d.ties) fix(t.root);
    for (int& n : d.fixes) fix(n);
    for (auto& j : d.joints) {
        fix(j.parent), fix(j.child);
        if (j.ref_x >= 0) fix(j.ref_x);
        if (j.ref_y >= 0) fix(j.ref_y);
    }
    for (auto& s : d.shells) fix(s.n1), fix(s.n2), fix(s.n3);
    for (auto& w : d.welds) {
        fix(w.anchor), fix(w.node);
        if (w.anchor2 >= 0) fix(w.anchor2);
    }
    for (auto& m : d.mounts) fix(m.a), fix(m.b);
    for (auto& t : d.fem_tris) fix(t.n1), fix(t.n2), fix(t.n3);
    for (auto& s : d.slidenodes) {
        fix(s.node);
        for (int& n : s.rail) fix(n);
    }
    for (auto& g : d.railgroups)
        for (int& n : g.nodes) fix(n);
}

// RoR spawns only the root module and the selected section, but its sequential importer numbers the nodes
// generated by wheels of *every* section (that's why they were kept until the refs were resolved).
// Now drop the nodes and wheels that belong to other sections and renumber.
void Parser::drop_inactive_modules(int sel) {
    auto active = [&](int mod) { return mod == 0 || mod == sel; };
    const int total = (int)doc_.nodes.size();
    bool all = true;
    for (int m : node_mod_) all = all && active(m);
    for (int m : wheel_mod_) all = all && active(m);
    if (all || (int)node_mod_.size() != total || wheel_mod_.size() != doc_.wheels.size()) return;
    std::vector<int> wremap(doc_.wheels.size(), -1);
    std::vector<WheelDef> wheels;
    for (size_t i = 0; i < doc_.wheels.size(); i++)
        if (active(wheel_mod_[i])) {
            wremap[i] = (int)wheels.size();
            wheels.push_back(std::move(doc_.wheels[i]));
        }
    std::vector<int> remap(total, -1);
    std::vector<NodeSlot> nodes;
    std::vector<NodeDef> explicit_nodes;
    std::vector<CinecamDef> cinecams;
    for (int i = 0; i < total; i++) {
        NodeSlot s = doc_.nodes[i];
        if (s.kind == NodeSlot::WHEEL ? wremap[s.ref] < 0 : !active(node_mod_[i])) continue;
        if (s.kind == NodeSlot::WHEEL) s.ref = wremap[s.ref];
        else if (s.kind == NodeSlot::EXPLICIT) {
            explicit_nodes.push_back(std::move(doc_.nodes_explicit[(size_t)s.ref]));
            s.ref = (int)explicit_nodes.size() - 1;
        } else {
            cinecams.push_back(std::move(doc_.cinecams[(size_t)s.ref]));
            s.ref = (int)cinecams.size() - 1;
        }
        remap[i] = (int)nodes.size();
        nodes.push_back(s);
    }
    doc_.nodes = std::move(nodes);
    doc_.nodes_explicit = std::move(explicit_nodes);
    doc_.cinecams = std::move(cinecams);
    doc_.wheels = std::move(wheels);
    for (WheelDef& w : doc_.wheels) w.first_node = remap[w.first_node];
    int dangling = 0;
    for_each_ref([&](int& r) {
        if (r < 0 || r >= total) return;
        if (remap[r] < 0) dangling++;
        r = std::max(0, remap[r]);
    });
    for (FlexbodyDef& fb : doc_.flexbodies) {
        std::vector<int> kept;
        for (int n : fb.forset)
            if (n >= 0 && n < total && remap[n] >= 0) kept.push_back(remap[n]);
        fb.forset.swap(kept);
    }
    if (dangling) warn_at(0, "%d references to nodes of inactive sections, using node 0", dangling);
}

// Forset (first,last) pairs -> node list. Indices past the last node are dropped (as RoR does).
void Parser::expand_forsets() {
    const int total = (int)doc_.nodes.size();
    for (FlexbodyDef& fb : doc_.flexbodies) {
        std::vector<int> pairs;
        pairs.swap(fb.forset);
        int dropped = 0;
        for (size_t i = 0; i + 1 < pairs.size(); i += 2) {
            int a = pairs[i], b = pairs[i + 1];
            if (a > b) continue;
            if (b >= total) {
                dropped += (int)std::min<long long>((long long)b - std::max(a, total) + 1, INT32_MAX);
                b = total - 1;
            }
            for (int k = a; k <= b; k++) fb.forset.push_back(k);
        }
        if (dropped)
            warn_at(fb.line, "forset of flexbody '%s': %d node indices >= %d dropped", fb.mesh.c_str(), dropped, total);
    }
}

void Parser::finish(const std::string& config) {
    begin_block(Kw::NONE); // flush the staged submesh (RoR Finalize)

    for (size_t i = 1; i < mods_.size(); i++) doc_.configs.push_back(mods_[i].name);
    int sel = 0;
    for (size_t i = 1; i < mods_.size(); i++)
        if (!config.empty() && mods_[i].name == config) sel = (int)i;
    if (sel == 0 && !config.empty()) warn_at(0, "configuration '%s' not found, using the default", config.c_str());
    if (sel == 0 && mods_.size() > 1) sel = 1;
    if (sel > 0) doc_.selected_config = mods_[sel].name;

    // Root entries first, then the selected module's (RoR spawn order per element type).
    merge(mods_[0]);
    if (sel > 0) merge(mods_[sel]);
    if (!doc_.torquecurve.model.empty()) doc_.torquecurve.points.clear(); // a model name wins over samples
    doc_.has_axles_section = !doc_.axles.empty();

    // wheeldetachers: wheel_id indexes wheels in RoR spawn order (grouped by type).
    if (!wheel_detachers_.empty()) {
        std::vector<int> order;
        for (int t = WheelDef::WHEELS; t <= WheelDef::FLEXBODYWHEELS; t++)
            for (size_t i = 0; i < doc_.wheels.size(); i++)
                if (doc_.wheels[i].type == t && i < wheel_mod_.size() && (wheel_mod_[i] == 0 || wheel_mod_[i] == sel)) order.push_back((int)i);
        for (const WheelDetacher& wd : wheel_detachers_) {
            if (wd.mod != 0 && wd.mod != sel) continue;
            if (wd.wheel < 0 || wd.wheel >= (int)order.size())
                warn_at(wd.line, "wheeldetachers: invalid wheel id %d", wd.wheel);
            else doc_.wheels[(size_t)order[(size_t)wd.wheel]].detacher_group = wd.group;
        }
    }

    resolve_refs();
    expand_forsets();
    drop_inactive_modules(sel);
    if (suppressed_) doc_.warnings.push_back(format("%d more warnings suppressed", suppressed_));
}

} // namespace

bool parse_truck_file(const std::string& path, Document& doc, const std::string& config) {
    std::string text;
    if (!read_text_file(path, text)) {
        doc = Document();
        doc.path = path;
        doc.warnings.push_back("cannot read file");
        return false;
    }
    return parse_truck_text(text, path, doc, config);
}

bool parse_truck_text(const std::string& text, const std::string& path, Document& doc, const std::string& config) {
    doc = Document();
    doc.path = path;
    doc.dir = path_dir(path);
    Parser p(doc);
    p.run(text);
    p.finish(config);
    if (doc.nodes.empty()) doc.warnings.push_back("no nodes defined");
    return !doc.nodes.empty();
}

std::string read_truck_title(const std::string& path) {
    FILE* f = fopen(path.c_str(), "rb");
    if (!f) return {};
    char buf[kMaxLineLen + 1];
    std::string title;
    while (fgets(buf, (int)sizeof(buf), f)) { // reads at most kMaxLineLen chars, like RoR
        sv l(buf, strlen(buf));
        while (!l.empty() && (l.back() == '\n' || l.back() == '\r')) l.remove_suffix(1);
        while (!l.empty() && is_blank(l.front())) l.remove_prefix(1);
        if (l.empty() || l[0] == ';' || l[0] == '/') continue;
        title = std::string(trim_blanks(is_ascii(l) ? std::string(l) : utf8_fix(l)));
        break;
    }
    fclose(f);
    return title;
}

} // namespace bl::ror
