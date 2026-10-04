// Timed courses: the rally stage, the tape maze, the imported RBR stages.
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


// ---------------------------------------------------------------------------- stress scenes
// ------------------------------------------------------------------------------------------- rally stage
// Centre line of a stage: Catmull-Rom through control points, resampled every `step` metres.
struct StageRoute {
    std::vector<vec2> p;
    std::vector<float> s; // arc length at each sample
    float length() const { return s.back(); }
    int index_at(float d) const {
        int i = (int)(std::upper_bound(s.begin(), s.end(), d) - s.begin()) - 1;
        return std::clamp(i, 0, (int)p.size() - 2);
    }
    vec2 at(float d) const {
        int i = index_at(d);
        float t = clampf((d - s[i]) / std::max(1e-4f, s[i + 1] - s[i]), 0, 1);
        return p[i] + (p[i + 1] - p[i]) * t;
    }
    vec2 tangent(float d) const {
        int i = index_at(d);
        return normalize(p[i + 1] - p[i]);
    }
    // left of the driving direction (yaw +90 turns +z into +x, so left of +z is +x)
    vec2 left(float d) const {
        vec2 t = tangent(d);
        return vec2(t.y, -t.x);
    }
    float nearest_s(vec2 q, int from = 0, int to = -1) const {
        if (to < 0) to = (int)p.size() - 1;
        float best = 1e30f, bs = 0;
        for (int i = std::max(0, from); i < std::min(to, (int)p.size() - 1); i++) {
            vec2 ab = p[i + 1] - p[i];
            float t = clampf(dot(q - p[i], ab) / std::max(1e-6f, dot(ab, ab)), 0, 1);
            vec2 d = p[i] + ab * t - q;
            float dd = dot(d, d);
            if (dd < best) {
                best = dd;
                bs = s[i] + (s[i + 1] - s[i]) * t;
            }
        }
        return bs;
    }
    float dist(vec2 q) const {
        float best = 1e30f;
        for (size_t i = 0; i + 1 < p.size(); i++) {
            vec2 ab = p[i + 1] - p[i];
            float t = clampf(dot(q - p[i], ab) / std::max(1e-6f, dot(ab, ab)), 0, 1);
            vec2 d = p[i] + ab * t - q;
            best = std::min(best, dot(d, d));
        }
        return std::sqrt(best);
    }
    // signed curvature at sample i (+ = turning left), over +-w samples
    float curvature(int i, int w = 3) const {
        int a = std::max(0, i - w), b = std::min((int)p.size() - 1, i + w);
        if (b - a < 2) return 0;
        vec2 t0 = normalize(p[i] - p[a]), t1 = normalize(p[b] - p[i]);
        float turn = std::asin(clampf(t0.y * t1.x - t0.x * t1.y, -1, 1)); // cross(t0, t1) in (x, z): > 0 left
        return turn / std::max(1e-3f, 0.5f * (s[b] - s[a]));
    }
};

StageRoute make_stage_route(const std::vector<vec2>& ctrl, float step) {
    StageRoute r;
    std::vector<vec2> q = {ctrl.front()};
    q.insert(q.end(), ctrl.begin(), ctrl.end());
    q.push_back(ctrl.back());
    auto cr = [](vec2 p0, vec2 p1, vec2 p2, vec2 p3, float t) {
        float t2 = t * t, t3 = t2 * t;
        return (p1 * 2.0f + (p2 - p0) * t + (p0 * 2.0f - p1 * 5.0f + p2 * 4.0f - p3) * t2 + (p1 * 3.0f - p0 - p2 * 3.0f + p3) * t3) * 0.5f;
    };
    std::vector<vec2> dense;
    for (size_t i = 1; i + 2 < q.size(); i++) {
        int n = std::max(8, (int)(length(q[i + 1] - q[i]) / 0.5f));
        for (int k = 0; k < n; k++) dense.push_back(cr(q[i - 1], q[i], q[i + 1], q[i + 2], (float)k / n));
    }
    dense.push_back(ctrl.back());
    // resample at even spacing
    r.p.push_back(dense[0]);
    r.s.push_back(0);
    float acc = 0;
    for (size_t i = 1; i < dense.size(); i++) {
        acc += length(dense[i] - dense[i - 1]);
        if (acc >= step || i + 1 == dense.size()) {
            r.s.push_back(r.s.back() + length(dense[i] - r.p.back()));
            r.p.push_back(dense[i]);
            acc = 0;
        }
    }
    return r;
}

// Rounded crest across the road (jump): rises over `up` metres, falls over `down`, full height within the road.
void add_crest(Heightfield& hf, vec2 c, vec2 dir, float height, float up, float down, float half_width) {
    vec2 n(dir.y, -dir.x);
    const vec2 o = hf.origin();
    float reach = std::max(up, down) + half_width + 10;
    for (int z = 0; z < hf.nz(); z++)
        for (int x = 0; x < hf.nx(); x++) {
            vec2 p(o.x + x * hf.cell(), o.y + z * hf.cell());
            if (std::fabs(p.x - c.x) > reach || std::fabs(p.y - c.y) > reach) continue;
            float a = dot(p - c, dir), l = std::fabs(dot(p - c, n));
            float prof = 0;
            if (a > -up && a <= 0) prof = smoothstepf(0, 1, (a + up) / up);
            else if (a > 0 && a < down) prof = 1.0f - smoothstepf(0, 1, a / down);
            float lat = 1.0f - smoothstepf(0, 1, (l - half_width) / 10.0f);
            hf.h(x, z) += height * prof * clampf(lat, 0, 1);
        }
}

std::string stage_time(double t) {
    int m = (int)(t / 60.0);
    return format("%d:%05.2f", m, t - m * 60.0);
}

// Stage clock between two arc lengths of the route (banner + status line) and the Autopilot scene action
// (the AI drives the player's car along `line`).
void install_stage_timer(Game& g, std::shared_ptr<StageRoute> route, float s_start, float s_finish, std::vector<vec3> line, float ai_speed,
                         float ai_lat_acc, std::string wait_text) {
    struct Timer {
        int state = 0; // 0 before the start, 1 running, 2 finished
        double t0 = 0, result = 0, best = 0;
        int idx = 0;
    };
    auto timer = std::make_shared<Timer>();
    g.scene_actions.push_back({"Autopilot: AI drives the stage", [line, ai_speed, ai_lat_acc](Game& gg) {
                                   Vehicle* v = gg.player_vehicle();
                                   if (!v) return;
                                   if (v->ai && v->ai_state.autopilot) {
                                       v->ai = false;
                                       v->ai_state.autopilot = false;
                                       return;
                                   }
                                   v->ai = true;
                                   v->ai_state.autopilot = true;
                                   ai_set_race_route(*v, line, ai_speed, ai_lat_acc, 4.0f, false);
                               }});
    g.scene_update = [route, timer, s_start, s_finish, wait_text](Game& gg, float dt) {
        ai_update_all(gg, dt);
        Vehicle* v = gg.player_vehicle();
        if (!v) return;
        const StageRoute& R = *route;
        vec3 p3 = v->position();
        vec2 p(p3.x, p3.z);
        int i0 = std::max(0, timer->idx - 20), i1 = std::min((int)R.p.size() - 1, timer->idx + 60);
        float s = R.nearest_s(p, i0, i1);
        if (length(R.at(s) - p) > 20.0f) s = R.nearest_s(p); // reset / off the stage: global search
        timer->idx = R.index_at(s);
        double now = gg.world.time();
        std::string pilot = v->ai_state.autopilot && v->ai ? "  [autopilot]" : "";
        if (s < s_start - 5.0f && timer->state != 0) timer->state = 0;
        if (timer->state == 0) {
            gg.scene_banner.clear();
            gg.scene_status = wait_text + pilot;
            if (s >= s_start && s < s_start + 30) {
                timer->state = 1;
                timer->t0 = now;
            }
        }
        if (timer->state == 1) {
            double t = now - timer->t0;
            gg.scene_banner = format("%s   %.2f / %.2f km", stage_time(t).c_str(), std::max(0.0f, s - s_start) / 1000.0f, (s_finish - s_start) / 1000.0f);
            gg.scene_status = "Stage running" + pilot;
            if (s >= s_finish) {
                timer->state = 2;
                timer->result = t;
                if (timer->best <= 0 || t < timer->best) timer->best = t;
            }
        }
        if (timer->state == 2) {
            gg.scene_banner = "FINISH  " + stage_time(timer->result);
            gg.scene_status = format("Stage time %s, best %s. F5 restarts the stage.", stage_time(timer->result).c_str(),
                                     stage_time(timer->best).c_str()) + pilot;
        }
    };
}

void scene_rally(Game& g) {
    auto& A = SharedAssets::get();
    g.create_terrain(701, 701, 1.0f, vec2(-350, -350));
    auto& hf = g.world.statics.terrain;
    te::add_noise(hf, 11.0f, 130.0f, 4, 23);
    const std::vector<vec2> ctrl = {
        {-300, -255}, {-250, -255}, {-190, -255}, {-150, -250}, {-126, -232}, {-118, -195}, {-124, -150}, {-112, -110}, {-80, -86},
        {-35, -78},   {10, -86},    {52, -104},   {92, -100},   {122, -72},   {132, -30},   {118, 10},    {124, 50},    {152, 78},
        {190, 84},    {222, 100},   {240, 128},   {244, 152},   {236, 170},   {220, 176},   {204, 168},   {198, 150},   {190, 128},
        {168, 120},   {140, 130},   {112, 152},   {92, 184},    {60, 200},    {20, 196},    {-20, 180},   {-60, 184},   {-100, 204},
        {-140, 212},  {-190, 208},  {-240, 214},  {-290, 214}};
    auto route = std::make_shared<StageRoute>(make_stage_route(ctrl, 3.0f));
    const StageRoute& R = *route;
    const float half_w = 3.6f;
    // sections (arc length): fields at both ends, a forest in the middle (twisty part + hairpin)
    const float forest0 = R.nearest_s(ctrl[12]) - 25, forest1 = R.nearest_s(ctrl[29]) + 30;
    auto in_forest = [&](float s) { return s > forest0 && s < forest1; };
    const float s_start = R.nearest_s(ctrl[1]), s_finish = R.length() - 45.0f;
    // crests on the fast field straights
    for (int k : {9, 34}) {
        float s = R.nearest_s(ctrl[k]);
        add_crest(hf, R.at(s), R.tangent(s), 1.5f, 11.0f, 8.0f, half_w + 2);
    }
    for (float s : {0.0f, R.length()}) {
        vec2 c = R.at(s);
        te::flatten_circle(hf, c, 22, hf.height(c.x, c.y), 12, SURF_GRASS);
    }
    te::paint_road(hf, R.p, half_w, SURF_GRAVEL, 0.85f);
    // (the terrain is finished after the bales and haystacks: each gets a levelled patch, a round bale on a
    // cross slope would roll away)
    auto ground = [&](vec2 p) { return vec3(p.x, hf.height(p.x, p.y), p.y); };

    Rng rng(4242);
    std::vector<std::pair<vec2, float>> taken; // occupied spots (centre, radius): stakes, bales, trees
    auto free_at = [&](vec2 p, float r) {
        for (auto& t : taken)
            if (length(t.first - p) < t.second + r) return false;
        return true;
    };
    int n_tape = 0, n_bale = 0, n_tree = 0, n_stack = 0;

    // ---- start / finish gates: two posts and a chequered cross bar
    auto gate = [&](float s) {
        vec2 c = R.at(s), t = R.tangent(s), l = R.left(s);
        float h = hf.height(c.x, c.y);
        quat q = yaw_q(std::atan2(t.x, t.y) * kRad2Deg);
        for (int side : {-1, 1}) {
            vec2 pp = c + l * (side * (half_w + 1.6f));
            g.add_static_box(vec3(pp.x, h + 2.6f, pp.y), vec3(0.15f, 2.6f, 0.15f), q, SURF_METAL, A.metal);
            taken.push_back({pp, 1.0f});
        }
        g.add_static_box(vec3(c.x, h + 5.0f, c.y), vec3(half_w + 1.75f, 0.4f, 0.08f), q, SURF_METAL, A.banner);
    };
    gate(s_start);
    gate(s_finish);

    // ---- corners: tape on the outside + round bales at the apex (fields), bales in front of the trees (forest)
    std::vector<int> apexes;
    for (int i = 3; i + 3 < (int)R.p.size(); i++) {
        float k = std::fabs(R.curvature(i));
        if (k < 1.0f / 70.0f) continue;
        bool peak = true;
        for (int j = std::max(0, i - 6); j <= std::min((int)R.p.size() - 1, i + 6); j++)
            if (std::fabs(R.curvature(j)) > k) peak = false;
        if (peak && (apexes.empty() || R.s[i] - R.s[apexes.back()] > 30.0f)) apexes.push_back(i);
    }
    auto tape_line = [&](float s0, float s1, float offset, float spacing) {
        TapeLineDesc td;
        for (float s = s0; s <= s1 + 0.01f; s += spacing) {
            vec2 p = R.at(s) + R.left(s) * offset;
            if (R.dist(p) < half_w + 1.2f || !free_at(p, 0.6f)) {
                if (td.posts.size() >= 3) {
                    g.add_object(build_tape_line(g.world, td, format("tape%d", n_tape++)));
                }
                td.posts.clear();
                continue;
            }
            td.posts.push_back(ground(p) - vec3(0, 0.05f, 0));
        }
        if (td.posts.size() >= 3) g.add_object(build_tape_line(g.world, td, format("tape%d", n_tape++)));
        for (float s = s0; s <= s1; s += spacing) taken.push_back({R.at(s) + R.left(s) * offset, 0.5f});
    };
    // round bale lying on its side; `h`: ground height of the levelled patch (shared by a row of bales)
    auto round_bale = [&](vec2 p, float yaw, float h) {
        if (!free_at(p, 0.9f) || R.dist(p) < half_w + 0.6f) return;
        te::flatten_circle(hf, p, 1.0f, h, 0.9f);
        g.add_object(build_round_bale(g.world, vec3(p.x, h + 0.78f, p.y), yaw, format("bale%d", n_bale++)));
        taken.push_back({p, 0.9f});
    };
    for (int ai : apexes) {
        float s = R.s[ai], k = R.curvature(ai);
        if (s < s_start + 20 || s > s_finish - 10) continue;
        float out = k > 0 ? -1.0f : 1.0f; // outside of the corner
        float radius = 1.0f / std::fabs(k);
        vec2 t = R.tangent(s);
        float yaw = std::atan2(t.x, t.y) * kRad2Deg + 90.0f; // bale axis along the road
        // round bales at the apex on the outside
        int nb = radius < 25 ? 4 : 3;
        float row_h = hf.height(R.at(s).x + R.left(s).x * out * (half_w + 1.6f), R.at(s).y + R.left(s).y * out * (half_w + 1.6f));
        for (int b = 0; b < nb; b++) {
            float sb = s + (b - (nb - 1) * 0.5f) * 1.7f;
            round_bale(R.at(sb) + R.left(sb) * (out * (half_w + 1.6f)), std::atan2(R.tangent(sb).x, R.tangent(sb).y) * kRad2Deg + 90.0f, row_h);
        }
        (void)yaw;
        if (!in_forest(s) || radius < 25) tape_line(s - 26, s + 26, out * (half_w + 4.2f), 4.0f);
        if (radius < 25) {
            // hairpin: bales on the inside too, and a second tape line further out for the spectators
            vec2 pin = R.at(s) - R.left(s) * (out * (half_w + 1.5f));
            round_bale(pin, std::atan2(t.x, t.y) * kRad2Deg + 90.0f, hf.height(pin.x, pin.y));
            tape_line(s - 20, s + 20, out * (half_w + 9.0f), 4.0f);
        }
    }
    // spectator tape along the start straight (both sides)
    tape_line(s_start + 8, s_start + 70, half_w + 4.0f, 4.0f);
    tape_line(s_start + 8, s_start + 70, -(half_w + 4.0f), 4.0f);

    // ---- chicane of small square bales on the return straight (two stacked walls, alternating sides)
    auto square_bale = [&](vec3 c, float yaw) {
        SoftBoxDesc d;
        d.center = c;
        d.size = vec3(0.9f, 0.4f, 0.46f); // length (x), height, width
        d.rot = yaw_q(yaw);
        d.nx = 3;
        d.ny = 2;
        d.nz = 2;
        d.mass = 22;
        d.mat = A.straw;
        d.beams = {3e5f, 120, 1500, 6e4f, 0.35f};
        g.add_object(build_soft_box(g.world, d, format("sqbale%d", n_stack++)));
    };
    {
        float sc = R.nearest_s(ctrl[37]);
        for (int w = 0; w < 2; w++) {
            float s = sc + w * 24.0f;
            vec2 c = R.at(s), l = R.left(s), t = R.tangent(s);
            float side = w == 0 ? 1.0f : -1.0f;
            float yaw = std::atan2(t.x, t.y) * kRad2Deg; // local z along the road -> bale length (x) across it
            // gaps of ~0.1 m: neighbours must not start inside each other's collision radius
            for (int layer = 0; layer < 2; layer++)
                for (int row = 0; row < 2; row++)
                    for (int b = 0; b < 4 - layer; b++) {
                        float lat = side * (0.3f + 0.5f + b * 1.0f + layer * 0.5f);
                        vec2 p = c + l * lat + t * ((row - 0.5f) * 0.58f);
                        square_bale(ground(p) + vec3(0, 0.23f + layer * 0.48f, 0), yaw);
                    }
            taken.push_back({c + l * (side * 2.0f), 2.5f});
        }
    }
    // ---- haystacks in the fields (a couple close to fast corners), a round bale pyramid at the start
    for (int i = 0, tries = 0; i < 9 && tries < 400; tries++) {
        float s = rng.range(s_start + 30, s_finish - 30);
        if (in_forest(s)) continue;
        float lat = (rng.uniform() < 0.5f ? -1.0f : 1.0f) * rng.range(i < 3 ? 10.0f : 18.0f, i < 3 ? 13.0f : 45.0f);
        vec2 p = R.at(s) + R.left(s) * lat;
        if (!hf.inside(p.x, p.y) || R.dist(p) < 9.0f || !free_at(p, 3.5f)) continue;
        float rad = rng.range(1.9f, 2.4f), hgt = rng.range(3.4f, 4.2f);
        te::flatten_circle(hf, p, rad + 0.4f, hf.height(p.x, p.y), 2.0f);
        g.add_object(build_haystack(g.world, ground(p) - vec3(0, 0.05f, 0), hgt, rad, format("haystack%d", i)));
        taken.push_back({p, 3.0f});
        i++;
    }
    {
        vec2 c = R.at(4.0f), l = R.left(4.0f), t = R.tangent(4.0f);
        float yaw = std::atan2(t.x, t.y) * kRad2Deg + 90.0f;
        vec2 base = c + l * 14.0f;
        // 3-2-1 pyramid; centres 1.62 m apart (0.12 m gaps), rows at the touching height + 6 cm
        const int rows[3] = {3, 2, 1};
        const float pitch = 1.62f, rise = std::sqrt(pitch * pitch - 0.25f * pitch * pitch) + 0.06f;
        float h0 = hf.height(base.x, base.y);
        te::flatten_circle(hf, base, 3.6f, h0, 2.0f);
        for (int r = 0; r < 3; r++)
            for (int b = 0; b < rows[r]; b++) {
                vec2 p = base + t * ((b - (rows[r] - 1) * 0.5f) * pitch);
                g.add_object(build_round_bale(g.world, vec3(p.x, std::max(h0, hf.height(p.x, p.y)) + 0.78f + r * rise, p.y), yaw,
                                              format("bale%d", n_bale++)));
            }
        taken.push_back({base, 3.0f});
    }

    // ---- detailed road surface: ruts, camber, bumps, potholes, washboard before the tighter corners
    {
        auto road = std::make_shared<RoadSurface>();
        StageRoute dense = make_stage_route(ctrl, 1.0f);
        std::vector<vec2> wash;
        for (int ai : apexes)
            if (1.0f / std::max(1e-4f, std::fabs(R.curvature(ai))) < 45.0f) wash.push_back({R.s[ai] - 55.0f, R.s[ai] - 12.0f});
        road->build(dense.p, half_w, 1.2f, SURF_GRAVEL, 77, wash);
        g.world.statics.road = road;
        // the terrain under the frayed road edge is grass (the road texture's alpha lets it through)
        const vec2 o = hf.origin();
        for (int z = 0; z < hf.nz(); z++)
            for (int x = 0; x < hf.nx(); x++) {
                float s, lat;
                if (hf.surf(x, z) == SURF_GRAVEL && road->locate(o.x + x * hf.cell(), o.y + z * hf.cell(), s, lat) && std::fabs(lat) > half_w - 0.9f)
                    hf.surf(x, z) = SURF_GRASS;
            }
    }
    te::auto_surfaces(hf);
    g.finish_terrain();

    // ---- trees: the forest section (dense at the verge), a birch line along the first field, a few field trees
    auto tree = [&](vec2 p, float verge) {
        if (!hf.inside(p.x, p.y) || R.dist(p) < verge || !free_at(p, 1.6f)) return false;
        vec3 n;
        float h;
        hf.sample(p.x, p.y, h, n);
        if (n.y < 0.8f) return false;
        TreeDesc d;
        d.base = vec3(p.x, h - 0.1f, p.y);
        d.seed = 7000 + n_tree;
        float k = rng.uniform();
        if (k < 0.18f) {
            d.kind = TreeKind::Bush;
            d.height = rng.range(1.2f, 2.2f);
            d.trunk_radius = 0.035f;
            d.strength = 0.6f;
        } else if (k < 0.62f) {
            d.kind = TreeKind::Pine;
            d.height = rng.range(9, 15);
            d.trunk_radius = rng.range(0.14f, 0.22f);
        } else if (k < 0.82f) {
            d.kind = TreeKind::Deciduous;
            d.height = rng.range(7, 11);
            d.trunk_radius = rng.range(0.14f, 0.2f);
        } else {
            d.kind = TreeKind::Birch;
            d.height = rng.range(7, 10);
            d.trunk_radius = rng.range(0.08f, 0.12f);
        }
        g.add_object(build_tree(g.world, d, format("tree%d", n_tree++)));
        taken.push_back({p, 1.2f});
        return true;
    };
    for (float s = forest0; s < forest1; s += 5.5f)
        for (int side : {-1, 1})
            if (rng.uniform() < 0.8f) tree(R.at(s) + R.left(s) * (side * rng.range(half_w + 1.9f, half_w + 5.5f)), half_w + 1.8f);
    for (int i = 0, tries = 0; i < 170 && tries < 3000; tries++) {
        float s = rng.range(forest0 - 25, forest1 + 25);
        vec2 p = R.at(s) + R.left(s) * ((rng.uniform() < 0.5f ? -1.0f : 1.0f) * rng.range(half_w + 6.0f, 42.0f));
        if (tree(p, half_w + 5.0f)) i++;
    }
    for (float s = s_start + 20; s < R.nearest_s(ctrl[5]); s += 14.0f) tree(R.at(s) + R.left(s) * 12.0f, 8.0f);
    for (int i = 0, tries = 0; i < 16 && tries < 400; tries++) {
        float s = rng.range(0, R.length());
        if (in_forest(s)) continue;
        vec2 p = R.at(s) + R.left(s) * ((rng.uniform() < 0.5f ? -1.0f : 1.0f) * rng.range(9.0f, 50.0f));
        if (tree(p, 8.0f)) i++;
    }

    // ---- decorative grass: dense on the verges (hides the seam), thinning out into the fields, a few tufts on the
    // crown of the road; vehicles flatten it
    {
        g.grass = std::make_unique<GrassField>();
        Rng gr(9090);
        const RoadSurface& road = *g.world.statics.road;
        uint32_t seed = 1;
        auto put = [&](vec2 p, float scale, float tint) {
            if (!hf.inside(p.x, p.y)) return;
            for (auto& t : taken)
                if (t.second >= 1.0f && length(t.first - p) < t.second * 0.8f) return; // not inside bales / stacks
            g.grass->add(vec3(p.x, g.ground_height(p.x, p.y) - 0.02f, p.y), scale, tint, seed++);
        };
        for (float s = 0; s < road.length(); s += 0.3f) {
            vec2 c = road.point(s), l = road.left(s);
            for (int side : {-1, 1}) {
                // verge band: from the frayed road edge 2.5 m out; short near the gravel, patchy, taller further out
                float patch = value_noise2(s * 0.08f, side * 3.0f, 21) * 0.5f + 0.5f;
                for (int k = 0; k < 7; k++) {
                    float lat = half_w + 0.2f + k * 0.36f + gr.range(-0.15f, 0.15f);
                    if (gr.uniform() < 0.3f + 0.4f * (1.0f - patch)) continue;
                    float sc = (k == 0 ? gr.range(0.45f, 0.8f) : gr.range(0.6f, 1.25f)) * (0.8f + 0.4f * patch);
                    put(c + l * (side * lat), sc, gr.uniform() * 0.5f);
                }
                // outer band: sparse, taller, drier
                if (gr.uniform() < 0.6f) put(c + l * (side * gr.range(half_w + 2.6f, half_w + 9.0f)), gr.range(0.9f, 1.6f), gr.range(0.2f, 0.8f));
            }
            // grass on the crown between the ruts, here and there
            if (gr.uniform() < 0.12f && value_noise2(s * 0.05f, 0.0f, 5) > 0.2f) put(c + l * gr.range(-0.25f, 0.25f), gr.range(0.4f, 0.7f), 0.3f);
        }
        // patches in the fields around the stage
        for (int i = 0; i < 260; i++) {
            float s = gr.range(0, R.length());
            vec2 base = R.at(s) + R.left(s) * ((gr.uniform() < 0.5f ? -1.0f : 1.0f) * gr.range(10.0f, 30.0f));
            int n = (int)gr.range(20, 60);
            for (int k = 0; k < n; k++) put(base + vec2(gr.range(-3, 3), gr.range(-3, 3)), gr.range(0.8f, 1.5f), gr.uniform());
        }
        g.grass->finalize();
        log_info("grass: %zu tufts", g.grass->count());
    }

    g.world.settings.wind = vec3(1.5f, 0, 1.0f); // a breeze: the tapes flutter, the trees sway a little
    g.world.settings.wind_gusts = 0.5f;
    g.world.settings.wind_radius = 80.0f; // trees and tapes sway around the camera, the rest of the stage sleeps
    vec2 sp = R.at(3.0f), st = R.tangent(3.0f);
    g.set_spawn(ground(sp), std::atan2(st.x, st.y) * kRad2Deg);
    g.scene_hint = format("Rally stage, %.1f km of gravel: fields, a forest with a hairpin, two crests and a bale chicane. "
                          "Trees, tape stakes, hay bales and haystacks are all soft bodies. Timer starts at the START gate.",
                          (s_finish - s_start) / 1000.0f);
    log_info("rally: %.0f m, %d corners, %d trees, %d tape lines, %d round bales, %d square bales", R.length(), (int)apexes.size(), n_tree,
             n_tape, n_bale, n_stack);

    // ---- stage timer + autopilot
    std::vector<vec3> line;
    for (vec2 p : R.p) line.push_back(ground(p));
    install_stage_timer(g, route, s_start, s_finish, line, 26.0f, 4.5f, "Drive through the START gate to start the clock.");
}

// ------------------------------------------------------------------------------------------- tape maze
// A gymkhana course marked only with tape: five lanes joined by hairpins, then a chicane to the finish. Every tape
// line is a soft body (stakes on orientation joints, tearing tape), split into short sections.
void scene_tape_maze(Game& g) {
    auto& A = SharedAssets::get();
    g.create_terrain(181, 161, 1.0f, vec2(-90, -80));
    auto& hf = g.world.statics.terrain;
    te::add_noise(hf, 0.6f, 35.0f, 3, 515);
    te::flatten_rect(hf, vec2(0, -5), vec2(75, 70), 0, hf.height(0, -5), 10.0f, SURF_GRASS);
    // centre line: lanes along x, 14 m apart, joined by hairpins (radius 7 m)
    const int lanes = 5;
    const float x0 = -38, x1 = 38, z0 = -56, pitch = 14.0f, half_w = 3.3f;
    std::vector<vec2> ctrl = {{x0 - 18, z0}, {x0 - 8, z0}};
    for (int l = 0; l < lanes; l++) {
        float z = z0 + l * pitch;
        bool east = (l % 2) == 0;
        float xa = east ? x0 : x1, xb = east ? x1 : x0, dir = east ? 1.0f : -1.0f;
        ctrl.push_back({xa, z});
        ctrl.push_back({(xa + xb) * 0.5f, z});
        ctrl.push_back({xb, z});
        if (l + 1 < lanes) { // hairpin
            ctrl.push_back({xb + dir * 5.0f, z + 1.8f});
            ctrl.push_back({xb + dir * 7.0f, z + pitch * 0.5f});
            ctrl.push_back({xb + dir * 5.0f, z + pitch - 1.8f});
        }
    }
    // (lane 4 ends in the east) chicane northwards, then the finish straight to the west
    const float zc = z0 + (lanes - 1) * pitch;
    for (vec2 q : std::initializer_list<vec2>{{x1 + 10, zc + 6}, {x1 + 6, zc + 16}, {x1 - 6, zc + 20}, {x1 - 16, zc + 16}, {x1 - 26, zc + 22},
                                               {x1 - 36, zc + 17}, {x1 - 46, zc + 22}, {x1 - 60, zc + 20}, {x1 - 80, zc + 20}, {x1 - 92, zc + 20}})
        ctrl.push_back(q);
    auto route = std::make_shared<StageRoute>(make_stage_route(ctrl, 1.0f));
    const StageRoute& R = *route;
    te::paint_road(hf, R.p, half_w + 0.3f, SURF_DIRT, 0.7f);
    g.finish_terrain();
    auto ground = [&](vec2 p) { return vec3(p.x, hf.height(p.x, p.y), p.y); };
    const float s_start = R.nearest_s(vec2(x0 - 8, z0)) + 2.0f, s_finish = R.length() - 14.0f;

    // tape on both sides, a stake every 3 m, sections of 8 stakes (short bodies: a knocked section sleeps again soon)
    int n_tape = 0, n_stakes = 0;
    for (int side : {-1, 1}) {
        TapeLineDesc td;
        auto flush = [&]() {
            if (td.posts.size() >= 2) g.add_object(build_tape_line(g.world, td, format("maze_tape%d", n_tape++)));
            n_stakes += (int)td.posts.size();
            td.posts.clear();
        };
        for (float d = s_start - 6.0f; d <= s_finish + 6.0f; d += 3.0f) {
            vec2 p = R.at(d) + R.left(d) * (side * half_w);
            // keep clear of the other lanes (the inside of the hairpins folds back onto itself)
            bool clear = R.dist(p) > half_w - 0.4f;
            if (!clear) {
                flush();
                continue;
            }
            td.posts.push_back(ground(p) - vec3(0, 0.05f, 0));
            if (td.posts.size() >= 8) {
                vec3 last = td.posts.back();
                flush();
                td.posts.push_back(last + vec3(0.25f * R.tangent(d).x, 0, 0.25f * R.tangent(d).y)); // next section starts at the same stake
            }
        }
        flush();
    }
    // start / finish gates
    auto gate = [&](float d, MaterialPtr bar) {
        vec2 c = R.at(d), t = R.tangent(d), l = R.left(d);
        float h = hf.height(c.x, c.y);
        quat q = yaw_q(std::atan2(t.x, t.y) * kRad2Deg);
        for (int side : {-1, 1}) {
            vec2 pp = c + l * (side * (half_w + 1.2f));
            g.add_static_box(vec3(pp.x, h + 1.6f, pp.y), vec3(0.12f, 1.6f, 0.12f), q, SURF_METAL, A.metal);
        }
        g.add_static_box(vec3(c.x, h + 3.3f, c.y), vec3(half_w + 1.3f, 0.3f, 0.06f), q, SURF_METAL, bar);
    };
    gate(s_start, A.banner);
    gate(s_finish, A.banner);
    vec2 sp = R.at(1.0f), st = R.tangent(1.0f);
    g.set_spawn(ground(sp), std::atan2(st.x, st.y) * kRad2Deg);
    std::vector<vec3> line;
    for (vec2 p : R.p) line.push_back(ground(p));
    install_stage_timer(g, route, s_start, s_finish, line, 13.0f, 4.0f, "Drive through the START gate to start the clock.");
    g.world.settings.wind = vec3(1.2f, 0, 0.8f);
    g.world.settings.wind_gusts = 0.5f;
    g.world.settings.wind_radius = 60.0f;
    g.scene_hint = format("Tape maze: %.0f m between tapes only, 5 lanes, 4 hairpins and a chicane (%d stakes in %d tape sections). "
                          "The stakes bend and snap, the tape tears; the clock runs from START to FINISH.",
                          s_finish - s_start, n_stakes, n_tape);
    log_info("tape maze: %.0f m, %d stakes, %d tape sections", s_finish - s_start, n_stakes, n_tape);
}

// ------------------------------------------------------------------------------------------- imported RBR stage
// A Richard Burns Rally community stage converted by tools/fetch_rbr_stage.py: the original meshes and textures,
// the collision mesh as a 25 cm heightfield with the stage's surfaces, signs / banners / boards as light bodies with
// their own meshes, the round bales as soft bales, and the RBR driveline for the clock and the autopilot.
void scene_rbr_stage(Game& g, const std::string& id) {
    const std::string dir = asset_path("stages/" + id);
    auto bundle = std::make_shared<StageBundle>();
    std::string err;
    if (!bundle->load(dir + "/stage.bin", err)) {
        log_warn("RBR stage: %s", err.c_str());
        flat_arena(g, 60, SURF_ASPHALT, false);
        g.set_spawn(vec3(0, 0, -30), 0);
        g.scene_hint = "This RBR stage is not installed. Run  python3 tools/fetch_rbr_stage.py --stage " + id +
                       "  (downloads the stage from the author's Google Drive and converts it locally), then reload this scene.";
        g.scene_status = err;
        return;
    }
    const StageBundle& B = *bundle;
    auto& hf = g.world.statics.terrain;
    hf.create_sparse(B.nx, B.nz, B.cell, B.origin);
    g.world.statics.has_terrain = true;
    const size_t tn = (size_t)B.tile * B.tile;
    for (size_t t = 0; t < B.tiles.size(); t++) {
        std::copy_n(&B.tile_height[t * tn], tn, hf.tile_heights(B.tiles[t].first, B.tiles[t].second));
        uint8_t* ts = hf.tile_surfaces(B.tiles[t].first, B.tiles[t].second);
        for (size_t i = 0; i < tn; i++) ts[i] = B.tile_surface[t * tn + i] < SURF_COUNT ? B.tile_surface[t * tn + i] : (uint8_t)SURF_DIRT;
    }
    hf.update_bounds();
    // the stage's static shape collision: solid trunks, stumps and walls; bendable trees and bushes yield
    for (const auto& st : B.statics) {
        phys::StaticBox box;
        box.center = st.centre;
        box.rot = st.rot;
        box.half = st.half;
        box.surface = st.deck ? SURF_ASPHALT : SURF_WOOD;
        box.max_force = st.yielding ? 2500.0f : 0.0f; // N per node: a creeping car stops, a fast one pushes through
        box.update_aabb();
        g.world.statics.boxes.push_back(box);
    }
    g.scenery = std::make_unique<StageScenery>();
    g.scenery->build(B, dir);

    // props: the stage's own meshes on light box bodies (knocked over by the cars)
    int n_prop = 0;
    float s_clock = -1, s_finish_sign = -1, s_finish_board = -1;
    StageRoute R0;
    for (const vec3& p : B.line) {
        R0.s.push_back(R0.p.empty() ? 0.0f : R0.s.back() + length(vec2(p.x, p.z) - R0.p.back()));
        R0.p.push_back(vec2(p.x, p.z));
    }
    for (const auto& pr : B.props) {
        const auto& t = B.templates[(size_t)pr.tmpl];
        MeshPropDesc d;
        d.xform = mat4::from_mat3(pr.rot, pr.pos);
        d.hull_min = t.hull_min;
        d.hull_max = t.hull_max;
        d.mass = t.mass;
        for (auto& part : g.scenery->templates[(size_t)pr.tmpl]) d.parts.push_back({part.mesh.get(), part.mat.get()});
        DynamicObject* o = g.add_object(build_mesh_prop(g.world, d, format("%s #%d", t.name.c_str(), n_prop++)));
        // like in RBR the props stay put until something hits them (narrow bases on uneven ground would tip over)
        o->body->sleeping = true;
        std::string lname = to_lower(t.name);
        float s = R0.nearest_s(vec2(pr.pos.x, pr.pos.z));
        if (lname.find("start_clock") != std::string::npos) s_clock = s;
        if (lname.find("finish r") != std::string::npos) s_finish_sign = std::max(s_finish_sign, s); // red FINISH sign
        if (lname.find("finish") != std::string::npos && (lname.find("left") != std::string::npos || lname.find("right") != std::string::npos))
            s_finish_board = std::max(s_finish_board, s);
    }
    int n_bale = 0;
    for (const auto& b : B.bales) // (asleep like the props: some stand on slopes and embankment edges)
        g.add_object(build_standing_bale(g.world, b.pos - vec3(0, 0.02f, 0), b.yaw, b.radius, b.height, format("bale%d", n_bale++), 260.0f))
            ->body->sleeping = true;

    // clock: the stage's own start / finish pacenotes; without them the start clock and the red FINISH sign
    // (the car stands on the start line: the clock starts once it has rolled 2 m)
    auto route = std::make_shared<StageRoute>(std::move(R0));
    float s_start = B.clock_start >= 0 ? B.clock_start : (s_clock >= 0 ? s_clock : 2.0f);
    s_start = std::clamp(s_start + 2.0f, 2.0f, route->length() * 0.5f);
    float s_finish = route->length() - 5.0f;
    if (B.clock_finish > s_start + 50) s_finish = std::min(B.clock_finish, route->length() - 1.0f);
    else if (s_finish_sign > s_start + 50) s_finish = s_finish_sign;
    else if (s_finish_board > s_start + 50) s_finish = s_finish_board;
    vec3 sp = B.start;
    sp.y = g.ground_height(sp.x, sp.z);
    g.set_spawn(sp, B.heading);
    // the autopilot stops a little after the finish (at the STOP control; the collision mesh ends soon after)
    std::vector<vec3> line;
    for (size_t i = 0; i < B.line.size() && route->s[i] < s_finish + 20.0f; i++) line.push_back(vec3(B.line[i].x, g.ground_height(B.line[i].x, B.line[i].z), B.line[i].z));
    // autopilot limits by the surface under the driveline: tarmac corners faster than gravel
    int tarmac = 0;
    for (const vec3& p : line) tarmac += hf.surface_at(p.x, p.z) == SURF_ASPHALT ? 1 : 0;
    const bool on_tarmac = tarmac * 2 > (int)line.size();
    install_stage_timer(g, route, s_start, s_finish, line, on_tarmac ? 26.0f : 24.0f, on_tarmac ? 7.0f : 4.5f,
                        "Cross the start line to start the clock.");

    // morning light (the M variant of the stage)
    g.light.sun_dir = normalize(vec3(0.75f, 0.55f, -0.45f));
    g.light.sun_color = vec3(2.6f, 2.35f, 2.05f);
    g.light.fog_color = vec3(0.7f, 0.76f, 0.84f);
    g.light.fog_density = 0.0008f;
    g.world.settings.wind = vec3(1.0f, 0, 0.6f);
    g.world.settings.wind_radius = 60.0f;
    g.scene_hint = format("%s: a Richard Burns Rally community stage by RALLY Guru (rallyguru-tracks.blogspot.com), %.2f km. "
                          "Original meshes and textures; %d signs, banners and boards are knockable props%s. "
                          "Where the original has no collision (buildings, far scenery) invisible walls close the stage. "
                          "Personal non-commercial use (see assets/stages/%s/SOURCE.txt).",
                          B.title.c_str(), (s_finish - s_start) / 1000.0f, n_prop, n_bale ? ", the hay bales are soft bodies" : "", id.c_str());
    log_info("rbr stage: %zu triangles, %d props, %d bales, route %.0f m (clock %.0f..%.0f m)", g.scenery->triangles, n_prop, n_bale,
             route->length(), s_start, s_finish);
}

} // namespace bl
