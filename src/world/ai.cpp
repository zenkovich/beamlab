#include "world/ai.h"
#include "game/game.h"
#include "vehicle/vehicle.h"

#include <cmath>

namespace bl {

void ai_set_route(Vehicle& v, const std::vector<vec3>& route, float speed) {
    v.ai_state.route = route;
    v.ai_state.wp = 0;
    v.ai_state.target_speed = speed;
}

void ai_set_race_route(Vehicle& v, const std::vector<vec3>& route, float max_speed, float lat_acc, float brake_acc, bool loop) {
    AIState& s = v.ai_state;
    s.route = route;
    s.route_loop = loop;
    s.wp = 0;
    s.target_speed = max_speed;
    const int n = (int)route.size();
    s.route_speed.assign(n, max_speed);
    s.route_radius.assign(n, 1e4f);
    auto xz = [&](int i) { return vec2(route[(size_t)std::clamp(i, 0, n - 1)].x, route[(size_t)std::clamp(i, 0, n - 1)].z); };
    for (int i = 0; i < n; i++) {
        // circumradius of three points ~4 m apart on each side (dense lines: a 1 m spacing is too noisy)
        int k = 1;
        while (k < 16 && i - k > 0 && i + k < n - 1 && length(xz(i + k) - xz(i)) < 4.0f) k++;
        vec2 a = xz(i - k), b = xz(i), c = xz(i + k);
        float ab = length(b - a), bc = length(c - b), ca = length(a - c);
        float area2 = std::fabs((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x));
        if (area2 > 1e-4f) {
            s.route_radius[i] = ab * bc * ca / (2.0f * area2);
            s.route_speed[i] = std::min(max_speed, std::sqrt(lat_acc * s.route_radius[i]));
        }
    }
    // crests: keep the wheels on the ground, v^2 < 0.9 g R over convex bends of the height profile (~6 m chords)
    for (int i = 0; i < n; i++) {
        int a = i, c = i;
        while (a > 0 && length(xz(i) - xz(a)) < 6.0f) a--;
        while (c < n - 1 && length(xz(c) - xz(i)) < 6.0f) c++;
        float d1 = length(xz(i) - xz(a)), d2 = length(xz(c) - xz(i));
        if (d1 < 2.0f || d2 < 2.0f) continue;
        float k = ((route[(size_t)i].y - route[(size_t)a].y) / d1 - (route[(size_t)c].y - route[(size_t)i].y) / d2) / (0.5f * (d1 + d2));
        if (k > 1e-3f) s.route_speed[i] = std::min(s.route_speed[i], std::sqrt(0.9f * 9.81f / k));
    }
    // braking zones: the speed must be reachable from every later limit
    for (int i = n - 2; i >= 0; i--) {
        float ds = length(xz(i + 1) - xz(i));
        s.route_speed[i] = std::min(s.route_speed[i], std::sqrt(s.route_speed[i + 1] * s.route_speed[i + 1] + 2.0f * brake_acc * ds));
    }
    if (!loop) s.route_speed[n - 1] = 0;
}

// Race line follower: nearest point search around the last waypoint, pure-pursuit steering at a speed dependent
// look-ahead, speed from the precomputed limits. Returns false when the route is finished.
static void race_input(AIState& s, vec3 pos, vec3 fwd, float speed, VehicleInput& in) {
    const int n = (int)s.route.size();
    int best = s.wp;
    float bd = 1e30f;
    for (int i = std::max(0, s.wp - 10); i < std::min(n, s.wp + 60); i++) {
        vec2 dv(s.route[i].x - pos.x, s.route[i].z - pos.z);
        float d = dot(dv, dv);
        if (d < bd) {
            bd = d;
            best = i;
        }
    }
    if (std::sqrt(bd) > 25.0f) // lost (reset / crash): full search
        for (int i = 0; i < n; i++) {
            vec2 dv(s.route[i].x - pos.x, s.route[i].z - pos.z);
        float d = dot(dv, dv);
            if (d < bd) {
                bd = d;
                best = i;
            }
        }
    s.wp = best;
    float look = clampf(5.0f + 0.55f * std::fabs(speed), 6.0f, 24.0f);
    // pure pursuit cuts a corner by about look^2 / 8R: in hairpins look closer (walls / planters on the inside)
    if (s.route_radius.size() == s.route.size()) {
        float rmin = 1e4f, acc_r = 0;
        for (int i = best; i < n - 1 && acc_r < look; i++) {
            rmin = std::min(rmin, s.route_radius[i]);
            acc_r += length(vec2(s.route[i + 1].x - s.route[i].x, s.route[i + 1].z - s.route[i].z));
        }
        look = std::min(look, std::max(3.5f, 0.75f * rmin));
    }
    int t = best;
    float acc = 0;
    while (acc < look) {
        int nx = t + 1;
        if (nx >= n) {
            if (!s.route_loop) break;
            nx = 0;
        }
        acc += length(vec2(s.route[nx].x - s.route[t].x, s.route[nx].z - s.route[t].z));
        t = nx;
        if (t == best) break;
    }
    vec3 to = s.route[t] - pos;
    to.y = 0;
    vec3 f2 = normalize_or(vec3(fwd.x, 0, fwd.z), vec3(0, 0, 1));
    vec3 t2 = normalize_or(to, f2);
    float ang = std::atan2(dot(cross(f2, t2), vec3(0, 1, 0)), dot(f2, t2));
    // cross-track term: the look-ahead alone lets the car settle on the inside of long bends (roadside fences)
    const int nb = std::min(best + 1, n - 1), pb = nb - 1;
    vec2 tr(s.route[nb].x - s.route[pb].x, s.route[nb].z - s.route[pb].z);
    float trl = length(tr);
    tr = trl > 1e-4f ? tr * (1.0f / trl) : vec2(f2.x, f2.z);
    float e = dot(vec2(pos.x - s.route[best].x, pos.z - s.route[best].z), vec2(tr.y, -tr.x)); // + = left of the line
    in.steer = clampf(-ang * 2.2f + clampf(0.25f * e, -0.35f, 0.35f), -1, 1);
    float want = s.route_speed[std::min(n - 1, best + 3)];
    want *= 1.0f - 0.5f * clampf((std::sqrt(bd) - 3.0f) / 8.0f, 0, 1); // off the line: slow down
    if (!s.route_loop && best >= n - 3) want = 0;
    if (speed < want - 0.5f) in.throttle = clampf((want - speed) * 0.35f + 0.25f, 0, 1);
    else if (speed > want + 0.5f) in.brake = clampf((speed - want) * 0.25f, 0, 1);
}

void ai_update_all(Game& g, float dt, bool derby) {
    Rng rng((uint64_t)(g.world.time() * 1000.0) + 1);
    for (auto& vp : g.vehicles) {
        Vehicle& v = *vp;
        if (!v.ai || (v.is_player && !v.ai_state.autopilot)) continue;
        AIState& s = v.ai_state;
        vec3 pos = v.position(), fwd = v.forward(), vel = v.velocity();
        float speed = dot(vel, fwd);
        vec3 target;
        if (derby) {
            s.retarget_timer -= dt;
            if (!s.target || s.retarget_timer <= 0) {
                float best = 1e9f;
                s.target = nullptr;
                for (auto& o : g.vehicles) {
                    if (o.get() == &v) continue;
                    float d = length(o->position() - pos) * rng.range(0.7f, 1.3f);
                    if (d < best) {
                        best = d;
                        s.target = o.get();
                    }
                }
                s.retarget_timer = rng.range(3.0f, 6.0f);
            }
            target = s.target ? s.target->position() : vec3(0);
            s.target_speed = 14.0f;
        } else if (!s.route_speed.empty() && s.route_speed.size() == s.route.size()) {
            VehicleInput in;
            if (std::fabs(speed) < 0.8f) s.stuck_timer += dt;
            else s.stuck_timer = 0;
            if (s.stuck_timer > 2.5f && !(s.wp >= (int)s.route.size() - 3 && !s.route_loop)) {
                s.reverse_timer = 1.8f;
                s.stuck_timer = 0;
            }
            if (s.reverse_timer > 0) {
                s.reverse_timer -= dt;
                race_input(s, pos, fwd, speed, in);
                in.steer = -in.steer;
                in.throttle = 0;
                in.brake = 1.0f;
            } else {
                race_input(s, pos, fwd, speed, in);
            }
            v.set_input(in);
            continue;
        } else if (!s.route.empty()) {
            target = s.route[s.wp % s.route.size()];
            vec3 d = target - pos;
            d.y = 0;
            if (length(d) < 8.0f && s.route.size() > 1) s.wp = (s.wp + 1) % (int)s.route.size();
        } else {
            // wander inside the arena
            vec3 d = s.wander_target - pos;
            d.y = 0;
            if (length(d) < 10.0f || length2(s.wander_target) == 0) s.wander_target = vec3(rng.range(-50, 50), 0, rng.range(-50, 50));
            target = s.wander_target;
        }
        vec3 to = target - pos;
        to.y = 0;
        vec3 f2 = normalize(vec3(fwd.x, 0, fwd.z));
        vec3 t2 = normalize(to);
        float ang = std::atan2(dot(cross(f2, t2), vec3(0, 1, 0)), dot(f2, t2)); // + = target to the left
        VehicleInput in;
        // stuck detection -> reverse for a while
        if (std::fabs(speed) < 0.8f) s.stuck_timer += dt;
        else s.stuck_timer = 0;
        if (s.stuck_timer > 2.5f) {
            s.reverse_timer = 1.8f;
            s.stuck_timer = 0;
        }
        if (s.reverse_timer > 0) {
            s.reverse_timer -= dt;
            in.brake = 1.0f; // brake held at standstill engages reverse (auto gearbox)
            in.steer = clampf(ang * 2.0f, -1, 1);
        } else {
            in.steer = clampf(-ang * 2.0f, -1, 1);
            float want = s.target_speed * (1.0f - 0.5f * std::min(1.0f, std::fabs(ang)));
            if (speed < want) in.throttle = clampf((want - speed) * 0.4f + 0.3f, 0, 1);
            else in.brake = clampf((speed - want) * 0.2f, 0, 1);
        }
        v.set_input(in);
    }
}

} // namespace bl
