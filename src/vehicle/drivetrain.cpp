#include "vehicle/drivetrain.h"
#include "core/util.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace bl {

using namespace phys;

// ------------------------------------------------------------------ torque curve
void TorqueCurve::set(const ror::TorqueCurveDef& d) {
    pts = d.points;
    if (pts.size() >= 2) return;
    std::string m = to_lower(d.model);
    if (m == "diesel") pts = {{0, 0}, {1000, 0.79f}, {1500, 0.9f}, {2000, 0.97f}, {2500, 0.99f}, {3000, 0.9f}, {3500, 0.77f}};
    else if (m == "turbodiesel") pts = {{0, 0}, {1000, 0.89f}, {1500, 1}, {2000, 1}, {2500, 1}, {3000, 1}, {3500, 1}, {4000, 0.89f}, {4500, 0.81f}, {5000, 0.65f}};
    else if (m == "gas") pts = {{0, 0}, {1000, 0.75f}, {1500, 0.8f}, {2000, 0.88f}, {2500, 0.93f}, {3000, 1}, {3500, 0.98f}, {4000, 0.93f}, {4500, 0.9f}, {5000, 0.88f}, {5500, 0.83f}, {6000, 0.78f}};
    else if (m == "turbogas") pts = {{0, 0}, {1000, 0.67f}, {1500, 1}, {2000, 1}, {2500, 1}, {3000, 1}, {3500, 1}, {4000, 1}, {4500, 1}, {5000, 0.95f}, {5500, 0.88f}, {6000, 0.83f}};
    else pts = {{0, 1}, {1000, 1}, {10000, 1}};
}

float TorqueCurve::eval(float rpm) const {
    if (pts.empty()) return 1.0f;
    if (rpm <= pts.front().x) return pts.front().y;
    if (rpm >= pts.back().x) return pts.back().y;
    for (size_t i = 1; i < pts.size(); i++)
        if (rpm <= pts[i].x) {
            float t = (rpm - pts[i - 1].x) / std::max(1e-3f, pts[i].x - pts[i - 1].x);
            return lerpf(pts[i - 1].y, pts[i].y, t);
        }
    return pts.back().y;
}

// ------------------------------------------------------------------ differentials (RoR Differentials.cpp)
void Differential::compute(float in, float s0, float s1, float dt, float& o0, float& o1, float stiff) {
    switch (modes.empty() ? OPEN : modes[mode % modes.size()]) {
    case SPLIT:
        o0 = o1 = in * 0.5f;
        break;
    case OPEN: {
        float a0 = std::fabs(s0), a1 = std::fabs(s1);
        float r = std::min(a0, a1) > 1.0f ? a0 / (a0 + a1) : 0.5f;
        o0 = in * clampf(r, 0.1f, 0.9f);
        o1 = in * clampf(1.0f - r, 0.1f, 0.9f);
        break;
    }
    case VISCOUS:
        o0 = in * 0.5f - (s0 - s1) * 10000.0f * stiff;
        o1 = in * 0.5f + (s0 - s1) * 10000.0f * stiff;
        break;
    case LOCKED:
        delta_rot += (s0 - s1) * dt;
        delta_rot = clampf(delta_rot, -0.05f, 0.05f);
        o0 = in * 0.5f - (delta_rot * 1e6f + (s0 - s1) * 1e4f) * stiff;
        o1 = in * 0.5f + (delta_rot * 1e6f + (s0 - s1) * 1e4f) * stiff;
        break;
    }
}

// ------------------------------------------------------------------ init
void Drivetrain::init(const ror::Document& d, const SoftBody& b) {
    const auto& e = d.engine;
    has_engine = e.present && !e.gears.empty();
    if (has_engine) {
        min_rpm = std::fabs(e.shift_down_rpm);
        max_rpm = std::fabs(e.shift_up_rpm);
        torque = e.torque;
        idle_rpm = std::min(min_rpm, 800.0f);
        stall_rpm = 300;
        inertia = 10;
        clutch_force = 10000;
        braking_torque = -torque / 5.0f;
        const auto& o = d.engoption;
        if (o.present) {
            if (o.inertia > 0) inertia = o.inertia;
            type = o.type;
            clutch_force = o.clutch_force > 0 ? o.clutch_force : (type == 'c' || type == 'e' ? 5000.0f : 10000.0f);
            // RoR quirk: field 4 is used as clutch time and field 5 as shift time
            if (o.f4 > 0) clutch_time = o.f4;
            if (o.f5 > 0) shift_time = o.f5;
            if (o.post_shift_time > 0) post_shift_time = o.post_shift_time;
            if (o.idle_rpm > 0) idle_rpm = o.idle_rpm;
            if (o.stall_rpm > 0) stall_rpm = o.stall_rpm;
            if (o.max_idle_mix > 0) max_idle_mix = o.max_idle_mix;
            if (o.min_idle_mix > 0) min_idle_mix = o.min_idle_mix;
            if (o.braking_torque > 0) braking_torque = -o.braking_torque;
        }
        clutch_time = clampf(clutch_time, 0.0f, 0.9f * shift_time);
        stall_rpm = clampf(stall_rpm, 0.0f, 0.9f * idle_rpm);
        ratios.clear();
        ratios.push_back(-e.rev_ratio * e.diff_ratio);
        ratios.push_back(e.neutral_ratio * e.diff_ratio);
        for (float g : e.gears) ratios.push_back(g * e.diff_ratio);
        num_gears = (int)e.gears.size();
        curve.set(d.torquecurve);
        rpm = idle_rpm;
        gear = 1;
        running = true;
    }
    brake_force = d.brakes.force;
    parking_force = d.brakes.parking > 0 ? d.brakes.parking : 2.0f * brake_force;
    axles_section = d.has_axles_section;
    auto pulse_time = [](float pulse) { return (pulse <= 1.0f || pulse >= 2000.0f) ? 1.0f / 2000.0f : 1.0f / pulse; };
    tc_on = d.traction.present && d.traction.on;
    tc_ratio = d.traction.regulation;
    tc_slip = d.traction.wheelslip > 0 ? d.traction.wheelslip : 0.25f;
    tc_pulse_time = pulse_time(d.traction.pulse);
    abs_on = d.antilock.present && d.antilock.on;
    abs_ratio = d.antilock.regulation;
    abs_min_speed = std::max(0.5f, d.antilock.min_speed_kmh / 3.6f);
    abs_pulse_time = pulse_time(d.antilock.pulse);
    speed_limit = d.speed_limit;
    tc_state.assign(b.wheels.size(), 1.0f);
    abs_state.assign(b.wheels.size(), 1.0f);

    // propelled wheels + differentials
    propelled.clear();
    for (int i = 0; i < (int)b.wheels.size(); i++)
        if (b.wheels[i].propulsed) propelled.push_back(i);
    wheel_diffs.clear();
    axle_diffs.clear();
    if (!d.axles.empty()) {
        for (const auto& a : d.axles) {
            int w0 = -1, w1 = -1;
            for (int i = 0; i < (int)b.wheels.size(); i++) {
                const Wheel& w = b.wheels[i];
                auto match = [&](int x, int y) {
                    return ((int)w.axle0 == x && (int)w.axle1 == y) || ((int)w.axle0 == y && (int)w.axle1 == x);
                };
                if (w0 < 0 && match(a.w1a, a.w1b)) w0 = i;
                else if (w1 < 0 && match(a.w2a, a.w2b)) w1 = i;
            }
            if (w0 < 0 || w1 < 0) continue;
            WheelDiff wd{w0, w1, {}};
            if (!a.modes.empty()) {
                wd.diff.modes.clear();
                for (char c : a.modes)
                    wd.diff.modes.push_back(c == 'l' ? Differential::LOCKED : c == 's' ? Differential::SPLIT : c == 'v' ? Differential::VISCOUS : Differential::OPEN);
            }
            wheel_diffs.push_back(wd);
        }
    }
    if (wheel_diffs.empty()) {
        for (size_t i = 0; i + 1 < propelled.size(); i += 2) {
            WheelDiff wd{propelled[i], propelled[i + 1], {}};
            wd.diff.modes = {Differential::VISCOUS};
            wheel_diffs.push_back(wd);
        }
    }
    for (size_t i = 0; i + 1 < wheel_diffs.size(); i++) {
        AxleDiff ad{(int)i, (int)i + 1, {}};
        ad.diff.modes = {axles_section ? Differential::LOCKED : Differential::VISCOUS};
        axle_diffs.push_back(ad);
    }
    // steering hydros / commands are filled in by the builder
}

void Drivetrain::shift(int d) {
    if (shifting) return;
    shifting = true;
    shift_val = d;
    shift_clock = 0;
}

// ------------------------------------------------------------------ engine (RoR Engine::UpdateEngine)
void Drivetrain::update_engine(float dt) {
    if (!has_engine) return;
    // automatic clutch / shift sequencing
    if (shifting) {
        shift_clock += dt;
        if (shift_val != 0) {
            float dc = std::min(shift_time - clutch_time, clutch_time);
            if (shift_clock <= dc && dc > 0) {
                float r = sqr(1.0f - shift_clock / dc);
                clutch = std::min(r, clutch);
                cur_acc = std::min(r, auto_acc);
            } else {
                gear = std::max(-1, std::min(num_gears, gear + shift_val));
                shift_val = 0;
            }
        }
        if (shift_clock > shift_time) {
            cur_acc = auto_acc;
            shifting = false;
            post_shifting = true;
            post_clock = 0;
        } else if (!shift_val && gear && shift_clock >= shift_time - clutch_time && clutch_time > 0) {
            cur_acc = auto_acc * 0.5f * std::sqrt((shift_clock - (shift_time - clutch_time)) / clutch_time);
        }
    } else {
        cur_acc = auto_acc;
    }
    if (post_shifting) {
        post_clock += dt;
        if (post_clock > post_shift_time) post_shifting = false;
        else if (auto_acc > 0) cur_acc = auto_acc * 0.5f * (1.0f + post_clock / post_shift_time);
        else if (gear && wheel_rpm > rpm / ratios[gear + 1]) clutch = std::max(clutch, std::sqrt(post_clock / post_shift_time));
    }
    float declutch = 0.75f * min_rpm + 0.25f * stall_rpm;
    if (gear == 0 || rpm < declutch) clutch = 0;
    else if (rpm < min_rpm && min_rpm > declutch) clutch = std::min(sqr((rpm - declutch) / (min_rpm - declutch)), clutch);
    else if (!shift_val && rpm > min_rpm && clutch < 1) {
        float ratio = ratios[gear + 1];
        float thr = 1.5f * engine_power(rpm) * std::fabs(ratio);
        float spin = rpm / ratio;
        float reT = clampf((spin - wheel_rpm) * clutch_force, -thr, thr) / ratio;
        float range = (max_rpm - min_rpm) * 0.4f * std::sqrt(std::max(0.2f, cur_acc));
        float eT = engine_power(rpm) * std::min(cur_acc, 0.9f) * std::min((rpm - min_rpm) / std::max(1.0f, range), 1.0f);
        if (reT > 1e-3f) clutch = std::max(clutch, std::min(eT, std::fabs(reT)) / reT);
        else clutch = std::max(clutch, cur_acc > 0.05f ? 1.0f : clutch);
    }
    // over-rev protection
    if (gear != 0) {
        float wr = std::fabs(wheel_rpm * ratios[gear + 1]);
        if (wr > 1.25f * max_rpm) clutch = std::min(clutch, 1.0f / (1.0f + (wr - 1.25f * max_rpm) / 2.0f));
    }
    clutch = clampf(clutch, 0, 1);

    // engine torque balance
    float idle_mix = rpm <= idle_rpm ? max_idle_mix : min_idle_mix;
    float acc = std::max(cur_acc, idle_mix);
    float T = 0;
    if (running) T += braking_torque * rpm / max_rpm * (1.0f - cur_acc);
    if (running && rpm < 1.25f * max_rpm) T += engine_power(rpm) * acc;
    if (running && rpm < stall_rpm && type != 'e') {
        // stalled: restart immediately (the prototype has no ignition controls)
        rpm = idle_rpm;
    }
    if (gear != 0) T -= clutch_torque / ratios[gear + 1];
    rpm += dt * T / inertia;
    if (gear != 0) {
        float ratio = ratios[gear + 1];
        // clutch capacity in the current gear (RoR used the 1st gear ratio: a 2-3x torque spike after every
        // upshift that broke the driven wheels loose)
        float thr = 1.3f * std::max(torque, engine_power(rpm)) * std::fabs(ratio);
        float spin = rpm / ratio;
        float ct = clampf((spin - wheel_rpm) * clutch * clutch_force, -thr, thr);
        ct *= 1.0f - std::exp(-std::fabs(spin - wheel_rpm));
        clutch_torque = ct;
    } else {
        clutch_torque = 0;
    }
    rpm = std::max(0.0f, rpm);
}

void Drivetrain::auto_gearbox(float dt) {
    if (!has_engine || shifting) return;
    shift_cooldown -= dt;
    if (gear <= 0 || shift_cooldown > 0) return;
    // shift points like a road car's automatic: light throttle shifts early (low rpm, high gear), full throttle
    // holds the gear up to the shift-up rpm; kick-down when flooring it at low rpm
    const float a = clampf(auto_acc, 0, 1);
    const float span = max_rpm - min_rpm;
    const float up_rpm = lerpf(min_rpm + 0.3f * span, max_rpm - 100.0f, smoothstepf(0.15f, 0.95f, a));
    if (gear < num_gears && rpm > up_rpm && clutch > 0.99f) {
        float after = rpm * std::fabs(ratios[gear + 2] / ratios[gear + 1]);
        if (after > min_rpm * 1.1f) {
            shift(+1);
            shift_cooldown = 0.8f;
            return;
        }
    }
    if (gear > 1) {
        float lower = std::fabs(wheel_rpm * ratios[gear]); // engine rpm in the next lower gear
        bool lugging = rpm < min_rpm * 1.02f;
        bool kick = a > 0.9f && rpm < min_rpm + 0.25f * span && lower < 0.85f * max_rpm;
        if ((lugging || kick) && lower < max_rpm - 150) {
            shift(-1);
            shift_cooldown = 0.6f;
        }
    }
}

// ------------------------------------------------------------------ per substep
void Drivetrain::update(float dt, const VehicleInput& in, SoftBody& b) {
    speed = b.wheel_speed;
    {
        vec3 f = b.nodes[cam_center].p - b.nodes[cam_back].p;
        float fl = length(f);
        ground_speed = fl > 1e-4f ? dot(b.nodes[cam_center].v, f / fl) : 0.0f;
    }
    wheel_rpm = b.wheel_spin * 9.5492965855f;

    // ---- steering (RoR digital path with speed coupling + self centering; rates scaled by the steering settings)
    float cmd = clampf(in.steer, -1, 1);
    const float centre_rate = 1.5f * steer_return;
    if (cmd != 0) {
        float rate = std::max(1.2f, 30.0f / (10.0f + steer_speed_sens * std::fabs(speed / 2.0f))) * steer_speed;
        // counter-steering: the wheels come back through the centre at the return rate on top of the turn rate
        if (dir_state * cmd < 0) rate += centre_rate;
        float step = dt * rate;
        if (dir_state > cmd) dir_state = std::max(cmd, dir_state - step);
        else dir_state = std::min(cmd, dir_state + step);
    }
    if (cmd == 0 || std::fabs(dir_state) > std::fabs(cmd)) {
        float step = dt * centre_rate;
        dir_state = std::fabs(dir_state) > step ? dir_state - signf(dir_state) * step : 0.0f;
    }
    for (const HydroCtl& h : hydros) {
        Beam& bm = b.beams[h.beam];
        float cstate = 0;
        int div = 0;
        if (h.speed_dep) {
            if (std::fabs(speed) < 12.0f) cstate += dir_state * (12.0f - std::fabs(speed)) / 12.0f;
            div++;
        }
        if (h.steer) {
            cstate += dir_state;
            div++;
        }
        if (div) {
            cstate = clampf(cstate, -1, 1) / div;
            bm.L = bm.L0 * (1.0f - cstate * h.factor);
        }
    }
    // ---- commands (hold key)
    for (const CommandCtl& c : commands) {
        Beam& bm = b.beams[c.beam];
        float dir = 0;
        if (in.command_key != 0 && in.command_key == c.key_extend) dir = 1;
        else if (in.command_key != 0 && in.command_key == c.key_contract) dir = -1;
        float ratio = bm.L / std::max(1e-4f, bm.L0);
        if (dir > 0 && ratio < c.max_ratio) bm.L += c.rate_long * bm.L0 * dt;
        else if (dir < 0 && ratio > c.min_ratio) bm.L -= c.rate_short * bm.L0 * dt;
        else if (dir == 0 && c.autocenter) {
            float center = 0.5f * (c.min_ratio + c.max_ratio);
            float step = c.rate_long * bm.L0 * dt;
            float target = center * bm.L0;
            bm.L = std::fabs(bm.L - target) < step ? target : bm.L + signf(target - bm.L) * step;
        }
    }

    // ---- driver input (arcade: brake at standstill engages reverse)
    float thr_raw = clampf(in.throttle, 0, 1), brk_raw = clampf(in.brake, 0, 1);
    parking = in.handbrake || (!has_engine && b.wheels.size() > 0);
    if (has_engine) {
        if (gear > 0 && brk_raw > 0.1f && thr_raw < 0.1f && std::fabs(speed) < 0.5f) {
            reverse_timer += dt;
            if (reverse_timer > 0.3f) {
                gear = -1;
                reverse_timer = 0;
                shifting = false;
                shift_val = 0;
            }
        } else if (gear < 0 && thr_raw > 0.1f && brk_raw < 0.1f && std::fabs(speed) < 0.5f) {
            reverse_timer += dt;
            if (reverse_timer > 0.3f) {
                gear = 1;
                reverse_timer = 0;
            }
        } else {
            reverse_timer = 0;
        }
        if (gear < 0) {
            auto_acc = brk_raw;
            brake_in = thr_raw;
        } else {
            auto_acc = thr_raw;
            brake_in = brk_raw;
        }
        if (speed_limit > 0) auto_acc = clampf((speed_limit - std::fabs(speed / 1.02f)) * 2.0f, 0.0f, auto_acc);
        // hold the vehicle when idle on level ground (RoR UpdateTruckFeatures)
        if (auto_acc < 0.05f && brake_in == 0 && !parking) brake_in = std::max(0.0f, 0.2f - std::fabs(speed)) / 0.2f;
        auto_gearbox(dt);
        update_engine(dt);
    } else {
        brake_in = brk_raw;
    }

    // ---- torque distribution (RoR CalcDifferentials)
    // (a ring tyre's rim alone takes the differential's coupling at the step: a tenth of the node wheels' inertia, the
    // coupling of 1e4 N m per m/s between two of them was unstable - their torques swung +-30 kN m, the wheels spun; kept
    // under half the rim's limit I / (dt R))
    auto stiff_of = [&](const Wheel& a, const Wheel& c) {
        float s = 1.0f;
        for (const Wheel* w : {&a, &c})
            if (w->ring) s = std::min(s, 0.5f * w->inertia / (dt * std::max(0.05f, w->radius) * 1e4f));
        return s;
    };
    if (has_engine && !propelled.empty()) {
        float T = clutch_torque / (float)propelled.size();
        if (axles_section) T *= 2.0f;
        for (int wi : propelled)
            if (!b.wheels[wi].detached) b.wheels[wi].torque += T;
        for (auto& ad : axle_diffs) {
            const WheelDiff& A = wheel_diffs[ad.a0];
            const WheelDiff& B = wheel_diffs[ad.a1];
            Wheel& a0 = b.wheels[A.w0];
            Wheel& a1 = b.wheels[A.w1];
            Wheel& b0 = b.wheels[B.w0];
            Wheel& b1 = b.wheels[B.w1];
            float in_t = a0.torque + a1.torque + b0.torque + b1.torque;
            float sa = 0.5f * (a0.speed + a1.speed), sb = 0.5f * (b0.speed + b1.speed);
            float oa, ob;
            ad.diff.compute(in_t, sa, sb, dt, oa, ob, std::min(stiff_of(a0, a1), stiff_of(b0, b1)));
            a0.torque = a1.torque = oa * 0.5f;
            b0.torque = b1.torque = ob * 0.5f;
        }
        for (auto& wd : wheel_diffs) {
            Wheel& w0 = b.wheels[wd.w0];
            Wheel& w1 = b.wheels[wd.w1];
            float in_t = w0.torque + w1.torque, o0, o1;
            wd.diff.compute(in_t, w0.speed, w1.speed, dt, o0, o1, stiff_of(w0, w1));
            w0.torque = o0;
            w1.torque = o1;
        }
    }
    // ---- traction control, ABS and brakes (RoR CalcWheels)
    tc_clock += dt;
    if (tc_clock >= tc_pulse_time) { tc_clock = 0; tc_pulse = !tc_pulse; }
    abs_clock += dt;
    if (abs_clock >= abs_pulse_time) { abs_clock = 0; abs_pulse = !abs_pulse; }
    const float cur = std::fabs(ground_speed);
    for (size_t i = 0; i < b.wheels.size(); i++) {
        Wheel& w = b.wheels[i];
        float ws = std::fabs(w.speed);
        float slip = std::fabs(w.speed - ground_speed) / std::max(1.0f, cur);
        const bool assist = !tc_on && traction_assist;
        const float slip_limit = assist ? 0.12f : tc_slip;
        if ((tc_on || assist) && std::fabs(w.torque) > 0 && ws > cur && slip > slip_limit) {
            if (tc_pulse || assist) tc_state[i] = std::pow(cur / std::max(ws, 1e-3f), assist ? 2.0f : tc_ratio);
            w.torque *= std::pow(tc_state[i], std::min(ws / 5.0f, 1.0f));
        } else {
            tc_state[i] = 1.0f;
        }
        w.brake = 0;
        if (w.braked == 0) continue;
        float ab = brake_force * brake_in;
        float hb = (parking && w.braked != 4) ? parking_force : 0.0f;
        float db = 0;
        if (ws < 20.0f && ((w.braked == 2 && dir_state > 0) || (w.braked == 3 && dir_state < 0))) db = brake_force * std::fabs(dir_state);
        float adb = ab + db;
        if (abs_on && cur > abs_min_speed && cur > ws && adb > 0 && slip > 0.25f) {
            if (abs_pulse) abs_state[i] = std::pow(ws / std::max(cur, 1e-3f), abs_ratio);
            adb *= abs_state[i];
        } else {
            abs_state[i] = 1.0f;
        }
        w.brake = adb + hb;
    }
    // (BL_DRIVEDBG=1: the engine, the clutch and what reaches the wheels, twice a second)
    static const bool dbg = getenv("BL_DRIVEDBG") != nullptr;
    static float dbg_clock = 0;
    if (dbg && (dbg_clock += dt) >= 0.5f) {
        dbg_clock = 0;
        printf("drive: rpm %5.0f gear %d acc %.2f clutch %.2f clutch T %6.0f wheel rpm %6.1f ground %5.2f m/s | wheels", rpm, gear, cur_acc, clutch, clutch_torque, wheel_rpm,
               ground_speed);
        for (size_t i = 0; i < b.wheels.size(); i++) printf(" [%.1f rad/s T %.0f tc %.2f br %.0f]", b.wheels[i].speed, b.wheels[i].torque, tc_state[i], b.wheels[i].brake);
        printf("\n");
    }
}

} // namespace bl
