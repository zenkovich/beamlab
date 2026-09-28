// Engine / gearbox / clutch / differentials / brakes / steering (Rigs of Rods model).
#pragma once

#include "game/game.h"
#include "phys/softbody.h"
#include "vehicle/ror_def.h"

#include <vector>

namespace bl {

struct TorqueCurve {
    std::vector<vec2> pts; // (rpm, fraction)
    void set(const ror::TorqueCurveDef& d);
    float eval(float rpm) const;
};

struct Differential {
    enum Type { OPEN, LOCKED, SPLIT, VISCOUS };
    std::vector<Type> modes{OPEN, LOCKED};
    int mode = 0;
    float delta_rot = 0;
    void compute(float in, float s0, float s1, float dt, float& o0, float& o1);
};

struct HydroCtl {
    uint32_t beam;
    float factor;
    bool steer = true;
    bool speed_dep = false;
};

struct CommandCtl {
    uint32_t beam;
    float rate_short, rate_long; // fraction of L0 per second
    float min_ratio, max_ratio;
    int key_contract, key_extend;
    bool autocenter = false;
};

struct Drivetrain {
    // ---- engine config
    bool has_engine = false;
    char type = 't';
    float min_rpm = 800, max_rpm = 2000, torque = 1000;
    float idle_rpm = 800, stall_rpm = 300, inertia = 10, clutch_force = 10000, braking_torque = -200;
    float shift_time = 0.5f, clutch_time = 0.2f, post_shift_time = 0.2f;
    float max_idle_mix = 0.1f, min_idle_mix = 0.0f;
    std::vector<float> ratios; // [-rev*diff, neutral*diff, g1*diff, ...]
    int num_gears = 0;
    TorqueCurve curve;
    // ---- brakes
    float brake_force = 30000, parking_force = 60000;
    // ---- differentials
    bool axles_section = false;
    struct WheelDiff {
        int w0, w1;
        Differential diff;
    };
    struct AxleDiff {
        int a0, a1; // indices into wheel_diffs
        Differential diff;
    };
    std::vector<WheelDiff> wheel_diffs;
    std::vector<AxleDiff> axle_diffs;
    std::vector<int> propelled;
    // ---- traction control / ABS / speed limiter (RoR CalcWheels)
    bool tc_on = false, abs_on = false;
    static inline bool traction_assist = true; // generic traction control for vehicles whose mod has none
    // steering response (Physics menu): multipliers of RoR's digital steering rates
    static inline float steer_speed = 2.0f;       // how fast the wheels turn in (1 = RoR)
    static inline float steer_return = 2.0f;      // how fast they return to the centre (1 = RoR)
    static inline float steer_speed_sens = 1.0f;  // how much slower the steering gets with speed (1 = RoR, 0 = none)
    float tc_ratio = 1, tc_slip = 0.25f, tc_pulse_time = 0.0005f;
    float abs_ratio = 1, abs_min_speed = 0.5f, abs_pulse_time = 0.0005f;
    float speed_limit = -1;
    std::vector<float> tc_state, abs_state;
    float tc_clock = 0, abs_clock = 0;
    bool tc_pulse = true, abs_pulse = true;
    int cam_center = 0, cam_back = 0;
    float ground_speed = 0; // m/s along the vehicle forward axis
    // ---- steering / hydros / commands
    std::vector<HydroCtl> hydros;
    std::vector<CommandCtl> commands;

    // ---- state
    float rpm = 0;
    int gear = 0;
    bool running = true;
    float clutch = 0, clutch_torque = 0;
    float cur_acc = 0, auto_acc = 0;
    bool shifting = false, post_shifting = false;
    int shift_val = 0;
    float shift_clock = 0, post_clock = 0;
    float shift_cooldown = 0;
    float reverse_timer = 0;
    float dir_state = 0;
    float brake_in = 0;
    bool parking = false;
    float wheel_rpm = 0;
    float speed = 0; // m/s along vehicle forward (from wheels)

    void init(const ror::Document& d, const phys::SoftBody& b);
    // Per substep: steering, hydros, commands, engine, torque distribution, brakes.
    void update(float dt, const VehicleInput& in, phys::SoftBody& b);
    float engine_power(float r) const { return torque * curve.eval(r); }

private:
    void update_engine(float dt);
    void auto_gearbox(float dt);
    void shift(int d);
};

} // namespace bl
