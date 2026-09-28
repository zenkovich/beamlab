// Orbit / chase / free-fly camera controller.
#pragma once

#include "gfx/renderer.h"

namespace bl {

struct CameraInput {
    vec2 mouse_delta;     // pixels
    float wheel = 0;
    bool rotate = false;  // RMB held
    bool pan = false;     // MMB held
    vec3 move;            // free-fly: x strafe, y up/down, z forward
    bool fast = false;
};

class CameraController {
public:
    enum Mode { ORBIT = 0, CHASE = 1, FREE = 2, COCKPIT = 3 };
    Mode mode = CHASE;
    float yaw = 0.8f, pitch = 0.35f, dist = 9.0f;
    vec3 target{0, 1, 0};
    vec3 free_pos{0, 5, 15};
    float free_yaw = 0, free_pitch = -0.2f;
    float fov = 60.0f;

    // follow_pos/follow_fwd describe the followed object (vehicle); has_follow=false for free camera.
    void update(float dt, const CameraInput& in, bool has_follow, vec3 follow_pos, vec3 follow_fwd, vec3 follow_vel, vec3 cockpit_pos, vec3 cockpit_up);
    void snap_behind(vec3 fwd);
    Camera camera() const { return m_cam; }
    void look_free(vec3 pos, vec3 target);
    // switch to the free camera where the current camera is (whatever the mode was), looking the same way
    void enter_free() {
        if (mode != FREE) look_free(m_cam.pos, m_cam.target);
    }
    void set_free(vec3 pos, float yaw_, float pitch_) {
        free_pos = pos;
        free_yaw = yaw_;
        free_pitch = pitch_;
    }

private:
    Camera m_cam;
    vec3 m_smooth_target{0, 1, 0};
    float m_chase_yaw = 0;
    bool m_init = false;
};

} // namespace bl
