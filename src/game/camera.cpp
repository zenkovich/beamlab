#include "game/camera.h"

#include <cmath>

namespace bl {

static float wrap_angle(float a) {
    while (a > kPi) a -= 2 * kPi;
    while (a < -kPi) a += 2 * kPi;
    return a;
}

void CameraController::snap_behind(vec3 fwd) {
    m_chase_yaw = std::atan2(-fwd.x, -fwd.z);
    yaw = m_chase_yaw;
    m_init = false;
}

void CameraController::update(float dt, const CameraInput& in, bool has_follow, vec3 fpos, vec3 ffwd, vec3 fvel, vec3 cockpit_pos, vec3 cockpit_up) {
    const float rot_speed = 0.005f;
    if (!has_follow && mode != FREE) enter_free(); // (the followed vehicle is gone: stay where the view was)
    if (mode == FREE) {
        if (in.rotate) {
            free_yaw -= in.mouse_delta.x * rot_speed;
            free_pitch = clampf(free_pitch - in.mouse_delta.y * rot_speed, -1.5f, 1.5f);
        }
        vec3 fwd(-std::sin(free_yaw) * std::cos(free_pitch), std::sin(free_pitch), -std::cos(free_yaw) * std::cos(free_pitch));
        vec3 right = normalize(cross(fwd, vec3(0, 1, 0)));
        float sp = (in.fast ? 60.0f : 15.0f) * dt;
        free_pos += right * (in.move.x * sp) + vec3(0, in.move.y * sp, 0) + fwd * (in.move.z * sp);
        if (in.wheel != 0) free_pos += fwd * in.wheel * 2.0f;
        m_cam.pos = free_pos;
        m_cam.target = free_pos + fwd;
        m_cam.fov_deg = fov;
        return;
    }
    if (!m_init) {
        m_smooth_target = fpos;
        m_init = true;
    }
    if (mode == COCKPIT) {
        vec3 up = normalize_or(cockpit_up, vec3(0, 1, 0));
        m_cam.pos = cockpit_pos;
        vec3 fwd = normalize_or(ffwd, vec3(0, 0, 1));
        // allow looking around with RMB
        if (in.rotate) {
            yaw -= in.mouse_delta.x * rot_speed;
            pitch = clampf(pitch - in.mouse_delta.y * rot_speed, -1.2f, 1.2f);
        } else {
            yaw *= std::pow(0.02f, dt);
            pitch *= std::pow(0.02f, dt);
        }
        quat q = quat::axis_angle(up, yaw) * quat::axis_angle(normalize(cross(fwd, up)), pitch);
        m_cam.target = m_cam.pos + q.rotate(fwd);
        m_cam.up = up;
        m_cam.fov_deg = 70.0f;
        m_cam.znear = 0.05f;
        return;
    }
    m_cam.up = vec3(0, 1, 0);
    m_cam.znear = 0.1f;
    // smooth target follow (critically damped-ish)
    float k = 1.0f - std::exp(-dt * 12.0f);
    m_smooth_target = lerp(m_smooth_target, fpos, k);
    target = m_smooth_target;
    if (in.wheel != 0) dist = clampf(dist * std::pow(0.9f, in.wheel), 2.5f, 120.0f);
    if (mode == CHASE) {
        // yaw follows the vehicle's heading (or velocity when moving fast)
        vec3 dir = ffwd;
        vec3 hv(fvel.x, 0, fvel.z);
        if (length(hv) > 3.0f && dot(hv, ffwd) < 0) dir = hv; // reversing: keep behind the motion
        float want = std::atan2(-dir.x, -dir.z);
        if (in.rotate) {
            yaw -= in.mouse_delta.x * rot_speed;
            pitch = clampf(pitch + in.mouse_delta.y * rot_speed, -0.3f, 1.4f);
            m_chase_yaw = yaw;
        } else {
            float diff = wrap_angle(want - m_chase_yaw);
            m_chase_yaw = wrap_angle(m_chase_yaw + diff * (1.0f - std::exp(-dt * 2.5f)));
            yaw = m_chase_yaw;
        }
    } else {
        if (in.rotate) {
            yaw -= in.mouse_delta.x * rot_speed;
            pitch = clampf(pitch + in.mouse_delta.y * rot_speed, -0.3f, 1.5f);
        }
    }
    vec3 off(std::sin(yaw) * std::cos(pitch), std::sin(pitch), std::cos(yaw) * std::cos(pitch));
    m_cam.pos = target + off * dist + vec3(0, 0.5f, 0);
    m_cam.target = target + vec3(0, 0.6f, 0);
    m_cam.fov_deg = fov;
}

} // namespace bl

namespace bl {
void CameraController::look_free(vec3 pos, vec3 target) {
    vec3 d = normalize_or(target - pos, vec3(0, 0, -1));
    free_pos = pos;
    free_pitch = std::asin(clampf(d.y, -1, 1));
    free_yaw = std::atan2(-d.x, -d.z);
    mode = FREE;
}
} // namespace bl
