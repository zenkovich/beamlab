// Very simple AI drivers for demo / stress scenes.
#pragma once

#include "core/math.h"

#include <vector>

namespace bl {
class Game;
class Vehicle;
// Drives every vehicle with `ai == true`. derby: aim at the nearest other vehicle.
void ai_update_all(Game& g, float dt, bool derby = false);
void ai_set_route(Vehicle& v, const std::vector<vec3>& route, float speed);
// Race line (dense centre line, a few metres apart): speed limits from the curvature (lat_acc) with braking
// zones (brake_acc) before corners; the car follows a look-ahead point. Stops at the end unless `loop`.
void ai_set_race_route(Vehicle& v, const std::vector<vec3>& route, float max_speed, float lat_acc, float brake_acc, bool loop);
} // namespace bl
