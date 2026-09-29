// RoR definition -> SoftBody (ActorSpawner equivalent).
#pragma once

#include "phys/softbody.h"
#include "vehicle/drivetrain.h"
#include "vehicle/ror_def.h"

#include <array>
#include <string>
#include <vector>

namespace bl {

// Incremental 3D convex hull of a small point set: triangles as point indices, counter-clockwise seen from outside
// (the fallback collision shell of a vehicle without collision cabs; the model editor's collision hull)
std::vector<std::array<int, 3>> convex_hull(const std::vector<vec3>& pts);

class VehicleBuilder {
public:
    // Builds the body in definition (vehicle-local) space. Body node i == definition node i.
    static bool build(const ror::Document& d, phys::SoftBody& body, Drivetrain& drive, std::vector<std::string>& warnings);
};

} // namespace bl
