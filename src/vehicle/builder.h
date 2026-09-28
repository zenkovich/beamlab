// RoR definition -> SoftBody (ActorSpawner equivalent).
#pragma once

#include "phys/softbody.h"
#include "vehicle/drivetrain.h"
#include "vehicle/ror_def.h"

#include <string>
#include <vector>

namespace bl {

class VehicleBuilder {
public:
    // Builds the body in definition (vehicle-local) space. Body node i == definition node i.
    static bool build(const ror::Document& d, phys::SoftBody& body, Drivetrain& drive, std::vector<std::string>& warnings);
};

} // namespace bl
