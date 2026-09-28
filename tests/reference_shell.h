// Reference (pre-optimisation) triangle-element force kernel, see reference_shell.cpp.
#pragma once
#include "phys/softbody.h"

namespace bl::phys::ref {

struct RefBody : SoftBody {
    explicit RefBody(const SoftBody& b) : SoftBody(b) {}
    void ref_forces(float h, int step = 0, int sub = 1);
    std::vector<vec3> shell_acc[3]; // shell forces held per rate class
};

} // namespace bl::phys::ref
