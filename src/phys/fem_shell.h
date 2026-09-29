// Building shells of triangle elements (FemFrame::tris): nodes shared by position, each triangle's mass lumped on its
// corners, a collision triangle with it (two-sided, torn when it tears out). Plates, boxes, any mesh.
#pragma once

#include "phys/softbody.h"

#include <map>
#include <tuple>
#include <vector>

namespace bl::phys {

class ShellMesher {
public:
    ShellMesher(SoftBody& b, uint16_t node_flags = NF_GROUND | NF_CONTACTER) : m_b(b), m_flags(node_flags) {}
    // the node at p (made if none is within a millimetre)
    uint32_t node(vec3 p);
    // a triangle element (a, b, c body nodes) of `section`; its mass onto the corners; collide: a collision triangle
    uint32_t tri(uint32_t a, uint32_t b, uint32_t c, uint16_t section, bool collide = true, int32_t tag = -1);
    // a grid of nu x nv cells from `origin` along du and dv (the cells' edges), two triangles a cell, their diagonals
    // alternating (no direction favoured); returns the nodes, (nu + 1) x (nv + 1), row by row along du
    std::vector<uint32_t> grid(vec3 origin, vec3 du, vec3 dv, int nu, int nv, uint16_t section, bool collide = true);
    // the nodes' inverse masses from their masses (fixed nodes stay fixed): after building
    void finish();

private:
    SoftBody& m_b;
    uint16_t m_flags;
    std::map<std::tuple<long, long, long>, uint32_t> m_at;
};

} // namespace bl::phys
