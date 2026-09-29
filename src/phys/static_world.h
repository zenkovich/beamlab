// Static collision geometry: heightfield terrain, oriented boxes, vertical cylinders.
// Read-only during simulation (shared by all worker threads).
#pragma once

#include "phys/road.h"
#include <memory>

#include <atomic>

#include "core/math.h"

#include <string>
#include <vector>

namespace bl::phys {

// Surface friction model (RoR ground_models.cfg: adhesion + Stribeck curve) + simple mud layer.
struct GroundModel {
    std::string name;
    float va = 3.0f;       // adhesion velocity (m/s): below it static friction is used
    float ms = 1.2f;       // static friction coefficient
    float mc = 0.75f;      // sliding friction coefficient
    float t2 = 0.01f;      // hydrodynamic friction (s/m), capped at 5
    float vs = 6.0f;       // Stribeck velocity (m/s)
    float alpha = 2.0f;    // Stribeck exponent
    float strength = 1.0f; // ground strength (friction multiplier)
    float soft_depth = 0.0f; // fluid (mud) layer depth; 0 = solid surface
    float fluid_drag = 0.0f; // viscous drag inside the fluid layer (N s/m per node)
    float fluid_density = 0.0f;
    vec3 color{0.5f, 0.5f, 0.5f};
};

// RoR "primitiveCollision": contact force for a node given the force accumulated so far.
// Cancels the normal force, removes 80% of approach velocity + 20% penetration per step,
// then applies adhesion / Stribeck friction. `force` = accumulated force on the node (or contact side).
// (restitution >= 0: instead the approach speed turned back to that fraction of it at once - a ball's bounce)
vec3 primitive_collision(vec3 force, vec3 vel, float mass, vec3 normal, float dt, const GroundModel& gm, float pen, float friction_coef,
                         float push_max = 2.0f, float slop = 0.0f, float restitution = -1.0f);
// Tyre tread contact, evaluated per node and resolved per wheel (see World::collide_static): RoR normal
// response; friction is a velocity-level Coulomb model. `need` makes the node stick (cancels the applied
// tangential force and removes half of the slip in this step), `cap` = ms*N is the node's share of the static
// grip, `slide` the Stribeck sliding force. The contact patch of one wheel shares its grip: a node ring has only
// one or two (often lightly loaded) nodes on the ground that carry the whole drive torque.
struct TyreContact {
    vec3 normal_force{0};
    vec3 need{0};
    vec3 slide{0};
    float cap = 0;
    bool touching = false;
};
TyreContact tyre_contact(vec3 force, vec3 vel, float mass, vec3 normal, float dt, const GroundModel& gm, float pen, float friction_coef);

enum Surface : uint8_t {
    SURF_GRASS = 0, SURF_DIRT, SURF_ASPHALT, SURF_CONCRETE, SURF_GRAVEL, SURF_MUD, SURF_SAND, SURF_ROCK, SURF_WOOD, SURF_METAL, SURF_ICE,
    SURF_COUNT
};
const std::vector<GroundModel>& ground_models();

// Heightfield stored in kTile x kTile tiles. create() allocates every tile (terrain scenes); create_sparse() starts
// empty and only the tiles that are added exist (imported stages: the ground along a 10 km road). Where no tile
// exists there is no ground.
class Heightfield {
public:
    static constexpr int kTileShift = 6, kTile = 1 << kTileShift;
    static constexpr float kHole = -1e30f;
    void create(int nx, int nz, float cell, vec2 origin);
    void create_sparse(int nx, int nz, float cell, vec2 origin);
    // tile (tx, tz): heights / surfaces of kTile * kTile cells, rows along x (allocated on first use)
    float* tile_heights(int tx, int tz);
    uint8_t* tile_surfaces(int tx, int tz);
    int nx() const { return m_nx; }
    int nz() const { return m_nz; }
    float cell() const { return m_cell; }
    vec2 origin() const { return m_origin; }
    vec2 size() const { return {(m_nx - 1) * m_cell, (m_nz - 1) * m_cell}; }
    size_t tile_count() const { return m_tile_pos.size(); }
    float& h(int x, int z) { return m_th[slot(x, z)]; }
    float h(int x, int z) const {
        int t = m_tile_of[(size_t)(z >> kTileShift) * m_tx + (x >> kTileShift)];
        return t < 0 ? kHole : m_th[((size_t)t << (2 * kTileShift)) + ((size_t)(z & (kTile - 1)) << kTileShift) + (x & (kTile - 1))];
    }
    uint8_t& surf(int x, int z) { return m_ts[slot(x, z)]; }
    uint8_t surf(int x, int z) const {
        int t = m_tile_of[(size_t)(z >> kTileShift) * m_tx + (x >> kTileShift)];
        return t < 0 ? (uint8_t)0 : m_ts[((size_t)t << (2 * kTileShift)) + ((size_t)(z & (kTile - 1)) << kTileShift) + (x & (kTile - 1))];
    }
    // Call after editing heights.
    void update_bounds();

    bool inside(float x, float z) const;
    float height(float x, float z) const;
    // Height + surface normal at (x,z); returns false where there is no ground.
    bool sample(float x, float z, float& h, vec3& n) const;
    uint8_t surface_at(float x, float z) const;
    // Upper bound of the terrain height inside an xz rectangle.
    float max_height(float x0, float z0, float x1, float z1) const;
    vec3 vertex_normal(int x, int z) const;

private:
    size_t slot(int x, int z); // allocates the tile
    int m_nx = 0, m_nz = 0;
    float m_cell = 1.0f;
    vec2 m_origin;
    int m_tx = 0, m_tz = 0;              // tiles per axis
    std::vector<int32_t> m_tile_of;      // tile grid -> index into the tile storage (-1: no ground)
    std::vector<std::pair<int, int>> m_tile_pos;
    std::vector<float> m_th;             // heights, kTile * kTile per tile
    std::vector<uint8_t> m_ts;           // surfaces
    // coarse max-height blocks for culling
    static constexpr int kBlock = 16;
    int m_bx = 0, m_bz = 0;
    std::vector<float> m_block_max;
};

struct StaticBox {
    vec3 center;
    mat3 rot;       // columns = local axes in world space
    vec3 half;
    uint8_t surface = SURF_CONCRETE;
    float max_force = 0; // > 0: yields (bushes, bendable trees): the contact force per node is capped, a car pushes through
    AABB aabb;
    void update_aabb();
};

struct StaticCylinder { // vertical
    vec3 base;
    float radius, height;
    uint8_t surface = SURF_CONCRETE;
};

struct ContactInfo {
    vec3 normal;
    float depth;
    uint8_t surface;
    float max_force = 0; // see StaticBox::max_force
};

class StaticWorld {
public:
    Heightfield terrain;
    bool has_terrain = false;
    std::vector<StaticBox> boxes;
    std::vector<StaticCylinder> cylinders;
    std::shared_ptr<RoadSurface> road; // detailed road over the terrain (optional)
    float kill_y = -200.0f; // nodes below are frozen (fell off the world)

    void add_box(vec3 center, vec3 half, const quat& rot, uint8_t surface);
    void build_grid();
    // Candidate boxes/cylinders overlapping `b` (indices appended).
    void query(const AABB& b, std::vector<int>& boxes_out, std::vector<int>& cyl_out) const;
    // Deepest contact of a sphere (p, r) with the listed candidates + terrain.
    bool collide_point(vec3 p, float r, const int* box_ids, int nbox, const int* cyl_ids, int ncyl, bool test_terrain, ContactInfo& out) const;
    // Ray cast (for picking/camera): returns distance or -1.
    float raycast(vec3 o, vec3 d, float max_t) const;

private:
    float m_grid_cell = 16.0f;
    vec2 m_grid_origin;
    int m_gx = 0, m_gz = 0;
    std::vector<std::vector<int>> m_grid_boxes;
    std::vector<int> m_big_boxes; // boxes spanning many cells
};

} // namespace bl::phys
