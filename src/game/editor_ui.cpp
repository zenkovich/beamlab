// The model editor's panels: the top bar (the model, its file menu, the mode, the views), the left panel (by mode:
// the tools, their options, the beam presets and utilities; the deformation demo's or the physics test's controls),
// the right panel (Properties, Structure, Graphics, Vehicle), the one-line status bar, the preset / template / copy
// windows, the views' titles and the icons (drawn, no image files) (editor.h).
#include "game/editor.h"

#include "core/util.h"
#include "game/editor_internal.h"
#include "game/editor_widgets.h"
#include "game/game.h"
#include "vehicle/vehicle.h"

#include "imgui.h"
#include "imgui_internal.h"

#include <algorithm>
#include <cfloat>
#include <cmath>

namespace bl {

using namespace edit_detail;
using namespace edit_ui;

namespace {

const ImU32 kAccentU = IM_COL32(245, 158, 41, 255);

struct ToolInfo {
    const char* name;
    const char* key;
    ModelEditor::Icon icon;
    const char* hint;
};
using I = ModelEditor::Icon;
const ToolInfo kTools[] = {
    {"Select", "Space", I::Select, "Click a node or an element (Shift adds, Ctrl toggles); drag a box: left to right takes what is inside, right to left what it touches; drag a selected node or a gizmo arrow to move; double click: the connected nodes, a whole face; keep only one kind of what is selected: Only in Properties or Alt+1 - 9 (nodes, beams, shells, cab, shocks, rods, wheels, joints, FEM triangles)"},
    {"Line", "L", I::Line, "Click point after point: a beam each time, a node made where a click lands off a node (on a beam: it is split); click the start to close; type a length + Enter; Esc or a right click ends"},
    {"Node", "N", I::Node, "Click to add a node in any view: the 3D view puts it on the work plane, the flat views at the depth of the selected node (the new node is selected)"},
    {"Rectangle", "R", I::Rect, "Two corners in the view's plane: nodes, edge beams, diagonals and faces (shell faces: no beams); type w,h + Enter"},
    {"Circle", "C", I::Circle, "The centre, then the radius: a ring of nodes with a centre and faces; type the radius + Enter"},
    {"Push / Pull", "P", I::PushPull, "Click a face (triangles), move, click: it is extruded into a braced box with sides; type the distance + Enter"},
    {"Move", "M", I::Move, "Click a base point, click where it goes: axes and nodes snap (arrow keys lock an axis); Ctrl at the first click moves a copy; type a distance or dx,dy,dz + Enter"},
    {"Rotate", "O", I::Rotate, "The centre, a reference direction, the angle (in the view's plane, 15 degree steps with snap); type degrees + Enter"},
    {"Scale", "K", I::Scale, "Click a reference point and move: scaled about the selection's centre (an arrow key: along one axis); type a factor or sx,sy,sz + Enter"},
    {"Tape measure", "U", I::Tape, "Two points: the distance and its components"},
    {"Eraser", "E", I::Erase, "Click or drag over nodes and elements to delete them"},
    {"Triangle", "T", I::Tri, "Three nodes (or points: nodes are made): a cab triangle (collision surface) facing you"},
    {"Shell", "Y", I::Shell, "Three nodes: a shell triangle, an element of the sheet body (it bends, dents and cracks)"},
    {"Shock", "J", I::Shock, "Two nodes: a shock absorber (spring and damper; the last shock's settings)"},
    {"Steering rod", "H", I::Rod, "Two nodes: a rod whose length follows the steering (the other side's twin works the other way)"},
    {"Wheel", "B", I::Wheel, "Two axle nodes, inner then outer: a wheel with the last wheel's settings"},
    {"Merge", "Ctrl+J: the selected", I::Merge, "Click a node, then the node it goes into: the elements on the first go to the second (with symmetry the twins too); Ctrl+J merges the selected nodes into one, the options merge the nodes closer than a distance"},
    {"Joint", "", I::Joint, "Frame elements (FEM beams): click near an end of one to join it to its node with the joint chosen below (welded, ball, hinges, swivel, elastic); Shift+click: its preset's joint again; with symmetry the twin's end too"},
    {"FEM triangle", "Q", I::FemTri, "Three nodes: a FEM triangle, a shell element of the frame (membrane and bending, plastic past the yield, torn past the elongation) of the FEM shell chosen below; it makes its own collision surface. The Rectangle, Circle and Push / Pull tools make them too (Faces: FEM triangles)"},
};
static_assert(sizeof(kTools) / sizeof(kTools[0]) == 19, "one entry per tool");
// one line each: what the tool does (the tooltips have more)
const char* kToolShort[] = {
    "Click or drag a box to select (Shift adds); drag to move",
    "Click point after point: beams, nodes made where needed",
    "Click to place a node, in any view",
    "Two corners: a braced rectangle",
    "The centre, then the radius: a ring of nodes",
    "Click a face, move, click: it is extruded",
    "A base point, then where it goes (Ctrl: a copy)",
    "The centre, a reference, the angle",
    "A reference point, then move",
    "Two points: the distance",
    "Click or drag over what to delete",
    "Three nodes: a cab triangle",
    "Three nodes: a shell triangle",
    "Two nodes: a shock absorber",
    "Two nodes: a steering rod",
    "Two axle nodes, inner then outer: a wheel",
    "A node, then the node it goes into",
    "Click near a frame element's end: its joint",
    "Three nodes: a FEM triangle (a shell element of the frame)",
};

const char* kViewNames[] = {"3D", "Front", "Side", "Top"};
const char* kViewTips[] = {"Perspective (right drag orbits)", "Front: from -x (the model's front), looking back", "Side: from +z (the model's left)", "Top: from above, the front up"};
const char* kBeamTypes[] = {"normal (pulls and pushes)", "rope (pulls only)", "support (pushes only)", "frame element (FEM)"};
// the frame elements' joints (phys::FrameJoint), what each does
const char* kJointNames[] = {"welded (rigid)", "ball joint", "hinge, swings up / down", "hinge, swings sideways", "swivel (turns on its axis)", "elastic (a bushing)"};
const char* kJointTips[] = {
    "Welded: the member keeps its angle to everything at the node (a real frame's joint)",
    "Ball joint: turns freely every way, takes no moment (a truss member, a suspension ball joint, a tie rod end)",
    "Hinge swinging in the member's vertical plane (free about its z axis, across it): a suspension arm on the body",
    "Hinge swinging in the member's horizontal plane (free about its y axis, up)",
    "Swivel: turns freely about the member's own axis, bending held (a bearing, a steering column)",
    "Elastic: every rotation held by a spring of the preset's joint stiffness (a rubber bushing, a bolted joint)"};
const char* kWheelTypes[] = {"wheels (rim + tyre drawn)", "wheels2 (rim and tyre springs)", "meshwheels (a rim mesh)", "meshwheels2", "flexbodywheels (tyre mesh)"};

ImVec2 V(float x, float y) { return ImVec2(x, y); }
void section(const char* s) { section_title(s); }

// euler angles (degrees, the props' X then Y then Z convention: R = Rz Ry Rx) of a rotation
vec3 euler_xyz_deg(const mat3& R) {
    auto e = [&](int row, int c) { return R.c[c][row]; };
    const float b = std::asin(clampf(-e(2, 0), -1, 1));
    const float a = std::atan2(e(2, 1), e(2, 2));
    const float c = std::atan2(e(1, 0), e(0, 0));
    return vec3(a, b, c) * kRad2Deg;
}

void frame_of(vec3 pr, vec3 px, vec3 py, vec3& X, vec3& Y, vec3& N, mat3& base) {
    X = px - pr, Y = py - pr;
    N = normalize_or(cross(Y, X), vec3(0, 1, 0));
    const vec3 rx = normalize_or(X, vec3(1, 0, 0)), ry = cross(rx, N);
    base = mat3(rx, N, ry);
}

} // namespace

// ------------------------------------------------------------------------------------------------ icons
void ModelEditor::draw_icon(ImDrawList* dl, Icon icon, float x, float y, float s, uint32_t col) {
    auto P = [&](float u, float v) { return ImVec2(x + u * s, y + v * s); };
    const float th = std::max(1.2f, s / 13.0f);
    const ImU32 c = col, cf = (col & 0x00ffffffu) | 0x60000000u;
    auto L = [&](float u0, float v0, float u1, float v1, float t = 0) { dl->AddLine(P(u0, v0), P(u1, v1), c, t > 0 ? t : th); };
    auto dot = [&](float u, float v, float r) { dl->AddCircleFilled(P(u, v), r * s, c); };
    auto arrow = [&](float u0, float v0, float u1, float v1) {
        L(u0, v0, u1, v1);
        const float dx = u1 - u0, dy = v1 - v0, l = std::sqrt(dx * dx + dy * dy);
        if (l < 1e-4f) return;
        const float ux = dx / l, uy = dy / l, h = 0.16f;
        dl->AddTriangleFilled(P(u1 + ux * 0.02f, v1 + uy * 0.02f), P(u1 - ux * h - uy * h * 0.6f, v1 - uy * h + ux * h * 0.6f), P(u1 - ux * h + uy * h * 0.6f, v1 - uy * h - ux * h * 0.6f), c);
    };
    auto eye = [&]() {
        dl->PathClear();
        dl->PathLineTo(P(0.08f, 0.5f));
        dl->PathBezierQuadraticCurveTo(P(0.5f, 0.08f), P(0.92f, 0.5f));
        dl->PathBezierQuadraticCurveTo(P(0.5f, 0.92f), P(0.08f, 0.5f));
        dl->PathStroke(c, ImDrawFlags_Closed, th);
        dl->AddCircleFilled(P(0.5f, 0.5f), 0.12f * s, c);
    };
    auto tri = [&](float ax, float ay, float bx, float by, float cx, float cy, bool fill) {
        if (fill) dl->AddTriangleFilled(P(ax, ay), P(bx, by), P(cx, cy), cf);
        dl->AddTriangle(P(ax, ay), P(bx, by), P(cx, cy), c, th);
    };
    switch (icon) {
    case Icon::Select: {
        const ImVec2 pts[] = {P(0.28f, 0.12f), P(0.28f, 0.82f), P(0.44f, 0.66f), P(0.56f, 0.9f), P(0.67f, 0.85f), P(0.55f, 0.61f), P(0.76f, 0.6f)};
        dl->AddConcavePolyFilled(pts, 7, cf);
        dl->AddPolyline(pts, 7, c, ImDrawFlags_Closed, th);
        break;
    }
    case Icon::Line: L(0.2f, 0.8f, 0.8f, 0.2f), dot(0.2f, 0.8f, 0.09f), dot(0.8f, 0.2f, 0.09f); break;
    case Icon::Node: dot(0.5f, 0.5f, 0.13f), dl->AddCircle(P(0.5f, 0.5f), 0.3f * s, c, 0, th); break;
    case Icon::Rect:
        dl->AddRect(P(0.18f, 0.28f), P(0.82f, 0.72f), c, 0, 0, th);
        dot(0.18f, 0.28f, 0.07f), dot(0.82f, 0.72f, 0.07f);
        break;
    case Icon::Circle: dl->AddCircle(P(0.5f, 0.5f), 0.32f * s, c, 0, th), dot(0.5f, 0.5f, 0.06f), L(0.5f, 0.5f, 0.82f, 0.5f, th * 0.7f); break;
    case Icon::PushPull: {
        const ImVec2 base[] = {P(0.12f, 0.82f), P(0.55f, 0.82f), P(0.88f, 0.62f), P(0.45f, 0.62f)};
        dl->AddConvexPolyFilled(base, 4, cf);
        dl->AddPolyline(base, 4, c, ImDrawFlags_Closed, th);
        arrow(0.5f, 0.7f, 0.5f, 0.12f);
        break;
    }
    case Icon::Move:
        arrow(0.5f, 0.5f, 0.5f, 0.1f), arrow(0.5f, 0.5f, 0.5f, 0.9f), arrow(0.5f, 0.5f, 0.1f, 0.5f), arrow(0.5f, 0.5f, 0.9f, 0.5f);
        break;
    case Icon::Rotate:
        dl->PathClear();
        dl->PathArcTo(P(0.5f, 0.52f), 0.32f * s, -kPi * 0.2f, kPi * 1.3f, 20);
        dl->PathStroke(c, 0, th);
        arrow(0.66f, 0.2f, 0.78f, 0.36f);
        dot(0.5f, 0.52f, 0.06f);
        break;
    case Icon::Scale:
        dl->AddRect(P(0.14f, 0.14f), P(0.86f, 0.86f), c, 0, 0, th * 0.8f);
        dl->AddRectFilled(P(0.14f, 0.52f), P(0.48f, 0.86f), cf);
        dl->AddRect(P(0.14f, 0.52f), P(0.48f, 0.86f), c, 0, 0, th);
        arrow(0.46f, 0.54f, 0.8f, 0.2f);
        break;
    case Icon::Tape:
        dl->AddRect(P(0.1f, 0.34f), P(0.9f, 0.66f), c, 0, 0, th);
        for (int k = 1; k < 8; k++) L(0.1f + k * 0.1f, 0.66f, 0.1f + k * 0.1f, k % 2 ? 0.55f : 0.48f, th * 0.7f);
        break;
    case Icon::Erase: {
        const ImVec2 pts[] = {P(0.12f, 0.6f), P(0.5f, 0.22f), P(0.88f, 0.6f), P(0.62f, 0.86f), P(0.38f, 0.86f)};
        dl->AddConvexPolyFilled(pts, 5, cf);
        dl->AddPolyline(pts, 5, c, ImDrawFlags_Closed, th);
        L(0.31f, 0.41f, 0.69f, 0.79f);
        break;
    }
    case Icon::Tri: tri(0.5f, 0.16f, 0.86f, 0.8f, 0.14f, 0.8f, false), dot(0.5f, 0.16f, 0.07f), dot(0.86f, 0.8f, 0.07f), dot(0.14f, 0.8f, 0.07f); break;
    case Icon::Shell:
        dl->AddTriangleFilled(P(0.5f, 0.16f), P(0.86f, 0.8f), P(0.14f, 0.8f), (col & 0x00ffffffu) | 0xa0000000u);
        tri(0.5f, 0.16f, 0.86f, 0.8f, 0.14f, 0.8f, false);
        break;
    case Icon::FemTri: // (a triangle cut into four, its nodes)
        dl->AddTriangleFilled(P(0.5f, 0.14f), P(0.88f, 0.82f), P(0.12f, 0.82f), (col & 0x00ffffffu) | 0x60000000u);
        tri(0.5f, 0.14f, 0.88f, 0.82f, 0.12f, 0.82f, false);
        tri(0.31f, 0.48f, 0.69f, 0.48f, 0.5f, 0.82f, false);
        dot(0.5f, 0.14f, 0.06f), dot(0.88f, 0.82f, 0.06f), dot(0.12f, 0.82f, 0.06f);
        break;
    case Icon::Shock: {
        dot(0.5f, 0.1f, 0.07f), dot(0.5f, 0.9f, 0.07f);
        L(0.5f, 0.1f, 0.5f, 0.25f), L(0.5f, 0.75f, 0.5f, 0.9f);
        const float zz[] = {0.25f, 0.33f, 0.42f, 0.5f, 0.58f, 0.67f, 0.75f};
        for (int k = 0; k + 1 < 7; k++) L(k % 2 ? 0.3f : 0.7f, zz[k], (k + 1) % 2 ? 0.3f : 0.7f, zz[k + 1]);
        break;
    }
    case Icon::Rod: arrow(0.5f, 0.5f, 0.1f, 0.5f), arrow(0.5f, 0.5f, 0.9f, 0.5f), dot(0.5f, 0.5f, 0.07f); break;
    case Icon::Wheel:
        dl->AddCircle(P(0.5f, 0.5f), 0.38f * s, c, 0, th * 1.3f);
        dl->AddCircle(P(0.5f, 0.5f), 0.14f * s, c, 0, th);
        for (int k = 0; k < 5; k++) {
            const float a = k * 2 * kPi / 5;
            L(0.5f + std::cos(a) * 0.14f, 0.5f + std::sin(a) * 0.14f, 0.5f + std::cos(a) * 0.38f, 0.5f + std::sin(a) * 0.38f, th * 0.7f);
        }
        break;
    case Icon::Undo:
    case Icon::Redo: {
        const bool u = icon == Icon::Undo;
        dl->PathClear();
        dl->PathArcTo(P(0.5f, 0.56f), 0.3f * s, u ? kPi * 1.1f : -kPi * 0.1f, u ? kPi * 2.2f : -kPi * 1.2f, 16);
        dl->PathStroke(c, 0, th);
        if (u) arrow(0.26f, 0.52f, 0.2f, 0.28f + 0.2f);
        else arrow(0.74f, 0.52f, 0.8f, 0.48f);
        break;
    }
    case Icon::Save:
        dl->AddRect(P(0.16f, 0.16f), P(0.84f, 0.84f), c, 0, 0, th);
        dl->AddRectFilled(P(0.3f, 0.16f), P(0.66f, 0.4f), cf);
        dl->AddRect(P(0.3f, 0.16f), P(0.66f, 0.4f), c, 0, 0, th * 0.8f);
        dl->AddRectFilled(P(0.28f, 0.58f), P(0.72f, 0.84f), c);
        break;
    case Icon::Drive:
        dl->AddCircle(P(0.5f, 0.5f), 0.38f * s, c, 0, th * 1.3f);
        dot(0.5f, 0.55f, 0.1f);
        L(0.14f, 0.48f, 0.4f, 0.55f), L(0.86f, 0.48f, 0.6f, 0.55f), L(0.5f, 0.6f, 0.5f, 0.88f);
        break;
    case Icon::Physics:
        dl->AddCircle(P(0.32f, 0.68f), 0.2f * s, c, 0, th);
        dot(0.32f, 0.68f, 0.07f);
        arrow(0.42f, 0.58f, 0.86f, 0.14f);
        break;
    case Icon::Close: L(0.2f, 0.2f, 0.8f, 0.8f, th * 1.3f), L(0.8f, 0.2f, 0.2f, 0.8f, th * 1.3f); break;
    case Icon::Eye:
    case Icon::Show: eye(); break;
    case Icon::EyeOff:
    case Icon::Hide: eye(), L(0.15f, 0.85f, 0.85f, 0.15f, th * 1.2f); break;
    case Icon::Lock:
    case Icon::Unlock:
        dl->AddRectFilled(P(0.22f, 0.46f), P(0.78f, 0.86f), c);
        dl->PathClear();
        dl->PathArcTo(P(icon == Icon::Lock ? 0.5f : 0.66f, 0.42f), 0.18f * s, kPi, 2 * kPi, 12);
        dl->PathStroke(c, 0, th * 1.2f);
        if (icon == Icon::Lock) L(0.32f, 0.42f, 0.32f, 0.47f), L(0.68f, 0.42f, 0.68f, 0.47f);
        break;
    case Icon::Plus: L(0.5f, 0.18f, 0.5f, 0.82f, th * 1.4f), L(0.18f, 0.5f, 0.82f, 0.5f, th * 1.4f); break;
    case Icon::Trash:
        dl->AddRect(P(0.26f, 0.3f), P(0.74f, 0.88f), c, 0, 0, th);
        L(0.16f, 0.24f, 0.84f, 0.24f), L(0.4f, 0.14f, 0.6f, 0.14f);
        L(0.42f, 0.42f, 0.42f, 0.76f, th * 0.7f), L(0.58f, 0.42f, 0.58f, 0.76f, th * 0.7f);
        break;
    case Icon::Edit: {
        L(0.22f, 0.78f, 0.72f, 0.28f, th * 2.2f);
        dl->AddTriangleFilled(P(0.14f, 0.86f), P(0.2f, 0.66f), P(0.34f, 0.8f), c);
        L(0.66f, 0.2f, 0.8f, 0.34f, th * 2.2f);
        break;
    }
    case Icon::Copy:
        dl->AddRect(P(0.14f, 0.14f), P(0.62f, 0.62f), c, 0, 0, th);
        dl->AddRectFilled(P(0.38f, 0.38f), P(0.86f, 0.86f), cf);
        dl->AddRect(P(0.38f, 0.38f), P(0.86f, 0.86f), c, 0, 0, th);
        break;
    case Icon::Quad:
        dl->AddRect(P(0.12f, 0.12f), P(0.88f, 0.88f), c, 0, 0, th);
        L(0.5f, 0.12f, 0.5f, 0.88f), L(0.12f, 0.5f, 0.88f, 0.5f);
        break;
    case Icon::Grid:
        for (int k = 0; k < 4; k++) {
            const float t = 0.14f + k * 0.24f;
            L(t, 0.14f, t, 0.86f, th * 0.7f), L(0.14f, t, 0.86f, t, th * 0.7f);
        }
        break;
    case Icon::Floor: {
        const ImVec2 pts[] = {P(0.08f, 0.8f), P(0.62f, 0.8f), P(0.92f, 0.52f), P(0.38f, 0.52f)};
        dl->AddConvexPolyFilled(pts, 4, cf);
        dl->AddPolyline(pts, 4, c, ImDrawFlags_Closed, th);
        dl->AddRectFilled(P(0.42f, 0.2f), P(0.58f, 0.62f), c);
        break;
    }
    case Icon::Symmetry:
    case Icon::Mirror:
        for (int k = 0; k < 5; k++) L(0.5f, 0.08f + k * 0.18f, 0.5f, 0.16f + k * 0.18f);
        tri(0.4f, 0.25f, 0.4f, 0.75f, 0.1f, 0.75f, true);
        tri(0.6f, 0.25f, 0.6f, 0.75f, 0.9f, 0.75f, icon == Icon::Mirror);
        break;
    case Icon::Snap:
        dl->PathClear();
        dl->PathArcTo(P(0.5f, 0.42f), 0.26f * s, kPi, 2 * kPi, 12);
        dl->PathStroke(c, 0, th * 2.2f);
        L(0.24f, 0.42f, 0.24f, 0.8f, th * 2.2f), L(0.76f, 0.42f, 0.76f, 0.8f, th * 2.2f);
        dl->AddRectFilled(P(0.16f, 0.7f), P(0.32f, 0.86f), IM_COL32(255, 90, 80, 255));
        dl->AddRectFilled(P(0.68f, 0.7f), P(0.84f, 0.86f), IM_COL32(120, 160, 255, 255));
        break;
    case Icon::Frame:
        L(0.12f, 0.12f, 0.36f, 0.12f), L(0.12f, 0.12f, 0.12f, 0.36f), L(0.88f, 0.12f, 0.64f, 0.12f), L(0.88f, 0.12f, 0.88f, 0.36f);
        L(0.12f, 0.88f, 0.36f, 0.88f), L(0.12f, 0.88f, 0.12f, 0.64f), L(0.88f, 0.88f, 0.64f, 0.88f), L(0.88f, 0.88f, 0.88f, 0.64f);
        dot(0.5f, 0.5f, 0.1f);
        break;
    case Icon::Chain:
        L(0.12f, 0.78f, 0.36f, 0.3f), L(0.36f, 0.3f, 0.62f, 0.7f), L(0.62f, 0.7f, 0.88f, 0.22f);
        dot(0.12f, 0.78f, 0.07f), dot(0.36f, 0.3f, 0.07f), dot(0.62f, 0.7f, 0.07f), dot(0.88f, 0.22f, 0.07f);
        break;
    case Icon::Pairs: {
        const float p[4][2] = {{0.18f, 0.2f}, {0.82f, 0.2f}, {0.82f, 0.8f}, {0.18f, 0.8f}};
        for (int a = 0; a < 4; a++)
            for (int b = a + 1; b < 4; b++) L(p[a][0], p[a][1], p[b][0], p[b][1], th * 0.8f);
        for (auto& q : p) dot(q[0], q[1], 0.07f);
        break;
    }
    case Icon::Fill: tri(0.5f, 0.14f, 0.88f, 0.84f, 0.12f, 0.84f, true); break;
    case Icon::Connected:
        L(0.2f, 0.3f, 0.5f, 0.55f), L(0.5f, 0.55f, 0.82f, 0.25f), L(0.5f, 0.55f, 0.5f, 0.86f), L(0.2f, 0.3f, 0.2f, 0.7f);
        dot(0.2f, 0.3f, 0.07f), dot(0.5f, 0.55f, 0.08f), dot(0.82f, 0.25f, 0.07f), dot(0.5f, 0.86f, 0.07f), dot(0.2f, 0.7f, 0.07f);
        break;
    case Icon::Grow:
        dot(0.5f, 0.5f, 0.1f);
        arrow(0.5f, 0.36f, 0.5f, 0.08f), arrow(0.5f, 0.64f, 0.5f, 0.92f), arrow(0.36f, 0.5f, 0.08f, 0.5f), arrow(0.64f, 0.5f, 0.92f, 0.5f);
        break;
    case Icon::Invert:
        dl->AddCircle(P(0.5f, 0.5f), 0.34f * s, c, 0, th);
        dl->PathClear();
        dl->PathArcTo(P(0.5f, 0.5f), 0.34f * s, -kPi * 0.5f, kPi * 0.5f, 12);
        dl->PathFillConvex(c);
        break;
    case Icon::Triangulate:
        dl->AddRect(P(0.14f, 0.14f), P(0.86f, 0.86f), c, 0, 0, th);
        L(0.14f, 0.14f, 0.86f, 0.86f), L(0.86f, 0.14f, 0.14f, 0.86f, th * 0.7f), L(0.5f, 0.14f, 0.14f, 0.5f, th * 0.7f);
        break;
    case Icon::Mesh:
        dl->AddRect(P(0.12f, 0.34f), P(0.64f, 0.86f), c, 0, 0, th);
        dl->AddRect(P(0.36f, 0.14f), P(0.88f, 0.66f), c, 0, 0, th * 0.7f);
        L(0.12f, 0.34f, 0.36f, 0.14f, th * 0.7f), L(0.64f, 0.34f, 0.88f, 0.14f, th * 0.7f), L(0.64f, 0.86f, 0.88f, 0.66f, th * 0.7f);
        break;
    case Icon::Link:
        dl->AddEllipse(P(0.36f, 0.5f), ImVec2(0.24f * s, 0.14f * s), c, 0, 0, th * 1.2f);
        dl->AddEllipse(P(0.64f, 0.5f), ImVec2(0.24f * s, 0.14f * s), c, 0, 0, th * 1.2f);
        break;
    case Icon::Ids:
        L(0.38f, 0.14f, 0.3f, 0.86f), L(0.7f, 0.14f, 0.62f, 0.86f), L(0.16f, 0.36f, 0.86f, 0.36f), L(0.14f, 0.64f, 0.84f, 0.64f);
        break;
    case Icon::Blast: // (a burst)
        for (int k = 0; k < 8; k++) {
            const float a = k * kPi / 4 + 0.2f, r0 = 0.14f, r1 = k % 2 ? 0.3f : 0.42f;
            L(0.5f + std::cos(a) * r0, 0.5f + std::sin(a) * r0, 0.5f + std::cos(a) * r1, 0.5f + std::sin(a) * r1, th * 1.2f);
        }
        dot(0.5f, 0.5f, 0.08f);
        break;
    case Icon::Shoot: // (a crosshair)
        dl->AddCircle(P(0.5f, 0.5f), 0.3f * s, c, 0, th);
        L(0.5f, 0.08f, 0.5f, 0.32f), L(0.5f, 0.68f, 0.5f, 0.92f), L(0.08f, 0.5f, 0.32f, 0.5f), L(0.68f, 0.5f, 0.92f, 0.5f);
        dot(0.5f, 0.5f, 0.05f);
        break;
    case Icon::Laser: // (a beam from an emitter, sparks at the end)
        dl->AddRectFilled(P(0.08f, 0.66f), P(0.3f, 0.9f), c);
        L(0.26f, 0.7f, 0.8f, 0.22f, th * 1.6f);
        L(0.8f, 0.22f, 0.93f, 0.12f, th * 0.7f), L(0.8f, 0.22f, 0.92f, 0.3f, th * 0.7f), L(0.8f, 0.22f, 0.74f, 0.08f, th * 0.7f);
        break;
    case Icon::Merge: // (two nodes drawn together into one)
        dl->AddCircle(P(0.18f, 0.28f), 0.1f * s, c, 0, th);
        dl->AddCircle(P(0.18f, 0.72f), 0.1f * s, c, 0, th);
        arrow(0.3f, 0.34f, 0.6f, 0.47f), arrow(0.3f, 0.66f, 0.6f, 0.53f);
        dot(0.8f, 0.5f, 0.13f);
        break;
    case Icon::Joint: // (a beam, a ring round its end at a node)
        L(0.12f, 0.82f, 0.62f, 0.32f, th * 1.6f);
        dl->AddCircle(P(0.72f, 0.24f), 0.17f * s, c, 0, th);
        dot(0.72f, 0.24f, 0.06f);
        break;
    case Icon::Divide: // (a beam with its new nodes)
        L(0.1f, 0.5f, 0.9f, 0.5f, th * 1.4f);
        dot(0.1f, 0.5f, 0.09f), dot(0.37f, 0.5f, 0.09f), dot(0.63f, 0.5f, 0.09f), dot(0.9f, 0.5f, 0.09f);
        break;
    case Icon::Width: // (lines of growing width, a dot)
        L(0.1f, 0.22f, 0.62f, 0.22f, th * 0.6f), L(0.1f, 0.47f, 0.62f, 0.47f, th * 1.4f), L(0.1f, 0.76f, 0.62f, 0.76f, th * 2.6f);
        dot(0.8f, 0.5f, 0.12f);
        break;
    default: break;
    }
}

bool ModelEditor::icon_button(const char* id, Icon icon, bool active, const char* tip, float size, bool enabled) {
    ImGui::BeginDisabled(!enabled);
    if (active) {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(kAccent.x, kAccent.y, kAccent.z, 0.55f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(kAccent.x, kAccent.y, kAccent.z, 0.75f));
    }
    const bool pressed = ImGui::Button(id, ImVec2(size, size));
    if (active) ImGui::PopStyleColor(2);
    const ImVec2 a = ImGui::GetItemRectMin(), b = ImGui::GetItemRectMax();
    const float pad = std::floor((b.x - a.x) * 0.16f);
    const ImU32 c = !enabled ? IM_COL32(120, 124, 132, 255) : active ? IM_COL32(255, 255, 255, 255) : IM_COL32(222, 226, 234, 255);
    draw_icon(ImGui::GetWindowDrawList(), icon, a.x + pad, a.y + pad, b.x - a.x - 2 * pad, c);
    ImGui::EndDisabled();
    if (tip && *tip) ImGui::SetItemTooltip("%s", tip);
    return pressed && enabled;
}

bool ModelEditor::action_row(Icon icon, const char* label, const char* shortcut, bool enabled) {
    // a row of its own: the icon, what it does, its key
    ImGui::BeginDisabled(!enabled);
    const float h = ImGui::GetFrameHeight();
    const bool pressed = ImGui::Selectable((std::string("##") + label).c_str(), false, 0, ImVec2(0, h));
    const ImVec2 a = ImGui::GetItemRectMin(), b = ImGui::GetItemRectMax();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImU32 c = enabled ? IM_COL32(225, 229, 236, 255) : IM_COL32(110, 114, 122, 255);
    draw_icon(dl, icon, a.x + 3, a.y + 2, h - 4, c);
    dl->AddText(ImVec2(a.x + h + 6, a.y + (h - ImGui::GetTextLineHeight()) * 0.5f), c, label);
    if (shortcut && *shortcut) {
        const ImVec2 ts = ImGui::CalcTextSize(shortcut);
        dl->AddText(ImVec2(b.x - ts.x - 4, a.y + (h - ts.y) * 0.5f), IM_COL32(140, 146, 158, 255), shortcut);
    }
    ImGui::EndDisabled();
    return pressed && enabled;
}

// ------------------------------------------------------------------------------------------------ the frame's UI
void ModelEditor::ui(ImFont* small) {
    if (!m_active) return;
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 2.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_PopupRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_GrabRounding, 2.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_ScrollbarRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_TabRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 1.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(6, 5));
    // calmer headers and selections than the game's menus
    ImGui::PushStyleColor(ImGuiCol_Header, ImVec4(0.26f, 0.29f, 0.34f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_HeaderHovered, ImVec4(0.32f, 0.36f, 0.42f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_HeaderActive, ImVec4(0.38f, 0.42f, 0.49f, 1.0f));
    if (m_mode == Mode::Drive) {
        ui_mode_banner();
    } else {
        m_show_binding = false;
        m_gfx_ui_hover = -1;
        ui_bar(small);
        ui_left(small);
        ui_right(small);
        ui_status(small);
        ui_views();
        if (m_preset_window) ui_preset_window();
        if (m_template_window) ui_template_window();
        if (m_shell_window) ui_shell_window();
        if (m_fem_window) ui_fem_window();
        if (m_copy_window) ui_copy_window();
        ui_labels();
    }
    if (m_toast_time > 0 && !m_toast.empty()) {
        const ImGuiViewport* vp = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + vp->WorkSize.x * 0.5f, vp->WorkPos.y + vp->WorkSize.y - m_status_h - 12), ImGuiCond_Always, ImVec2(0.5f, 1));
        ImGui::SetNextWindowBgAlpha(0.9f);
        if (ImGui::Begin("##edtoast", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoInputs))
            ImGui::TextUnformatted(m_toast.c_str());
        ImGui::End();
    }
    ImGui::PopStyleColor(3);
    ImGui::PopStyleVar(9);
}

void ModelEditor::set_mode_ui(int mode) {
    switch (mode) {
    case 0: end_mode(); break;
    case 1: enter_deform(); break;
    case 2: test_physics(); break;
    case 3: test_drive(); break;
    default: break;
    }
}

// ------------------------------------------------------------------------------------------------ the top bar
void ModelEditor::ui_bar(ImFont* small) {
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(vp->WorkPos, ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(vp->WorkSize.x, m_top_h), ImGuiCond_Always);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8, 5));
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollWithMouse;
    const bool open = ImGui::Begin("##editorbar", nullptr, flags);
    ImGui::PopStyleVar();
    if (!open) {
        ImGui::End();
        return;
    }
    const float bs = m_top_h - 10;
    auto gap = [&]() {
        ImGui::SameLine(0, 14);
        ImGui::SeparatorEx(ImGuiSeparatorFlags_Vertical);
        ImGui::SameLine(0, 14);
    };
    // the model: its name, the file menu, undo / redo / save
    ImGui::SetNextItemWidth(190);
    if (ImGui::InputTextWithHint("##title", "the model's name", m_title_buf, sizeof m_title_buf)) {
        m_model.title = trim(m_title_buf);
        m_dirty = true;
    }
    ImGui::SetItemTooltip("The model's name (the file is named after it)%s", m_dirty ? "\nUnsaved changes" : "");
    ImGui::SameLine(0, 4);
    ImGui::TextColored(kAccent, m_dirty ? "*" : " ");
    ImGui::SameLine();
    if (ImGui::Button("File", ImVec2(0, bs))) ImGui::OpenPopup("##filemenu");
    if (ImGui::BeginPopup("##filemenu")) {
        ui_file_menu();
        ImGui::EndPopup();
    }
    ImGui::SameLine();
    if (icon_button("##undo", Icon::Undo, false, "Undo (Ctrl+Z)", bs, !m_undo.empty())) undo();
    ImGui::SameLine();
    if (icon_button("##redo", Icon::Redo, false, "Redo (Ctrl+Shift+Z)", bs, !m_redo.empty())) redo();
    ImGui::SameLine();
    if (icon_button("##save", Icon::Save, false, "Save (Ctrl+S)", bs)) {
        std::string err;
        toast(save(&err) ? "Saved " + m_file : err);
    }
    gap();
    // the mode: editing, the deformation demo, the physics test, the test drive
    {
        static const char* names[] = {"Edit", "Deform", "Physics", "Drive"};
        static const char* tips[] = {"Build and edit the model", "The deformation demo: move nodes or beams, the meshes follow (the model is not changed)",
                                     "The physics test: the model simulated without gravity, drag its nodes (Ctrl+P)", "Save and drive the model (Ctrl+T)"};
        const int cur = m_mode == Mode::Deform ? 1 : m_mode == Mode::Physics ? 2 : 0;
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(1, 0));
        for (int i = 0; i < 4; i++) {
            if (i) ImGui::SameLine();
            const bool on = i == cur;
            if (on) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(kAccent.x, kAccent.y, kAccent.z, 0.6f));
            if (ImGui::Button(names[i], ImVec2(0, bs)) && !on) set_mode_ui(i);
            if (on) ImGui::PopStyleColor();
            ImGui::SetItemTooltip("%s", tips[i]);
        }
        ImGui::PopStyleVar();
    }
    gap();
    // how points are placed
    if (icon_button("##sym", Icon::Symmetry, m_symmetry, "Symmetry (X): edits are mirrored across z = 0 onto the twin nodes", bs)) m_symmetry = !m_symmetry;
    ImGui::SameLine();
    if (icon_button("##snap", Icon::Snap, m_snap, "Snap (G): points to the grid step, angles to 15 degrees (the step: View)", bs)) m_snap = !m_snap;
    gap();
    // the views
    if (icon_button("##quad", Icon::Quad, m_quad, "Four views (V): top, front, side, 3D", bs)) {
        m_quad = !m_quad;
        m_maximized = -1;
    }
    ImGui::SameLine();
    {
        const int cur = m_quad ? m_maximized : m_single;
        ImGui::SetNextItemWidth(92);
        if (ImGui::BeginCombo("##view", cur < 0 ? "All four" : kViewNames[cur])) {
            if (m_quad && ImGui::Selectable("All four", m_maximized < 0)) m_maximized = -1;
            for (int i = 0; i < 4; i++)
                if (ImGui::Selectable(kViewNames[i], cur == i)) {
                    if (m_quad) m_maximized = i;
                    else m_single = i;
                    m_active_view = i;
                }
            ImGui::EndCombo();
        }
        ImGui::SetItemTooltip("The view (keys 1 - 4)");
    }
    ImGui::SameLine();
    if (ImGui::Button("View", ImVec2(0, bs))) ImGui::OpenPopup("##viewpop");
    ImGui::SetItemTooltip("What is drawn and how: the grid, the floor, the lines' width, the graphics, the snap step");
    if (ImGui::BeginPopup("##viewpop")) {
        ui_view_popup();
        ImGui::EndPopup();
    }
    ImGui::SameLine();
    if (icon_button("##gfx", m_show_gfx ? Icon::Eye : Icon::EyeOff, m_show_gfx, "The graphics: the model's meshes drawn over its nodes", bs)) m_show_gfx = !m_show_gfx;
    ImGui::SameLine(ImGui::GetWindowWidth() - bs - 10);
    if (icon_button("##close", Icon::Close, false, "Close the editor (Ctrl+E)", bs)) close();
    ImGui::End();
    (void)small;
}

void ModelEditor::ui_file_menu() {
    if (ImGui::MenuItem("Save", "Ctrl+S")) {
        std::string err;
        toast(save(&err) ? "Saved " + m_file : err);
    }
    ImGui::TextDisabled("  to %s/%s", m_model.home.c_str(), path_filename(save_path()).c_str());
    ImGui::Separator();
    if (ImGui::MenuItem("New from a template...")) m_template_window = true;
    if (ImGui::BeginMenu("Open a model")) {
        const std::string dir = asset_path("vehicles/editor");
        int n = 0;
        if (dir_exists(dir))
            for (const auto& f : list_dir(dir, true, false))
                if (path_ext_lower(f) == ".truck" && f[0] != '.') {
                    n++;
                    if (ImGui::MenuItem(f.c_str(), nullptr, m_file == path_join(dir, f))) load(path_join(dir, f));
                }
        if (!n) ImGui::TextDisabled("none saved yet");
        ImGui::EndMenu();
    }
    if (ImGui::MenuItem("Copy a vehicle (with its graphics)...")) m_copy_window = true;
    ImGui::Separator();
    if (ImGui::MenuItem("Close the editor", "Ctrl+E")) close();
}

void ModelEditor::ui_view_popup() {
    section_title("Show");
    if (ImGui::BeginTable("##showflags", 2, ImGuiTableFlags_SizingStretchSame)) {
        ImGui::TableNextColumn();
        ImGui::Checkbox("Grid", &m_grid);
        ImGui::TableNextColumn();
        ImGui::Checkbox("Floor", &m_floor);
        ImGui::SetItemTooltip("The ground (it collides either way)");
        ImGui::TableNextColumn();
        ImGui::Checkbox("Filled faces", &m_fill);
        ImGui::TableNextColumn();
        ImGui::Checkbox("Node numbers (I)", &m_show_ids);
        ImGui::EndTable();
    }
    section_title("Beams and nodes");
    ImGui::PushItemWidth(220);
    if (props_begin("##lineprops", 0.35f)) {
        prop("Beam width", "[ and ] step it");
        ImGui::SliderFloat("##bw", &m_beam_px, 1.0f, 10.0f, "%.1f px");
        prop("Node size", "Shift+[ and Shift+] step it");
        ImGui::SliderFloat("##ns", &m_node_px, 2.0f, 20.0f, "%.1f px");
        prop("Opacity", "0: only the selected and the hovered are drawn");
        ImGui::SliderFloat("##op", &m_skel_alpha, 0.0f, 1.0f, m_skel_alpha < 0.02f ? "hidden" : "%.2f");
        props_end();
    }
    struct Preset {
        const char* name;
        float beam, node;
    };
    static const Preset presets[] = {{"Thin", 1.0f, 4.0f}, {"Normal", 1.5f, 6.0f}, {"Bold", 3.0f, 9.0f}, {"Heavy", 5.0f, 13.0f}};
    for (int i = 0; i < 4; i++) {
        if (i) ImGui::SameLine();
        if (ImGui::SmallButton(presets[i].name)) m_beam_px = presets[i].beam, m_node_px = presets[i].node;
    }
    bool only = m_gfx_only != 0;
    if (ImGui::Checkbox("Only the graphics' nodes (Alt+G)", &only)) set_gfx_only(only ? 1 : 0);
    section_title("Graphics");
    ImGui::Checkbox("Show the meshes", &m_show_gfx);
    if (props_begin("##gfxviewprops", 0.35f)) {
        prop("Opacity");
        ImGui::SliderFloat("##gop", &m_gfx_alpha, 0.05f, 1.0f, "%.2f");
        props_end();
    }
    section_title("Placing");
    if (props_begin("##placeprops", 0.35f)) {
        prop("Snap step", "G switches the snap");
        ImGui::DragFloat("##step", &m_snap_size, 0.005f, 0.005f, 1.0f, "%.3f m");
        prop("Work height", "The ground plane of the 3D and top views (PageUp / PageDown)");
        ImGui::DragFloat("##wy", &m_work_y, 0.01f, -2.0f, 5.0f, "%.3f m");
        props_end();
    }
    ImGui::PopItemWidth();
}

edit::Model ModelEditor::make_template(int tpl) const {
    switch (tpl) {
    case 0: return edit::make_cart(m_tpl_size.x, m_tpl_size.z, m_tpl_size.y, m_tpl_mass);
    case 1: return edit::make_box(m_tpl_n[0], m_tpl_n[1], m_tpl_n[2], m_tpl_size, m_tpl_mass);
    case 2: return edit::make_plate(m_tpl_n[0], m_tpl_n[1], vec2(m_tpl_size.x, m_tpl_size.y), m_tpl_mass, false);
    case 3: return edit::make_plate(m_tpl_n[0], m_tpl_n[1], vec2(m_tpl_size.x, m_tpl_size.y), m_tpl_mass, true);
    case 4: return edit::make_cylinder(m_tpl_n[0], m_tpl_n[1], m_tpl_size.y * 0.5f, m_tpl_size.x, m_tpl_mass);
    case 6: return edit::make_fem_plate(m_tpl_n[0], m_tpl_n[1], vec2(m_tpl_size.x, m_tpl_size.y), m_tpl_mm * 0.001f);
    case 7: return edit::make_fem_box(std::max(1, m_tpl_n[0]), m_tpl_size, m_tpl_mm * 0.001f);
    default: return edit::make_empty();
    }
}

void ModelEditor::ui_template_window() {
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + vp->WorkSize.x * 0.5f, vp->WorkPos.y + vp->WorkSize.y * 0.35f), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(380, 0), ImGuiCond_Appearing);
    if (!ImGui::Begin("New model###tplwin", &m_template_window, ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::End();
        return;
    }
    static const char* names[] = {"Cart (a box body, 4 wheels, an engine)", "Box", "Plate (cab triangles)", "Sheet (shell triangles)", "Cylinder", "Empty",
                                  "FEM plate (FEM triangles)", "FEM box (hollow, FEM triangles)"};
    if (props_begin("##tplprops", 0.35f)) {
        prop("Template");
        ImGui::Combo("##tpl", &m_tpl, names, 8);
        if (m_tpl == 0 || m_tpl == 1 || m_tpl == 4 || m_tpl == 7) prop("Size (m)"), ImGui::SliderFloat3("##size3", &m_tpl_size.x, 0.1f, 6.0f, "%.2f");
        if (m_tpl == 1) prop("Nodes"), ImGui::SliderInt3("##n3", m_tpl_n, 2, 12);
        if (m_tpl == 7) prop("Segments a face"), ImGui::SliderInt("##nf", m_tpl_n, 1, 12);
        if (m_tpl == 2 || m_tpl == 3 || m_tpl == 6) prop("Nodes"), ImGui::SliderInt2("##n2", m_tpl_n, 2, 24), prop("Size (m)"), ImGui::SliderFloat2("##size2", &m_tpl_size.x, 0.1f, 6.0f, "%.2f");
        if (m_tpl == 4) prop("Segments, rings"), ImGui::SliderInt2("##sr", m_tpl_n, 3, 24);
        if (m_tpl != 5 && m_tpl < 6) prop("Mass"), ImGui::SliderFloat("##mass", &m_tpl_mass, 5.0f, 5000.0f, "%.0f kg", ImGuiSliderFlags_Logarithmic);
        if (m_tpl >= 6) prop("Steel thickness", "The mass follows from the steel's density and the area"), ImGui::SliderFloat("##tmm", &m_tpl_mm, 0.3f, 20.0f, "%.2f mm", ImGuiSliderFlags_Logarithmic);
        props_end();
    }
    hint("The current model is replaced (Undo brings it back).");
    if (ImGui::Button("Create", ImVec2(-1, 0))) {
        const edit::Model m = make_template(m_tpl);
        push_undo();
        const auto undo = m_undo;
        set_model(m, "");
        m_undo = undo; // (the old model can come back)
        m_dirty = true;
        m_template_window = false;
    }
    ImGui::End();
}

void ModelEditor::ui_copy_window() {
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + vp->WorkSize.x * 0.5f, vp->WorkPos.y + vp->WorkSize.y * 0.4f), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(480, 420), ImGuiCond_Appearing);
    if (!ImGui::Begin("Copy a vehicle###copywin", &m_copy_window, ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoCollapse)) {
        ImGui::End();
        return;
    }
    hint("Its nodes, beams, wheels, drivetrain and graphics come into the editor; it is saved next to the vehicle under a name of its own.");
    static char filter[64] = "";
    static int sel = -1;
    ImGui::SetNextItemWidth(-1);
    ImGui::InputTextWithHint("##filter", "filter by name", filter, sizeof filter);
    const auto& reg = vehicle_registry();
    const std::string f = to_lower(filter);
    ImGui::BeginChild("##vehicles", ImVec2(0, -ImGui::GetFrameHeightWithSpacing() - 4), ImGuiChildFlags_Borders);
    bool go = false;
    for (int i = 0; i < (int)reg.size(); i++) {
        const VehicleEntry& e = reg[i];
        if (!f.empty() && to_lower(e.title + " " + e.id).find(f) == std::string::npos) continue;
        ImGui::PushID(i);
        if (ImGui::Selectable(e.title.c_str(), sel == i, ImGuiSelectableFlags_AllowDoubleClick)) {
            sel = i;
            if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) go = true;
        }
        ImGui::SameLine();
        ImGui::TextDisabled("%s", e.folder.c_str());
        ImGui::PopID();
    }
    ImGui::EndChild();
    ImGui::BeginDisabled(sel < 0 || sel >= (int)reg.size());
    go |= ImGui::Button("Copy it into the editor", ImVec2(-1, 0));
    ImGui::EndDisabled();
    if (go && sel >= 0 && sel < (int)reg.size() && import_vehicle_file(reg[sel].file)) {
        m_show_gfx = true;
        m_open_graphics_tab = !m_model.flexbodies.empty() || !m_model.props.empty();
        toast("Copied " + reg[sel].title + " (" + std::to_string(m_model.flexbodies.size()) + " meshes)");
        m_copy_window = false;
    }
    ImGui::End();
}

// ------------------------------------------------------------------------------------------------ the left panel
void ModelEditor::ui_left(ImFont* small) {
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    const float h = vp->WorkSize.y - m_top_h - m_status_h;
    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x, vp->WorkPos.y + m_top_h), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(m_left_w, h), ImGuiCond_Always);
    ImGui::SetNextWindowSizeConstraints(ImVec2(220, h), ImVec2(std::max(220.0f, vp->WorkSize.x * 0.4f), h));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10, 10));
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoFocusOnAppearing;
    const bool open = ImGui::Begin("##editorleft", nullptr, flags);
    ImGui::PopStyleVar();
    if (!open) {
        ImGui::End();
        return;
    }
    m_left_w = ImGui::GetWindowWidth();
    ImGui::PushFont(small, 0.0f);
    if (m_mode == Mode::Deform) {
        ImGui::TextColored(kAccent, "DEFORMATION DEMO");
        ui_deform_panel();
    } else if (m_mode == Mode::Physics) {
        ImGui::TextColored(kAccent, "PHYSICS TEST");
        ui_physics_panel();
    } else {
        ui_tools();
        ui_tool_options();
        if (ImGui::CollapsingHeader("Beam presets", ImGuiTreeNodeFlags_DefaultOpen)) ui_presets();
        if (ImGui::CollapsingHeader("Shell materials", ImGuiTreeNodeFlags_DefaultOpen)) ui_shell_presets();
        if (ImGui::CollapsingHeader("FEM shells", m_model.fem_count() ? ImGuiTreeNodeFlags_DefaultOpen : 0)) ui_fem_presets();
        if (ImGui::CollapsingHeader("Utilities", ImGuiTreeNodeFlags_DefaultOpen)) ui_utilities();
    }
    ImGui::PopFont();
    ImGui::End();
}

void ModelEditor::ui_tools() {
    // the tools in three groups, the active one's name and what it does
    struct Group {
        const char* name;
        std::vector<Tool> tools;
    };
    static const Group groups[] = {
        {"Select and change", {Tool::Select, Tool::Move, Tool::Rotate, Tool::Scale, Tool::Merge, Tool::Erase, Tool::Tape}},
        {"Draw", {Tool::Line, Tool::Node, Tool::Rect, Tool::Circle, Tool::PushPull}},
        {"Add", {Tool::Tri, Tool::Shell, Tool::FemTri, Tool::Shock, Tool::Rod, Tool::Wheel, Tool::Joint}},
    };
    const float bs = 32.0f;
    for (const Group& g : groups) {
        ImGui::PushStyleColor(ImGuiCol_Text, kDimText);
        ImGui::TextUnformatted(g.name);
        ImGui::PopStyleColor();
        bool first = true;
        for (Tool t : g.tools) {
            if (!first) ImGui::SameLine(0, 4);
            first = false;
            const int i = (int)t;
            char id[16], tip[400];
            snprintf(id, sizeof id, "##tool%d", i);
            snprintf(tip, sizeof tip, "%s  (%s)\n%s", kTools[i].name, kTools[i].key, kTools[i].hint);
            if (icon_button(id, kTools[i].icon, (int)m_tool == i, tip, bs)) set_tool(t);
        }
    }
    ImGui::Dummy(ImVec2(0, 2));
    ImGui::TextColored(kAccent, "%s", kTools[(int)m_tool].name);
    ImGui::SameLine();
    ImGui::TextDisabled("%s", kTools[(int)m_tool].key);
    hint(kToolShort[(int)m_tool]);
}

void ModelEditor::ui_tool_options() {
    // only the tools that have options show them
    const bool any = m_tool == Tool::Line || m_tool == Tool::Node || m_tool == Tool::Rect || m_tool == Tool::Circle || m_tool == Tool::Move || m_tool == Tool::Rotate ||
                     m_tool == Tool::Scale || m_tool == Tool::Shell || m_tool == Tool::PushPull || m_tool == Tool::Merge || m_tool == Tool::Joint ||
                     m_tool == Tool::FemTri;
    auto shell_mat = [&]() {
        std::vector<std::string> names;
        for (int i = 0; i < m_model.shell_preset_count(); i++) names.push_back(m_model.shell_preset(i).name + " (" + m_model.shell_preset(i).material + ")");
        prop("Material", "What the new shell triangles are made of (the Shell materials list)");
        if (ImGui::BeginCombo("##shellmat", names[std::clamp(m_shell_preset, 0, (int)names.size() - 1)].c_str())) {
            for (int i = 0; i < (int)names.size(); i++)
                if (ImGui::Selectable(names[i].c_str(), i == m_shell_preset)) m_shell_preset = i;
            ImGui::EndCombo();
        }
    };
    auto fem_mat = [&]() {
        std::vector<std::string> names;
        const int n = std::max(1, (int)m_model.fem_presets.size());
        for (int i = 0; i < n; i++) names.push_back(m_model.fem_preset(i).name);
        prop("FEM shell", "What the new FEM triangles are made of: a material and a thickness (the FEM shells list)");
        if (ImGui::BeginCombo("##femmat", names[std::clamp(m_fem_preset, 0, n - 1)].c_str())) {
            for (int i = 0; i < n; i++)
                if (ImGui::Selectable(names[i].c_str(), i == m_fem_preset)) m_fem_preset = i;
            ImGui::EndCombo();
        }
    };
    if (!any) return;
    section_title("Options");
    auto faces = [&]() {
        prop("Faces", "Cab: a collision surface; shell: elements of the sheet body (they bend, dent and crack); FEM: shell elements of the frame "
                      "(solved with the FEM beams: membrane and bending, plasticity, tearing)");
        static const char* f[] = {"none", "cab triangles", "shell triangles", "FEM triangles"};
        ImGui::Combo("##faces", &m_face_mode, f, 4);
        if (m_face_mode >= 2) {
            if (m_face_mode == 2) shell_mat();
            else fem_mat();
            prop("Beams too", "Shells hold their shape themselves (stretching and bending): no beams are made along them unless this is on");
            ImGui::Checkbox("##shellbeams", &m_shell_beams);
        }
    };
    auto plane = [&]() {
        if (m_views[m_active_view].ortho) return;
        prop("3D plane", "The plane the 3D view draws in");
        static const char* p[] = {"ground", "side (x y)", "front (y z)"};
        ImGui::Combo("##plane", &m_plane_mode, p, 3);
    };
    if (!props_begin("##toolopts")) return;
    switch (m_tool) {
    case Tool::Line: {
        std::vector<const char*> names;
        for (auto& g : m_model.groups) names.push_back(g.name.c_str());
        prop("Beam preset");
        ImGui::Combo("##preset", &m_group, names.data(), (int)names.size());
        break;
    }
    case Tool::Node:
        prop("Work height", "The plane the 3D and top views put nodes on (PageUp / PageDown); the front and side views use the selected node's depth");
        ImGui::DragFloat("##wy", &m_work_y, 0.01f, -2.0f, 5.0f, "%.3f m");
        break;
    case Tool::Rect:
        prop("Divisions");
        ImGui::SliderInt2("##div", m_rect_div, 1, 16);
        faces();
        if (m_face_mode < 2 || m_shell_beams) {
            prop("Diagonal beams");
            ImGui::Checkbox("##diag", &m_rect_diagonals);
        }
        plane();
        break;
    case Tool::Circle:
        prop("Sides");
        ImGui::SliderInt("##sides", &m_circle_sides, 3, 48);
        prop("Centre node");
        ImGui::Checkbox("##centre", &m_circle_center);
        faces();
        plane();
        break;
    case Tool::Shell: shell_mat(); break;
    case Tool::FemTri: fem_mat(); break;
    case Tool::Joint: {
        prop("Joint", "What a click near a frame element's end makes of its joint. Its axes: x along it, y up in its vertical plane, z across");
        ImGui::TextDisabled("%s", kJointNames[std::clamp(m_joint_type, 0, 5)]);
        props_end();
        ui_joint_palette(m_joint_type);
        props_begin("##toolopts2");
        break;
    }
    case Tool::Merge: {
        prop("Selected", "The selected nodes into one, at their centre (a fixed one stays where it is); with symmetry their twins too");
        if (ImGui::Button("Merge them (Ctrl+J)", ImVec2(-FLT_MIN, 0))) merge_selection();
        prop("Closer than", "Nodes nearer to one another than this merge (the selection's, or all of them)");
        float mm = m_merge_tol * 1000.0f;
        if (ImGui::SliderFloat("##tol", &mm, 0.1f, 200.0f, "%.1f mm", ImGuiSliderFlags_Logarithmic)) m_merge_tol = mm * 0.001f;
        prop("");
        if (ImGui::Button("Merge close ones", ImVec2(-FLT_MIN, 0))) merge_by_distance(m_merge_tol);
        break;
    }
    case Tool::PushPull:
        prop("Shell beams", "A shell face is extruded into a box of shells, which holds its shape itself: beams along it only if this is on (a cab face always gets them)");
        ImGui::Checkbox("##ppbeams", &m_shell_beams);
        break;
    case Tool::Move:
    case Tool::Rotate:
    case Tool::Scale:
        plane();
        prop("Axis lock", "The arrow keys: right x, up y, left z, down frees");
        ImGui::TextUnformatted(m_axis_lock < 0 ? "none" : m_axis_lock == 0 ? "x (red)" : m_axis_lock == 1 ? "y (green)" : "z (blue)");
        break;
    default: break;
    }
    props_end();
}

void ModelEditor::ui_presets() {
    edit::Model& M = m_model;
    const float row = ImGui::GetFrameHeight();
    const float list_h = std::min(7.5f, (float)M.groups.size() + 0.3f) * (row + ImGui::GetStyle().ItemSpacing.y) + 6;
    ImGui::BeginChild("##presets", ImVec2(0, list_h), ImGuiChildFlags_Borders);
    for (int i = 0; i < (int)M.groups.size(); i++) {
        edit::BeamGroup& g = M.groups[i];
        ImGui::PushID(i);
        ImGui::ColorButton("##c", ImVec4(g.color.x, g.color.y, g.color.z, 1), ImGuiColorEditFlags_NoTooltip, ImVec2(10, row));
        ImGui::SameLine(0, 6);
        std::string label = g.name;
        if (g.is_frame()) label += "  FEM";
        else if (g.hold_rotation) label += "  held";
        if (g.type == edit::BEAM_ROPE) label += "  rope";
        if (g.type == edit::BEAM_SUPPORT) label += "  support";
        if (ImGui::Selectable(label.c_str(), m_group == i, ImGuiSelectableFlags_AllowDoubleClick, ImVec2(ImGui::GetContentRegionAvail().x - row - 4, row))) {
            m_group = i;
            if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) m_preset_window = true, m_preset_edit = i, m_preset_new = false;
        }
        char tip[200];
        if (g.is_frame())
            snprintf(tip, sizeof tip, "Frame element: %s %s %.0f mm, wall %.3g mm; joints: %s / %s", g.frame_material.c_str(),
                     phys::frame_shape_name((phys::FrameShape)g.frame_shape), g.frame_outer * 1000.0f, g.frame_wall * 1000.0f,
                     phys::frame_joint_name((phys::FrameJoint)g.frame_end_a), phys::frame_joint_name((phys::FrameJoint)g.frame_end_b));
        else
            snprintf(tip, sizeof tip, "k %.3g N/m, d %.3g, deform %s, break %s%s", g.spring, g.damp, g.deform >= 1e29f ? "never" : format("%.3g N", g.deform).c_str(),
                     g.brk >= 1e29f ? "never" : format("%.3g N", g.brk).c_str(), g.hold_rotation ? ", ends hold their angle" : "");
        ImGui::SetItemTooltip("%s\nNew beams take the checked preset; double click: edit", tip);
        ImGui::SameLine();
        if (icon_button("##sel", Icon::Select, false, "Select every beam of this preset", row)) select_by_preset(i);
        ImGui::PopID();
    }
    ImGui::EndChild();
    const float bs = ImGui::GetFrameHeight();
    if (icon_button("##pnew", Icon::Plus, false, "New preset", bs)) {
        m_preset_draft = m_group >= 0 && m_group < (int)M.groups.size() ? M.groups[m_group] : edit::BeamGroup();
        m_preset_draft.name = "Preset " + std::to_string(M.groups.size() + 1);
        m_preset_window = true, m_preset_new = true, m_preset_edit = -1;
    }
    ImGui::SameLine();
    if (icon_button("##pedit", Icon::Edit, m_preset_window && !m_preset_new, "Edit the checked preset", bs)) m_preset_window = true, m_preset_new = false, m_preset_edit = m_group;
    ImGui::SameLine();
    if (icon_button("##pcopy", Icon::Copy, false, "Duplicate the checked preset", bs)) {
        push_undo();
        edit::BeamGroup g = M.groups[m_group];
        g.name += " copy";
        M.groups.push_back(g);
        m_group = (int)M.groups.size() - 1;
    }
    ImGui::SameLine();
    if (icon_button("##pdel", Icon::Trash, false, "Delete the checked preset (its beams take the first)", bs, M.groups.size() > 1)) {
        push_undo();
        const int g = m_group;
        for (edit::Beam& b : M.beams) b.group = b.group == g ? 0 : b.group > g ? b.group - 1 : b.group;
        M.groups.erase(M.groups.begin() + g);
        m_group = std::clamp(g - 1, 0, (int)M.groups.size() - 1);
        m_preset_window = false;
    }
    ImGui::SameLine();
    if (ImGui::Button("Apply to selection", ImVec2(-1, bs))) apply_preset_to_selection(m_group);
    ImGui::SetItemTooltip("The selected beams, and every beam between selected nodes, take the checked preset");
}

void ModelEditor::ui_preset_window() {
    edit::Model& M = m_model;
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + m_left_w + 12, vp->WorkPos.y + m_top_h + 12), ImGuiCond_Appearing);
    ImGui::SetNextWindowSize(ImVec2(400, 0), ImGuiCond_Appearing);
    if (!m_preset_new && (m_preset_edit < 0 || m_preset_edit >= (int)M.groups.size())) {
        m_preset_window = false;
        return;
    }
    edit::BeamGroup& g = m_preset_new ? m_preset_draft : M.groups[m_preset_edit];
    char title[96];
    snprintf(title, sizeof title, "%s###presetwin", m_preset_new ? "New beam preset" : ("Beam preset: " + g.name).c_str());
    if (!ImGui::Begin(title, &m_preset_window, ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::End();
        return;
    }
    bool ch = false;
    if (props_begin("##presetprops", 0.34f)) {
        prop("Name");
        char name[64];
        snprintf(name, sizeof name, "%s", g.name.c_str());
        if (ImGui::InputText("##name", name, sizeof name)) g.name = name, ch = true;
        prop("Colour");
        float c[3] = {g.color.x, g.color.y, g.color.z};
        if (ImGui::ColorEdit3("##colour", c, ImGuiColorEditFlags_NoInputs)) g.color = vec4(c[0], c[1], c[2], 1), ch = true;
        prop("Type", "Normal: pulls and pushes; rope: only pulls (slack when shorter); support: only pushes. Frame element: a real tube or bar "
                     "(FEM: it stretches, twists and bends like its section and material, solved implicitly, so as stiff as steel is)");
        const int type_before = g.type;
        ch |= ImGui::Combo("##type", &g.type, kBeamTypes, 4);
        (void)type_before;
        if (g.is_frame()) {
            ch |= ui_frame_section(g);
            props_end();
        } else {
        prop("Ends", "Free: the beam turns about its nodes (a truss needs triangles). Held: each end keeps its angle to the structure, the beam bends instead of pivoting (written as a pair of joints)");
        static const char* ends[] = {"free (a pin)", "held (a rigid joint)"};
        int e = g.hold_rotation ? 1 : 0;
        if (ImGui::Combo("##ends", &e, ends, 2)) g.hold_rotation = e == 1, ch = true;
        if (g.hold_rotation) prop("Hold stiffness"), ch |= ImGui::SliderFloat("##holdk", &g.joint_k, 1000.0f, 1.0e6f, "%.3g N/m", ImGuiSliderFlags_Logarithmic);
        prop("Spring", "Stiffness along the beam; stiff beams need heavy nodes (k dt^2 / m)");
        ch |= ImGui::SliderFloat("##spring", &g.spring, 1.0e3f, 3.0e7f, "%.3g N/m", ImGuiSliderFlags_Logarithmic);
        prop("Damping");
        ch |= ImGui::SliderFloat("##damp", &g.damp, 1.0f, 1.0e5f, "%.3g N s/m", ImGuiSliderFlags_Logarithmic);
        prop("Deforms at", "Above this force the beam takes a set (bends for good); the box: never");
        ch |= never_field("deform", &g.deform, 1.0e3f, 1.0e7f, "%.3g N");
        prop("Breaks at", "Above this force the beam breaks; the box: never");
        ch |= never_field("break", &g.brk, 1.0e3f, 1.0e8f, "%.3g N");
        prop("Plastic", "How much of a deformation stays (0: RoR's default behaviour)");
        ch |= ImGui::SliderFloat("##plastic", &g.plastic, 0.0f, 1.0f, "%.2f");
        prop("Invisible", "Not drawn in the game");
        ch |= ImGui::Checkbox("##invisible", &g.invisible);
        props_end();
        }
    }
    if (g.is_frame()) {
        ImGui::TextColored(kDimText, "Solved implicitly: any stiffness is stable");
    } else {
        const float dt = 0.0005f, m = std::max(1.0f, M.minimass), s = g.spring * dt * dt / m;
        ImGui::TextColored(s > 0.1f ? ImVec4(1, 0.65f, 0.3f, 1) : kDimText, "Stability: k dt^2 / m = %.3f on a %.0f kg node", s, m);
        ImGui::SetItemTooltip("Summed over a node's beams it must stay below ~0.45");
    }
    ImGui::Separator();
    if (m_preset_new) {
        if (ImGui::Button("Create")) {
            push_undo();
            M.groups.push_back(g);
            m_group = (int)M.groups.size() - 1;
            m_preset_new = false;
            m_preset_edit = m_group;
        }
        ImGui::SameLine();
        ImGui::BeginDisabled(selection_empty());
        if (ImGui::Button("Create and apply")) {
            push_undo();
            M.groups.push_back(g);
            m_group = (int)M.groups.size() - 1;
            m_preset_new = false;
            m_preset_edit = m_group;
            apply_preset_to_selection(m_group);
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::Button("Cancel")) m_preset_window = false;
    } else {
        if (ch) m_dirty = true;
        ImGui::BeginDisabled(selection_empty());
        if (ImGui::Button("Apply to selection")) apply_preset_to_selection(m_preset_edit);
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::Button("Select its beams")) select_by_preset(m_preset_edit);
        ImGui::SameLine();
        if (ImGui::Button("Close")) m_preset_window = false;
    }
    ImGui::End();
}

void ModelEditor::ui_utilities() {
    const bool n2 = m_sel.size() >= 2, n1 = !m_sel.empty(), any = !selection_empty();
    const bool n34 = m_sel.size() == 3 || m_sel.size() == 4, n3 = m_sel.size() >= 3, n4 = m_sel.size() >= 4;
    auto group = [](const char* s) {
        ImGui::PushStyleColor(ImGuiCol_Text, kDimText);
        ImGui::TextUnformatted(s);
        ImGui::PopStyleColor();
    };
    group("Connect");
    if (action_row(Icon::Chain, "Beams along the selection", "Ctrl+B", n2)) connect_selection(false);
    if (action_row(Icon::Pairs, "Beams between all pairs", "Ctrl+Shift+B", n2)) connect_selection(true);
    if (action_row(Icon::Fill, "Face over 3 - 4 nodes", "Ctrl+F", n34)) fill_selection();
    if (action_row(Icon::Triangulate, "Cover with beams", "", n3)) triangulate_selection(0);
    ImGui::SetItemTooltip("A Delaunay cover of the selected nodes (on their flattest plane) laid with beams");
    if (action_row(Icon::Tri, "Cover with cab triangles", "", n3)) triangulate_selection(1);
    if (action_row(Icon::Shell, "Cover with shells", "", n3)) triangulate_selection(2);
    if (action_row(Icon::FemTri, "Cover with FEM triangles", "", n3)) triangulate_selection(3);
    ImGui::SetItemTooltip("The cover laid with FEM triangles of the checked FEM shell (a sheet of the frame: no beams needed)");
    if (action_row(Icon::Tri, "Collision hull round the selection", "", n4)) hull_selection();
    ImGui::SetItemTooltip("The convex hull of the selected nodes as hull triangles: a coarse solid collision shell on a frame's\n"
                          "nodes (not drawn in the game, one-sided, facing out). Select the frame's outer corners and run it.");
    group("Copy");
    if (action_row(Icon::Mirror, "Mirror to the other side", "Ctrl+M", n1)) mirror_selection();
    if (action_row(Icon::Copy, "Duplicate and move", "Ctrl+D", n1)) {
        duplicate_selection(vec3(0));
        set_tool(Tool::Move);
        m_op_pushed = true;
        op_begin_nodes();
        m_op_a = gizmo_center();
        m_op = true;
        m_op_view = m_active_view;
    }
    if (action_row(Icon::Trash, "Delete", "Del", any)) delete_selection();
    group("Merge");
    if (action_row(Icon::Merge, "Merge the selected into one", "Ctrl+J", n2)) merge_selection();
    ImGui::SetItemTooltip("At their centre (a fixed one stays where it is); the elements on them go to it, the ones that fold up go");
    {
        char label[64];
        snprintf(label, sizeof label, "Merge nodes closer than %.1f mm", m_merge_tol * 1000.0f);
        if (action_row(Icon::Merge, label, "", !m_model.nodes.empty())) merge_by_distance(m_merge_tol);
        ImGui::SetItemTooltip("The selected nodes (or all of them) that nearly coincide: a weld after drawing overlapping shapes (the distance: the Merge tool's options)");
    }
    group("Divide");
    {
        char label[64];
        snprintf(label, sizeof label, "Divide the selected beams into %d", m_divide_n);
        if (action_row(Icon::Divide, label, "", !m_sel_beams.empty())) divide_selected_beams(m_divide_n);
        ImGui::SetItemTooltip("Each selected beam (and its twin) cut into equal beams: a frame element gets nodes to bend, dent and be hit between its joints (its joints stay at its ends)");
        ImGui::SetNextItemWidth(-FLT_MIN);
        ImGui::SliderInt("##divn", &m_divide_n, 2, 8, "into %d");
    }
    group("Select and show");
    if (action_row(Icon::Connected, "Everything connected", "Ctrl+L", any)) select_connected();
    if (action_row(Icon::Grow, "Grow by one beam", "Ctrl+G", n1)) select_grow();
    if (action_row(Icon::Invert, "Invert", "Ctrl+I")) select_invert();
    if (action_row(Icon::Hide, "Hide the selection", "Ctrl+H", n1)) hide_selection();
    if (action_row(Icon::Show, "Show everything", "Ctrl+Shift+H")) unhide_all();
    if (action_row(Icon::Frame, "Frame the selection", "F")) frame_selection();
    group("Place");
    if (action_row(Icon::Snap, "Snap to the grid", "", n1)) {
        push_undo();
        for (int n : selection_with_twins()) m_model.nodes[n].p = snap(m_model.nodes[n].p);
    }
    if (action_row(Icon::Mesh, "Onto the reference surface", "", n1 && !m_ref_idx.empty())) snap_to_surface();
}

void ModelEditor::ui_physics_panel() {
    hint("The model simulated where it stands. Use the tools in any view.");
    ImGui::Spacing();
    // the tools (1 - 4)
    struct T {
        Icon icon;
        const char* name;
        const char* tip;
    };
    static const T tools[4] = {
        {Icon::Physics, "Grab", "1: drag a node with the left button (the wheel while pulling: stronger / weaker)"},
        {Icon::Blast, "Destroy", "2: hold the button and sweep: what is under the cursor breaks"},
        {Icon::Shoot, "Shoot", "3: projectiles (hold the button to keep firing)"},
        {Icon::Laser, "Laser", "4: hold the button and sweep: a cut along the path"},
    };
    const float bs = 34.0f;
    for (int i = 0; i < 4; i++) {
        if (i) ImGui::SameLine(0, 4);
        char id[16];
        snprintf(id, sizeof id, "##ptool%d", i);
        if (icon_button(id, tools[i].icon, m_phys_tool == i, tools[i].tip, bs)) m_phys_tool = i;
    }
    ImGui::SameLine();
    ImGui::AlignTextToFramePadding();
    ImGui::TextColored(kAccent, "%s", tools[m_phys_tool].name);
    if (props_begin("##ptoolprops")) {
        switch (m_phys_tool) {
        case 0: {
            prop("Strength", "How hard the grab pulls the node toward the mouse (1x: from the body's mass; weaker: it lags and sags, stronger: it follows at once). It changes at once, also while pulling");
            ImGui::SliderFloat("##gstr", &m_game.grab_strength, 0.05f, 20.0f, "%.2gx", ImGuiSliderFlags_Logarithmic);
            prop("");
            const float presets[] = {0.2f, 1.0f, 5.0f};
            const char* names[] = {"0.2x", "1x", "5x"};
            const char* tips[] = {"Gentle: the node lags and sags", "Normal", "Strong: the node follows the mouse at once"};
            const float sp = ImGui::GetStyle().ItemSpacing.x * 0.5f;
            const float bw = (ImGui::GetContentRegionAvail().x - 2 * sp) / 3;
            for (int i = 0; i < 3; i++) {
                if (i) ImGui::SameLine(0, sp);
                const bool on = std::fabs(m_game.grab_strength - presets[i]) < 1e-3f;
                if (on) ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
                if (ImGui::Button(names[i], ImVec2(bw, 0))) m_game.grab_strength = presets[i];
                if (on) ImGui::PopStyleColor();
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", tips[i]);
            }
            if (m_game.grab_active && m_game.grab_body) {
                prop("Pull");
                ImGui::Text("%.3g N/m", m_game.grab_body->grab_k * m_game.grab_strength);
            }
            break;
        }
        case 1:
            prop("Radius");
            ImGui::SliderFloat("##rad", &m_game.destroy_radius, 0.05f, 3.0f, "%.2f m");
            prop("Blast", "The broken pieces are thrown apart");
            ImGui::Checkbox("##blast", &m_game.destroy_blast);
            prop("Broken");
            ImGui::Text("%d", m_game.destroyed_total);
            break;
        case 2:
            prop("Projectile");
            if (ImGui::BeginCombo("##proj", Game::projectile_name(m_game.projectile_kind))) {
                for (int i = 0; i < Game::kProjectileKinds; i++)
                    if (ImGui::Selectable(Game::projectile_name(i), i == m_game.projectile_kind)) m_game.projectile_kind = i;
                ImGui::EndCombo();
            }
            prop("Speed");
            ImGui::SliderFloat("##spd", &m_game.projectile_speed, 5, 150, "%.0f m/s");
            prop("Rate");
            ImGui::SliderFloat("##rate", &m_game.fire_rate, 1, 20, "%.0f shots/s");
            prop("");
            if (ImGui::SmallButton("Clear the shots")) m_game.clear_projectiles();
            break;
        case 3:
            prop("Range");
            ImGui::SliderFloat("##range", &m_game.laser_range, 5, 300, "%.0f m");
            prop("Cut");
            ImGui::Text("%d links", m_game.laser_total);
            break;
        default: break;
        }
        props_end();
    }
    section_title("Test");
    if (props_begin("##physprops")) {
        prop("Gravity", "G");
        if (ImGui::Checkbox("##gravity", &m_phys_gravity)) m_game.world.settings.gravity = m_phys_gravity ? m_saved_gravity : vec3(0);
        prop("Speed", "The simulation's speed: slow motion down to 0.01x (- and =, the keypad's - and +; Backspace: real time)");
        {
            int ti = time_scale_index(m_phys_time);
            char fmt[24];
            snprintf(fmt, sizeof fmt, "%gx", kTimeScales[ti]);
            if (ImGui::SliderInt("##speed", &ti, 0, kTimeScaleCount - 1, fmt)) set_phys_time(kTimeScales[ti]);
        }
        prop("Running", "Space; paused, Step (N) moves it on by 10 ms of simulated time");
        bool run = !m_game.paused;
        if (ImGui::Checkbox("##running", &run)) m_game.paused = !run;
        ImGui::SameLine();
        ImGui::BeginDisabled(!m_game.paused);
        if (ImGui::SmallButton("Step")) m_game.step_once = true;
        ImGui::EndDisabled();
        prop("Beam width");
        ImGui::SliderFloat("##bw", &m_beam_px, 1.0f, 10.0f, "%.1f px");
        prop("Meshes");
        ImGui::SliderFloat("##ga", &m_gfx_alpha, 0.05f, 1.0f, "%.2f");
        props_end();
    }
    if (m_test && alive(m_test)) {
        const phys::FemFrame& fem = m_test->body->fem;
        if (!fem.empty()) {
            // the frame: its most loaded member against its yield (or buckling) limit, the ones bent for good or broken
            float peak = 0;
            int bent = 0;
            for (const phys::FrameElement& e : fem.elems)
                if (!e.broken) peak = std::max(peak, e.util), bent += e.damage > 1e-4f;
            int dented = 0;
            for (const phys::FrameTri& t : fem.tris)
                if (!t.broken) peak = std::max(peak, t.util), dented += t.dmg > 0;
            if (fem.tris.empty()) ImGui::TextDisabled("Frame: %zu members, peak load %.0f%% of yield", fem.elems.size(), peak * 100.0f);
            else if (fem.elems.empty()) ImGui::TextDisabled("Frame: %zu triangles, peak %.0f%% of yield", fem.tris.size(), peak * 100.0f);
            else ImGui::TextDisabled("Frame: %zu members, %zu tris, peak %.0f%%", fem.elems.size(), fem.tris.size(), peak * 100.0f);
            ImGui::SetItemTooltip("The most loaded frame member or triangle: its force, moment or torque (a triangle's stress) against the limit where it "
                                  "yields (or buckles). Show the loads: the game's debug view, colour by stress");
            if (bent || fem.broken) ImGui::TextColored(ImVec4(1, 0.65f, 0.3f, 1), "%d bent for good, %d broken", bent, fem.broken);
            if (dented || fem.tris_torn) ImGui::TextColored(ImVec4(1, 0.65f, 0.3f, 1), "%d triangles dented for good, %d torn", dented, fem.tris_torn);
        }
        ImGui::TextDisabled("%d broken beams", m_test->broken_beams() - fem.broken);
    }
    ImGui::Spacing();
    if (ImGui::Button("Restart (R)", ImVec2(-1, 0))) test_physics();
    if (ImGui::Button("Back to editing (Esc)", ImVec2(-1, 0))) end_mode();
}

// ------------------------------------------------------------------------------------------------ the right panel
void ModelEditor::ui_right(ImFont* small) {
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    const float h = vp->WorkSize.y - m_top_h - m_status_h;
    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + vp->WorkSize.x - m_right_w, vp->WorkPos.y + m_top_h), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(m_right_w, h), ImGuiCond_Always);
    ImGui::SetNextWindowSizeConstraints(ImVec2(260, h), ImVec2(std::max(260.0f, vp->WorkSize.x * 0.45f), h));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10, 8));
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoFocusOnAppearing;
    const bool open = ImGui::Begin("##editorright", nullptr, flags);
    ImGui::PopStyleVar();
    if (!open) {
        ImGui::End();
        return;
    }
    m_right_w = ImGui::GetWindowWidth();
    ImGui::PushFont(small, 0.0f);
    if (ImGui::BeginTabBar("edtabs")) {
        auto tab = [&](const char* name, void (ModelEditor::*fn)(), ImGuiTabItemFlags f = 0) {
            if (ImGui::BeginTabItem(name, nullptr, f)) {
                ImGui::BeginChild("##tab", ImVec2(0, 0));
                ImGui::Dummy(ImVec2(0, 2));
                (this->*fn)();
                ImGui::EndChild();
                ImGui::EndTabItem();
            }
        };
        const ImGuiTabItemFlags gfx_flags = m_open_graphics_tab ? ImGuiTabItemFlags_SetSelected : 0;
        m_open_graphics_tab = false;
        tab("Properties", &ModelEditor::ui_properties);
        tab("Structure", &ModelEditor::ui_structure);
        tab("Graphics", &ModelEditor::ui_graphics, gfx_flags);
        tab("Vehicle", &ModelEditor::ui_vehicle);
        ImGui::EndTabBar();
    }
    ImGui::PopFont();
    ImGui::End();
}

void ModelEditor::ui_structure() {
    if (ImGui::CollapsingHeader("Layers and groups", ImGuiTreeNodeFlags_DefaultOpen)) ui_layers();
    if (ImGui::CollapsingHeader("All elements")) ui_elements();
    if (ImGui::CollapsingHeader("Reference mesh")) ui_reference();
}

void ModelEditor::ui_transform() {
    if (!ImGui::TreeNode("Move, rotate, scale by numbers")) return;
    hint("About the selection's centre; with symmetry the twins follow.");
    if (props_begin("##transform", 0.3f)) {
        prop("Move by");
        ImGui::DragFloat3("##mv", &m_move_by.x, 0.01f, -10, 10, "%.3f");
        prop("");
        if (ImGui::Button("Move##go", ImVec2(-1, 0)) && length2(m_move_by) > 0) {
            push_undo();
            move_selection(m_move_by);
        }
        prop("Rotate (deg)");
        ImGui::DragFloat3("##rot", &m_rotate_by.x, 0.5f, -180, 180, "%.1f");
        prop("");
        if (ImGui::Button("Rotate##go", ImVec2(-1, 0)) && length2(m_rotate_by) > 0) {
            push_undo();
            const vec3 c = gizmo_center();
            const quat q = quat_euler_xyz_deg(m_rotate_by.x, m_rotate_by.y, m_rotate_by.z);
            m_op_a = c;
            op_begin_nodes();
            op_apply([&](vec3 p) { return c + q.rotate(p - c); });
        }
        prop("Scale");
        ImGui::SliderFloat("##sc", &m_scale_by, 0.1f, 4.0f, "%.3f", ImGuiSliderFlags_Logarithmic);
        prop("");
        if (ImGui::Button("Scale##go", ImVec2(-1, 0)) && m_scale_by > 0 && m_scale_by != 1.0f) {
            push_undo();
            m_op_a = gizmo_center();
            op_begin_nodes();
            op_scale(vec3(m_scale_by));
        }
        props_end();
    }
    if (ImGui::SmallButton("Onto z = 0")) {
        push_undo();
        for (int n : m_sel) m_model.nodes[n].p.z = 0;
    }
    ImGui::SetItemTooltip("Onto the symmetry plane");
    ImGui::SameLine();
    if (ImGui::SmallButton("Drop to the ground")) {
        push_undo();
        float mn = 1e30f;
        for (int n : m_sel) mn = std::min(mn, m_model.nodes[n].p.y);
        move_selection(vec3(0, -mn, 0));
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Snap to grid")) {
        push_undo();
        for (int n : selection_with_twins()) m_model.nodes[n].p = snap(m_model.nodes[n].p);
    }
    ImGui::TreePop();
}

void ModelEditor::ui_selection_filter() {
    // what is selected, a row per kind; with more than one kind: Only keeps that kind alone, x deselects it
    static const Icon icons[] = {Icon::Node, Icon::Line, Icon::Shell, Icon::Tri, Icon::Shock, Icon::Rod, Icon::Wheel, Icon::Link};
    int kinds = 0;
    for (int k = 0; k < (int)SelKind::Count; k++) kinds += sel_count((SelKind)k) > 0;
    const float h = ImGui::GetFrameHeight(), sp = 4.0f;
    const float only_w = ImGui::CalcTextSize("Only").x + ImGui::GetStyle().FramePadding.x * 2;
    ImDrawList* dl = ImGui::GetWindowDrawList();
    for (int k = 0; k < (int)SelKind::Count; k++) {
        const SelKind kind = (SelKind)k;
        const int n = sel_count(kind);
        if (!n) continue;
        ImGui::PushID(k);
        const float x0 = ImGui::GetCursorPosX(), avail = ImGui::GetContentRegionAvail().x;
        const ImVec2 p = ImGui::GetCursorScreenPos();
        draw_icon(dl, icons[k], p.x + 2, p.y + 2, h - 4, ImGui::GetColorU32(kAccent));
        ImGui::SetCursorPosX(x0 + h + 6);
        ImGui::AlignTextToFramePadding();
        ImGui::TextColored(kAccent, "%d %s", n, sel_kind_name(kind, n != 1));
        if (kinds > 1) {
            char tip[96];
            ImGui::SameLine(x0 + avail - only_w - sp - h);
            if (ImGui::Button("Only", ImVec2(only_w, h))) select_filter(kind, true);
            snprintf(tip, sizeof tip, "Keep only the %s selected (Alt+%d)", sel_kind_name(kind, true), k + 1);
            ImGui::SetItemTooltip("%s", tip);
            ImGui::SameLine(0, sp);
            snprintf(tip, sizeof tip, "Deselect the %s (Alt+Shift+%d)", sel_kind_name(kind, true), k + 1);
            if (icon_button("##drop", Icon::Close, false, tip, h)) select_filter(kind, false);
        }
        ImGui::PopID();
    }
}

void ModelEditor::ui_properties() {
    edit::Model& M = m_model;
    const int ne = selected_elements();
    if (m_sel.empty() && ne == 0) {
        // nothing selected: the model at a glance and its checks
        hint("Nothing selected. Click a node or an element, or drag a box (left to right: inside, right to left: touching).");
        section_title("The model");
        if (props_begin("##summary", 0.5f)) {
            auto row = [&](const char* l, const std::string& v) {
                prop(l);
                ImGui::TextUnformatted(v.c_str());
            };
            row("Nodes", std::to_string(M.nodes.size()));
            row("Beams", std::to_string(M.beams.size()));
            row("Triangles", std::to_string(M.tris.size()) + (M.shell_count() ? "  (" + std::to_string(M.shell_count()) + " shells)" : std::string()) +
                                 (M.fem_count() ? "  (" + std::to_string(M.fem_count()) + " FEM)" : std::string()));
            row("Wheels", std::to_string(M.wheels.size()));
            row("Shocks, rods, joints", std::to_string(M.shocks.size()) + ", " + std::to_string(M.hydros.size()) + ", " + std::to_string(M.joints.size()));
            row("Meshes", std::to_string(M.flexbodies.size() + M.props.size()));
            row("Folder", M.home);
            props_end();
        }
        section_title("Checks");
        const std::vector<std::string> problems = edit::validate(M);
        if (problems.empty()) ImGui::TextColored(ImVec4(0.5f, 1, 0.5f, 1), "No problems found");
        ImGui::PushTextWrapPos(0);
        for (const std::string& s : problems) ImGui::TextColored(ImVec4(1, 0.6f, 0.3f, 1), "%s", s.c_str());
        ImGui::PopTextWrapPos();
        if (!m_notes.empty() && ImGui::TreeNode("Import notes")) {
            ImGui::PushTextWrapPos(0);
            for (const std::string& s : m_notes) ImGui::TextDisabled("%s", s.c_str());
            ImGui::PopTextWrapPos();
            ImGui::TreePop();
        }
        return;
    }
    ui_selection_filter();
    if (props_begin("##selprops")) {
        std::vector<const char*> names;
        for (auto& l : M.layers) names.push_back(l.name.c_str());
        int cur = 0;
        if (!m_sel.empty()) cur = M.nodes[m_sel[0]].layer;
        else if (!m_sel_beams.empty()) cur = M.beams[m_sel_beams[0]].layer;
        else if (!m_sel_tris.empty()) cur = M.tris[m_sel_tris[0]].layer;
        prop("Layer", "Moves the selection (the nodes with the elements among them) to a layer");
        if (ImGui::Combo("##layer", &cur, names.data(), (int)names.size())) set_selection_layer(cur);
        props_end();
    }
    if (!m_sel.empty()) {
        section_title(m_sel.size() == 1 ? ("Node " + std::to_string(m_sel[0])).c_str() : "Nodes");
        if (props_begin("##nodeprops")) {
            if (m_sel.size() == 1) {
                edit::Node& n = M.nodes[m_sel[0]];
                vec3 p = n.p;
                prop("Position", "x: forward is -x, y: up, z: left is +z");
                if (ImGui::DragFloat3("##pos", &p.x, 0.005f, -50, 50, "%.3f")) {
                    if (!m_drag_pushed_ui) push_undo(), m_drag_pushed_ui = true;
                    move_selection(p - n.p);
                }
                if (!ImGui::IsItemActive()) m_drag_pushed_ui = false;
                prop("Used by");
                ImGui::TextDisabled("%d elements", M.node_uses(m_sel[0]));
            } else {
                const vec3 c = gizmo_center();
                vec3 p = c;
                prop("Centre");
                if (ImGui::DragFloat3("##centre", &p.x, 0.005f, -50, 50, "%.3f")) {
                    if (!m_drag_pushed_ui) push_undo(), m_drag_pushed_ui = true;
                    move_selection(p - c);
                }
                if (!ImGui::IsItemActive()) m_drag_pushed_ui = false;
                if (m_sel.size() == 2) prop("Distance"), ImGui::TextDisabled("%.3f m", length(M.nodes[m_sel[0]].p - M.nodes[m_sel[1]].p));
            }
            auto flag = [&](const char* label, bool edit::Node::*f, const char* tip) {
                bool v = M.nodes[m_sel[0]].*f;
                prop(label, tip);
                if (ImGui::Checkbox((std::string("##") + label).c_str(), &v)) {
                    push_undo();
                    for (int i : selection_with_twins()) M.nodes[i].*f = v;
                }
            };
            flag("Load bearing", &edit::Node::load_bearing, "l: takes a share of the cargo mass (the frame's nodes)");
            flag("No ground", &edit::Node::no_ground, "c: passes through the terrain (inner nodes)");
            flag("Collides", &edit::Node::contacter, "Collides with other bodies");
            flag("Fixed", &edit::Node::fixed, "Nailed to the world (a stand, a wall)");
            float load = M.nodes[m_sel[0]].load;
            bool has_load = load >= 0;
            prop("Own mass", "A load weight (kg) on top of the frame's share");
            if (ImGui::Checkbox("##ownmass", &has_load)) {
                push_undo();
                for (int i : selection_with_twins()) M.nodes[i].load = has_load ? 10.0f : -1.0f, M.nodes[i].load_bearing |= has_load;
            }
            if (has_load) {
                ImGui::SameLine();
                ImGui::SetNextItemWidth(-FLT_MIN);
                if (ImGui::SliderFloat("##kg", &load, 0.1f, 1000.0f, "%.1f kg", ImGuiSliderFlags_Logarithmic))
                    for (int i : selection_with_twins()) M.nodes[i].load = std::max(0.0f, load), m_dirty = true;
            }
            std::vector<const char*> gnames = {"(none)"};
            for (auto& g : M.node_groups) gnames.push_back(g.name.c_str());
            int gcur = M.nodes[m_sel[0]].group + 1;
            prop("Group");
            if (ImGui::Combo("##group", &gcur, gnames.data(), (int)gnames.size())) {
                push_undo();
                for (int i : selection_with_twins()) M.nodes[i].group = gcur - 1;
            }
            props_end();
        }
        {
            // the meshes bound to these nodes: a click selects one (the Graphics tab)
            std::vector<std::pair<int, int>> parts;
            for (int k = 1; k <= 2; k++)
                for (int i = 0; i < gfx_count(k); i++) {
                    bool on = false;
                    for (int n : gfx_nodes(k, i)) on |= is_selected(n);
                    if (on) parts.push_back({k, i});
                }
            if (!parts.empty()) {
                char hdr[64];
                snprintf(hdr, sizeof hdr, "Meshes on these nodes (%d)###meshes_on", (int)parts.size());
                if (ImGui::TreeNode(hdr)) {
                    for (auto [k, i] : parts) {
                        ImGui::PushID(k * 100000 + i);
                        if (ImGui::Selectable(gfx_name(k, i).c_str(), m_gfx_kind == k && m_gfx_sel == i)) gfx_select(k, i), m_open_graphics_tab = true;
                        if (ImGui::IsItemHovered()) m_gfx_ui_hover = gfx_code(k, i);
                        ImGui::PopID();
                    }
                    ImGui::TreePop();
                }
            }
        }
        ui_transform();
    }
    if (!ne) return;
    if (!m_sel_beams.empty()) {
        section_title(m_sel_beams.size() == 1 ? ("Beam " + std::to_string(m_sel_beams[0])).c_str() : "Beams");
        if (props_begin("##beamprops")) {
            if (m_sel_beams.size() == 1) {
                const edit::Beam& b = M.beams[m_sel_beams[0]];
                prop("Nodes");
                ImGui::Text("%d - %d, %.3f m", b.a, b.b, length(M.nodes[b.a].p - M.nodes[b.b].p));
            }
            std::vector<const char*> names;
            for (auto& g : M.groups) names.push_back(g.name.c_str());
            int cur = M.beams[m_sel_beams[0]].group;
            prop("Preset");
            if (ImGui::Combo("##preset", &cur, names.data(), (int)names.size())) {
                push_undo();
                for (int i : m_sel_beams) M.beams[i].group = cur;
            }
            const edit::BeamGroup& g = M.groups[cur];
            int type = g.type;
            prop("Type", "The beams move to a preset of this type (made from theirs if needed)");
            if (ImGui::Combo("##type", &type, kBeamTypes, 4)) {
                push_undo();
                const int v = preset_variant(cur, type, g.hold_rotation);
                for (int i : m_sel_beams) M.beams[i].group = v;
            }
            int ends = M.groups[cur].hold_rotation ? 1 : 0;
            static const char* ends_names[] = {"free (pins)", "held (rigid joints)"};
            static const char* frame_ends[] = {"pinned", "welded (rigid)"};
            prop("Ends", "Held: each end keeps its angle to the rest of the structure (the beam bends instead of pivoting)");
            if (M.groups[cur].is_frame()) {
                (void)frame_ends;
                ImGui::TextDisabled("joints below");
                for (int end = 0; end < 2; end++) {
                    prop(end ? "Joint at B" : "Joint at A", "Their own joint at this end (the preset's: its name first); the Joint tool clicks them one by one");
                    int j = beam_joint(m_sel_beams[0], end);
                    ImGui::PushID(end);
                    if (joint_combo("##bj", j)) {
                        push_undo();
                        for (int i : m_sel_beams) set_beam_joint(i, end, j);
                    }
                    ImGui::PopID();
                }
            } else
            if (ImGui::Combo("##ends", &ends, ends_names, 2)) {
                push_undo();
                const int v = preset_variant(cur, M.groups[cur].type, ends == 1);
                for (int i : m_sel_beams) M.beams[i].group = v;
            }
            prop("");
            if (ImGui::SmallButton("Edit the preset")) m_preset_window = true, m_preset_new = false, m_preset_edit = M.beams[m_sel_beams[0]].group;
            props_end();
        }
    }
    if (m_sel_shocks.size() == 1) {
        edit::Shock& s = M.shocks[m_sel_shocks[0]];
        section_title(("Shock " + std::to_string(m_sel_shocks[0])).c_str());
        if (props_begin("##shockprops")) {
            prop("Nodes");
            ImGui::Text("%d - %d", s.a, s.b);
            prop("Spring");
            m_dirty |= ImGui::SliderFloat("##sk", &s.spring, 1000.0f, 1.0e6f, "%.3g N/m", ImGuiSliderFlags_Logarithmic);
            prop("Damping");
            m_dirty |= ImGui::SliderFloat("##sd", &s.damp, 10.0f, 1.0e5f, "%.3g N s/m", ImGuiSliderFlags_Logarithmic);
            prop("Shortens by", "A fraction of its length");
            m_dirty |= ImGui::SliderFloat("##sb", &s.short_bound, 0.0f, 1.0f, "%.2f");
            prop("Lengthens by", "A fraction of its length");
            m_dirty |= ImGui::SliderFloat("##lb", &s.long_bound, 0.0f, 1.0f, "%.2f");
            prop("Precompression");
            m_dirty |= ImGui::SliderFloat("##pc", &s.precomp, 0.5f, 1.5f, "%.2f");
            prop("Stops", "The spring and damping of its stops (the beam defaults in effect): light nodes need soft stops");
            m_dirty |= ImGui::SliderFloat("##stk", &s.bd.spring, 1.0e4f, 2.0e7f, "%.3g N/m", ImGuiSliderFlags_Logarithmic);
            prop("");
            m_dirty |= ImGui::SliderFloat("##std", &s.bd.damp, 1.0f, 5.0e4f, "%.3g N s/m", ImGuiSliderFlags_Logarithmic);
            prop("Invisible");
            m_dirty |= ImGui::Checkbox("##sinv", &s.invisible);
            props_end();
        }
    } else if (!m_sel_shocks.empty()) {
        section_title("Shocks");
    }
    if (m_sel_hydros.size() == 1) {
        edit::Hydro& h = M.hydros[m_sel_hydros[0]];
        section_title(("Steering rod " + std::to_string(m_sel_hydros[0])).c_str());
        if (props_begin("##hydroprops")) {
            prop("Nodes");
            ImGui::Text("%d - %d", h.a, h.b);
            prop("Factor", "Length change at full steering, a fraction (negative: the other way)");
            m_dirty |= ImGui::SliderFloat("##hf", &h.factor, -1.0f, 1.0f, "%.3f");
            prop("Spring");
            m_dirty |= ImGui::SliderFloat("##hk", &h.bd.spring, 1.0e4f, 3.0e7f, "%.3g N/m", ImGuiSliderFlags_Logarithmic);
            prop("Damping");
            m_dirty |= ImGui::SliderFloat("##hd", &h.bd.damp, 1.0f, 5.0e4f, "%.3g N s/m", ImGuiSliderFlags_Logarithmic);
            prop("Speed dependent");
            m_dirty |= ImGui::Checkbox("##hs", &h.speed_dep);
            prop("Invisible");
            m_dirty |= ImGui::Checkbox("##hi", &h.invisible);
            props_end();
        }
    }
    if (m_sel_joints.size() == 1) {
        edit::Joint& j = M.joints[m_sel_joints[0]];
        section_title(("Joint " + std::to_string(m_sel_joints[0])).c_str());
        if (props_begin("##jointprops")) {
            prop("Parent, child");
            ImGui::Text("%d, %d", j.parent, j.child);
            prop("Hold stiffness");
            m_dirty |= ImGui::SliderFloat("##jk", &j.k, 1000.0f, 1.0e6f, "%.3g N/m", ImGuiSliderFlags_Logarithmic);
            prop("Breaks at", "The box: never");
            m_dirty |= never_field("jbrk", &j.brk, 100.0f, 1.0e6f, "%.3g N");
            if (j.brk >= 1e29f) j.brk = 0; // (0 is never for joints)
            props_end();
        }
    }
    if (!m_sel_tris.empty()) {
        int shells = 0, fems = 0;
        for (int i : m_sel_tris) shells += M.tris[i].shell, fems += M.tris[i].fem;
        section_title(m_sel_tris.size() == 1 ? ("Triangle " + std::to_string(m_sel_tris[0])).c_str() : "Triangles");
        if (props_begin("##triprops")) {
            int hulls = 0;
            for (int i : m_sel_tris) hulls += M.tris[i].hull();
            int kind = fems ? 3 : shells ? 1 : hulls ? 2 : 0;
            static const char* kinds[] = {"cab (collision surface)", "shell (sheet element)", "hull (solid collision, faces out)", "FEM shell (frame element)"};
            prop("Kind");
            if (ImGui::Combo("##kind", &kind, kinds, 4)) {
                push_undo();
                if (kind == 3) M.ensure_fem_preset();
                for (int i : m_sel_tris) {
                    edit::Tri& t = M.tris[i];
                    if (kind == 3 && !t.fem) t.fem_preset = std::clamp(m_fem_preset, 0, (int)M.fem_presets.size() - 1);
                    if (t.fem && kind != 3) t.collision = true;
                    t.shell = kind == 1, t.fem = kind == 3;
                    if (t.fem) t.collision = false;
                    t.options.erase(std::remove(t.options.begin(), t.options.end(), 'h'), t.options.end());
                    if (kind == 2) t.options += 'h', t.collision = true;
                }
                fems = kind == 3 ? (int)m_sel_tris.size() : 0;
            }
            if (kind == 3) ImGui::SetItemTooltip("A shell element of the FEM frame: it stretches and bends (solved with the frame's beams), yields and keeps\n"
                                                 "the dent past its material's yield stress, tears past its elongation; its own collision surface");
            if (kind == 2) ImGui::SetItemTooltip("One-sided: a node up to 30 cm behind it is pushed back out along its outward normal (the amber tick).\n"
                                                 "Lay it on a frame's nodes round the body: two frames can't go through each other.");
            bool coll = M.tris[m_sel_tris[0]].collision;
            if (!fems) {
                prop("Collision");
                if (ImGui::Checkbox("##coll", &coll)) {
                    push_undo();
                    for (int i : m_sel_tris) M.tris[i].collision = coll;
                }
            }
            if (fems) {
                std::vector<std::string> names;
                const int np = std::max(1, (int)M.fem_presets.size());
                for (int i = 0; i < np; i++) names.push_back(M.fem_preset(i).name);
                int cur = 0;
                for (int i : m_sel_tris)
                    if (M.tris[i].fem) cur = std::clamp(M.tris[i].fem_preset, 0, np - 1);
                prop("FEM shell", "The material and thickness (the FEM shells list in the left panel)");
                if (ImGui::BeginCombo("##tfem", names[cur].c_str())) {
                    for (int k = 0; k < np; k++)
                        if (ImGui::Selectable(names[k].c_str(), k == cur)) {
                            push_undo();
                            for (int i : m_sel_tris)
                                if (M.tris[i].fem) M.tris[i].fem_preset = k;
                        }
                    ImGui::EndCombo();
                }
                prop("");
                if (ImGui::SmallButton("Edit the shell")) M.ensure_fem_preset(), m_fem_window = true, m_fem_edit = cur;
            }
            if (shells) {
                std::vector<std::string> names;
                for (int i = 0; i < M.shell_preset_count(); i++) names.push_back(M.shell_preset(i).name + " (" + M.shell_preset(i).material + ")");
                int cur = 0;
                for (int i : m_sel_tris)
                    if (M.tris[i].shell) cur = std::clamp(M.tris[i].shell_preset, 0, (int)names.size() - 1);
                prop("Material", "What the shells are made of (the Shell materials list in the left panel)");
                if (ImGui::BeginCombo("##tmat", names[cur].c_str())) {
                    for (int k = 0; k < (int)names.size(); k++)
                        if (ImGui::Selectable(names[k].c_str(), k == cur)) {
                            push_undo();
                            for (int i : m_sel_tris)
                                if (M.tris[i].shell) M.tris[i].shell_preset = k;
                        }
                    ImGui::EndCombo();
                }
                prop("");
                if (ImGui::SmallButton("Edit the material")) m_shell_window = true, m_shell_edit = cur, m_shell_new = false;
            }
            prop("Outside", "The white line shows the outside");
            if (ImGui::SmallButton("Flip")) {
                push_undo();
                for (int i : m_sel_tris) std::swap(M.tris[i].b, M.tris[i].c);
            }
            props_end();
        }
    }
    if (m_sel_wheels.size() == 1) {
        edit::Wheel& w = M.wheels[m_sel_wheels[0]];
        section_title(("Wheel " + std::to_string(m_sel_wheels[0])).c_str());
        if (props_begin("##wheelprops")) {
            prop("Axle nodes");
            ImGui::Text("%d - %d", w.n1, w.n2);
            prop("Kind");
            m_dirty |= ImGui::Combo("##wt", &w.type, kWheelTypes, 5);
            prop("Radius");
            m_dirty |= ImGui::SliderFloat("##wr", &w.radius, 0.05f, 1.5f, "%.3f m");
            if (w.type != 0) prop("Rim radius"), m_dirty |= ImGui::SliderFloat("##wrr", &w.rim_radius, 0.02f, 1.2f, "%.3f m");
            prop("Width");
            m_dirty |= ImGui::SliderFloat("##ww", &w.width, 0.02f, 1.0f, "%.3f m");
            prop("Rays");
            m_dirty |= ImGui::SliderInt("##wrays", &w.rays, 3, 32);
            prop("Mass");
            m_dirty |= ImGui::SliderFloat("##wm", &w.mass, 1.0f, 500.0f, "%.1f kg", ImGuiSliderFlags_Logarithmic);
            prop("Tyre spring");
            m_dirty |= ImGui::SliderFloat("##wk", &w.spring, 1.0e3f, 3.0e6f, "%.3g", ImGuiSliderFlags_Logarithmic);
            prop("Tyre damping");
            m_dirty |= ImGui::SliderFloat("##wd", &w.damp, 10.0f, 1.0e5f, "%.3g", ImGuiSliderFlags_Logarithmic);
            if (w.type == 1 || w.type == 4) {
                prop("Rim spring");
                m_dirty |= ImGui::SliderFloat("##wrk", &w.rim_spring, 1.0e3f, 1.0e7f, "%.3g", ImGuiSliderFlags_Logarithmic);
                prop("Rim damping");
                m_dirty |= ImGui::SliderFloat("##wrd", &w.rim_damp, 10.0f, 1.0e5f, "%.3g", ImGuiSliderFlags_Logarithmic);
            }
            prop("Braked");
            bool braked = w.braking != 0;
            if (ImGui::Checkbox("##wb", &braked)) w.braking = braked ? 1 : 0, m_dirty = true;
            prop("Driven");
            static const char* prop_names[] = {"no", "forward", "backward"};
            m_dirty |= ImGui::Combo("##wp", &w.propulsion, prop_names, 3);
            prop("Arm node", "The node the reaction torque acts on (-1: chosen)");
            m_dirty |= ImGui::InputInt("##warm", &w.arm);
            props_end();
        }
        if (ImGui::TreeNode("Beams and grip")) {
            hint("The beam defaults in effect for the wheel: its beams' strength and set; meshwheels2 also takes their spring and damping for the beams between the tyre nodes (light nodes: keep it soft).");
            if (props_begin("##wheelbd")) {
                prop("Beam spring");
                m_dirty |= ImGui::SliderFloat("##wbk", &w.bd.spring, 1.0e4f, 2.0e7f, "%.3g N/m", ImGuiSliderFlags_Logarithmic);
                prop("Beam damping");
                m_dirty |= ImGui::SliderFloat("##wbd", &w.bd.damp, 1.0f, 5.0e4f, "%.3g", ImGuiSliderFlags_Logarithmic);
                prop("Deforms at");
                m_dirty |= never_field("wdef", &w.bd.deform, 1.0e4f, 1.0e8f, "%.3g N");
                prop("Breaks at");
                m_dirty |= never_field("wbrk", &w.bd.brk, 1.0e4f, 1.0e9f, "%.3g N");
                prop("Tyre friction");
                m_dirty |= ImGui::SliderFloat("##wf", &w.friction, 0.2f, 2.0f, "%.2f");
                props_end();
            }
            ImGui::TreePop();
        }
        if (w.type >= 2) hint("Its meshes: the Graphics tab.");
    }
    ImGui::Spacing();
    if (ImGui::Button("Delete the elements (the nodes stay)", ImVec2(-1, 0))) {
        const std::vector<int> keep = m_sel;
        m_sel.clear();
        delete_selection();
        m_sel = keep;
    }
}

void ModelEditor::ui_vehicle() {
    edit::Model& M = m_model;
    section_title("Mass");
    if (props_begin("##mass")) {
        prop("Dry mass", "Spread over the beams (a node gets the share of its beams' length)");
        m_dirty |= ImGui::SliderFloat("##dry", &M.dry_mass, 1.0f, 40000.0f, "%.0f kg", ImGuiSliderFlags_Logarithmic);
        prop("Cargo", "Split between the load bearing nodes");
        m_dirty |= ImGui::SliderFloat("##cargo", &M.cargo_mass, 0.0f, 20000.0f, "%.0f kg");
        prop("Min node mass", "Lighter nodes are unstable with stiff beams (k dt^2 / m); RoR's default is 50");
        m_dirty |= ImGui::SliderFloat("##minm", &M.minimass, 0.5f, 100.0f, "%.1f kg", ImGuiSliderFlags_Logarithmic);
        props_end();
    }
    section_title("Shells (the sheet body)");
    hint("The shell triangles' materials (steel panels, glass, plastic...) are in the left panel: Shell materials.");
    if (props_begin("##sheet")) {
        prop("Shells");
        ImGui::Text("%d", M.shell_count());
        prop("Materials");
        ImGui::Text("%d", M.shell_preset_count());
        props_end();
    }
    section_title("Drivetrain");
    if (props_begin("##drive")) {
        prop("Engine");
        m_dirty |= ImGui::Checkbox("##engine", &M.engine);
        if (M.engine) {
            prop("Torque");
            m_dirty |= ImGui::SliderFloat("##torque", &M.torque, 10.0f, 5000.0f, "%.0f N m", ImGuiSliderFlags_Logarithmic);
            prop("Idle / max rpm");
            float rpm[2] = {M.min_rpm, M.max_rpm};
            if (ImGui::DragFloat2("##rpm", rpm, 10.0f, 300.0f, 12000.0f, "%.0f")) M.min_rpm = rpm[0], M.max_rpm = std::max(rpm[0] + 100, rpm[1]), m_dirty = true;
            prop("Differential");
            m_dirty |= ImGui::SliderFloat("##diff", &M.diff, 1.0f, 12.0f, "%.2f");
            prop("Reverse");
            m_dirty |= ImGui::SliderFloat("##rev", &M.reverse, 1.0f, 8.0f, "%.2f");
            prop("Gears");
            int ng = (int)M.gears.size();
            if (ImGui::SliderInt("##ng", &ng, 1, 10)) {
                while ((int)M.gears.size() < ng) M.gears.push_back(M.gears.empty() ? 3.0f : M.gears.back() * 0.75f);
                M.gears.resize(ng);
                m_dirty = true;
            }
            for (int i = 0; i < (int)M.gears.size(); i++) {
                ImGui::PushID(i);
                char l[16];
                snprintf(l, sizeof l, "  gear %d", i + 1);
                prop(l);
                m_dirty |= ImGui::SliderFloat("##g", &M.gears[i], 0.3f, 8.0f, "%.2f");
                ImGui::PopID();
            }
            prop("Automatic clutch", "A car engine (c); off: a truck (t)");
            m_dirty |= ImGui::Checkbox("##car", &M.engine_car);
            prop("Clutch force");
            m_dirty |= ImGui::SliderFloat("##clutch", &M.clutch_force, 10.0f, 5000.0f, "%.0f", ImGuiSliderFlags_Logarithmic);
            prop("Engine inertia");
            m_dirty |= ImGui::SliderFloat("##inertia", &M.inertia, 0.02f, 5.0f, "%.2f", ImGuiSliderFlags_Logarithmic);
        }
        prop("Brake force");
        m_dirty |= ImGui::SliderFloat("##brake", &M.brake_force, 100.0f, 50000.0f, "%.0f", ImGuiSliderFlags_Logarithmic);
        props_end();
    }
    section_title("Cameras");
    hint("The nodes the game orients the vehicle by (-1: chosen automatically).");
    if (props_begin("##cams")) {
        auto cam = [&](const char* label, int* v) {
            ImGui::PushID(label);
            prop(label);
            ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - 80);
            m_dirty |= ImGui::InputInt("##n", v);
            ImGui::SameLine();
            ImGui::BeginDisabled(m_sel.size() != 1);
            if (ImGui::SmallButton("= selected")) *v = m_sel[0], m_dirty = true;
            ImGui::EndDisabled();
            ImGui::PopID();
        };
        cam("Centre", &M.cam_center);
        cam("Back", &M.cam_back);
        cam("Left", &M.cam_left);
        prop("Cockpit node", "A cinecam: a node above the centre on eight beams to the nearest nodes, where the cockpit camera rides (in the file only, for driving: not in the physics test). "
                             "It is a node and beams of its own and takes a share of the dry mass; without it the cockpit camera sits 1.3 m above the centre");
        m_dirty |= ImGui::Checkbox("##cinecam", &M.cinecam);
        props_end();
    }
}

void ModelEditor::ui_reference() {
    edit::Model& M = m_model;
    hint("A Wavefront .obj drawn with the model (not part of the vehicle): trace over it, snap points to its surface.");
    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - 60);
    ImGui::InputTextWithHint("##reffile", "a path or a file of assets/reference", m_ref_buf, sizeof m_ref_buf);
    ImGui::SameLine();
    if (ImGui::Button("Load", ImVec2(-1, 0))) ref_load(trim(m_ref_buf));
    const std::string dir = asset_path("reference");
    if (dir_exists(dir))
        for (const std::string& f : list_dir(dir, true, false))
            if (path_ext_lower(f) == ".obj" && ImGui::Selectable(f.c_str(), m_ref_loaded == f)) ref_load(f);
    if (m_ref_idx.empty()) return;
    if (props_begin("##refprops")) {
        prop("Triangles");
        ImGui::TextDisabled("%d", (int)(m_ref_idx.size() / 3));
        prop("Show");
        ImGui::Checkbox("##show", &m_ref_show);
        prop("Snap to it");
        ImGui::Checkbox("##snap", &m_ref_snap);
        prop("See-through");
        m_dirty |= ImGui::Checkbox("##mock", &M.ref_mockup);
        prop("Opacity");
        m_dirty |= ImGui::SliderFloat("##op", &M.ref_alpha, 0.05f, 1.0f, "%.2f");
        prop("Offset");
        m_dirty |= ImGui::DragFloat3("##off", &M.ref_offset.x, 0.005f, -20, 20, "%.3f");
        prop("Scale");
        m_dirty |= ImGui::SliderFloat("##scale", &M.ref_scale, 0.01f, 100.0f, "%.3f", ImGuiSliderFlags_Logarithmic);
        prop("Yaw");
        m_dirty |= ImGui::SliderFloat("##yaw", &M.ref_yaw, -180.0f, 180.0f, "%.1f deg");
        props_end();
    }
    if (ImGui::Button("Selected nodes onto it")) snap_to_surface();
    ImGui::SameLine();
    if (ImGui::Button("Remove")) {
        m_ref_v.clear(), m_ref_idx.clear(), m_ref_loaded.clear(), M.ref_path.clear();
        m_dirty = true;
    }
}

// ------------------------------------------------------------------------------------------------ the status bar
void ModelEditor::ui_status(ImFont* small) {
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x, vp->WorkPos.y + vp->WorkSize.y - m_status_h), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(vp->WorkSize.x, m_status_h), ImGuiCond_Always);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10, 5));
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove;
    const bool open = ImGui::Begin("##edstatus", nullptr, flags);
    ImGui::PopStyleVar();
    if (open) {
        ImGui::PushFont(small, 0.0f);
        // left: the tool and what to do (or the last message); right: the selection, the model, the switches
        const bool edit = m_mode == Mode::Edit;
        const char* name = m_mode == Mode::Deform ? "Deform" : m_mode == Mode::Physics ? "Physics" : kTools[(int)m_tool].name;
        std::string msg;
        if (m_gfx_pick) msg = m_gfx_pick == 6 ? "Click a mesh in a view (Esc ends)" : "Click a node for the mesh binding (Esc ends)";
        else if (vcb_active() && !m_vcb.empty()) msg = "Value: " + m_vcb + "_  (Enter)";
        else if (!m_status.empty()) msg = m_status;
        else msg = edit ? kToolShort[(int)m_tool] : m_mode == Mode::Deform ? "Drag a node or a beam; the meshes follow"
                                                  : m_phys_tool == 0 ? "Grab: drag a node" : m_phys_tool == 1 ? "Destroy: hold and sweep" : m_phys_tool == 2 ? "Shoot: click or hold" : "Laser: hold and sweep";
        const float ih = ImGui::GetTextLineHeight() + 2;
        const ImVec2 p = ImGui::GetCursorScreenPos();
        if (edit) draw_icon(ImGui::GetWindowDrawList(), kTools[(int)m_tool].icon, p.x, p.y, ih, IM_COL32(245, 158, 41, 255));
        ImGui::Dummy(ImVec2(ih, ih));
        ImGui::SameLine();
        ImGui::TextColored(kAccent, "%s", name);
        ImGui::SameLine();
        ImGui::TextUnformatted(msg.c_str());
        std::string right;
        const int nsel = (int)m_sel.size() + selected_elements();
        if (nsel) right += std::to_string(m_sel.size()) + " nodes, " + std::to_string(selected_elements()) + " elements selected   ";
        right += std::to_string(m_model.nodes.size()) + " nodes  " + std::to_string(m_model.beams.size()) + " beams  " + std::to_string(m_model.tris.size()) + " tris";
        if (m_gfx_only) right += "   graphics' nodes: " + std::to_string(m_gfx_mask_count);
        if (m_mode == Mode::Physics) right = format("speed %gx%s   ", m_game.world.settings.time_scale, m_game.paused ? " (paused)" : "") + right;
        right += std::string("   ") + (m_symmetry ? "symmetry" : "no symmetry") + (m_snap ? format(" / snap %.3f", m_snap_size) : std::string(" / no snap"));
        const float w = ImGui::CalcTextSize(right.c_str()).x;
        ImGui::SameLine(std::max(ImGui::GetCursorPosX() + 20, ImGui::GetWindowWidth() - w - 12));
        ImGui::TextDisabled("%s", right.c_str());
        ImGui::PopFont();
    }
    ImGui::End();
}

void ModelEditor::ui_mode_banner() {
    // (the test drive: the game's view, a strip at the top)
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + vp->WorkSize.x * 0.5f, vp->WorkPos.y + 12), ImGuiCond_Always, ImVec2(0.5f, 0));
    ImGui::SetNextWindowBgAlpha(0.9f);
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove;
    if (ImGui::Begin("##edmode", nullptr, flags)) {
        ImGui::TextColored(kAccent, "TEST DRIVE: %s", m_model.title.c_str());
        ImGui::SameLine();
        if (ImGui::Button("Back to the editor (Esc)")) end_mode();
        ImGui::SameLine();
        if (ImGui::Button("Reset") && m_test && alive(m_test)) m_test->reset(m_origin, 0.0f);
    }
    ImGui::End();
}

// ------------------------------------------------------------------------------------------------ kept
void ModelEditor::menu() {
    if (ImGui::MenuItem(m_active ? "Close the model editor" : "Model editor", "Ctrl+E")) {
        if (m_active) close();
        else open();
    }
    if (ImGui::MenuItem("Edit the current vehicle (with its graphics)", nullptr, false, m_game.player_vehicle() != nullptr)) open_vehicle(m_game.player_vehicle());
    if (ImGui::MenuItem("New model (cart)")) {
        set_model(edit::make_cart(2.2f, 1.2f, 0.7f, 400), "");
        open();
    }
    ImGui::Separator();
    ImGui::TextDisabled("Models are saved as .truck files in assets/vehicles/<folder>");
}

void ModelEditor::apply_preset_to_selection(int group) {
    push_undo();
    int n = 0;
    for (int i : m_sel_beams) m_model.beams[i].group = group, n++;
    for (int i = 0; i < (int)m_model.beams.size(); i++) {
        edit::Beam& b = m_model.beams[i];
        if (is_selected(b.a) && is_selected(b.b) && !elem_selected(Elem::Beam, i)) b.group = group, n++;
    }
    m_status = std::to_string(n) + " beams took " + m_model.groups[group].name;
}

void ModelEditor::set_selection_layer(int layer) {
    edit::Model& M = m_model;
    push_undo();
    M.set_layer(selection_with_twins(), layer);
    for (int i : m_sel_beams) M.beams[i].layer = layer;
    for (int i : m_sel_shocks) M.shocks[i].layer = layer;
    for (int i : m_sel_hydros) M.hydros[i].layer = layer;
    for (int i : m_sel_tris) M.tris[i].layer = layer;
    for (int i : m_sel_wheels) M.wheels[i].layer = layer;
    for (int i : m_sel_joints) M.joints[i].layer = layer;
}

bool ModelEditor::joint_combo(const char* id, int& j) {
    // the joints with what each does (hover)
    bool ch = false;
    j = std::clamp(j, 0, 5);
    if (ImGui::BeginCombo(id, kJointNames[j])) {
        for (int i = 0; i < 6; i++) {
            if (ImGui::Selectable(kJointNames[i], i == j)) j = i, ch = true;
            ImGui::SetItemTooltip("%s", kJointTips[i]);
        }
        ImGui::EndCombo();
    }
    return ch;
}

void ModelEditor::ui_joint_palette(int& j) {
    // the joints as a row of buttons with their marks (as drawn at the members' ends)
    const float bs = 34.0f;
    ImDrawList* dl = ImGui::GetWindowDrawList();
    for (int i = 0; i < 6; i++) {
        if (i) ImGui::SameLine(0, 4);
        ImGui::PushID(i);
        const bool on = j == i;
        if (on) ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
        if (ImGui::Button("##jt", ImVec2(bs, bs))) j = i;
        if (on) ImGui::PopStyleColor();
        ImGui::SetItemTooltip("%s\n%s", kJointNames[i], kJointTips[i]);
        const ImVec2 a = ImGui::GetItemRectMin();
        const ImU32 c = IM_COL32(225, 229, 236, 255), k = ImGui::GetColorU32(ImVec4(joint_color(i).x, joint_color(i).y, joint_color(i).z, 1));
        // a member coming from the left into the node on the right, the joint's mark between
        dl->AddLine(ImVec2(a.x + 4, a.y + bs * 0.62f), ImVec2(a.x + bs * 0.72f, a.y + bs * 0.62f), c, 3.0f);
        dl->AddCircleFilled(ImVec2(a.x + bs * 0.8f, a.y + bs * 0.62f), 3.5f, c);
        const ImVec2 m(a.x + bs * 0.5f, a.y + bs * 0.62f);
        switch (i) {
        case 0: dl->AddRectFilled(ImVec2(m.x - 4, m.y - 4), ImVec2(m.x + 4, m.y + 4), k); break;
        case 1: dl->AddCircle(m, 6.0f, k, 0, 2.5f); break;
        case 2: dl->AddLine(ImVec2(m.x, m.y - 10), ImVec2(m.x, m.y + 10), k, 3.0f), dl->AddCircle(m, 3.0f, k, 0, 2.0f); break;
        case 3: dl->AddLine(ImVec2(m.x - 7, m.y - 7), ImVec2(m.x + 7, m.y + 7), k, 3.0f), dl->AddCircle(m, 3.0f, k, 0, 2.0f); break;
        case 4: dl->AddEllipse(m, ImVec2(4.0f, 9.0f), k, 0, 0, 2.5f); break;
        default:
            for (int z = 0; z < 4; z++)
                dl->AddLine(ImVec2(m.x - 8 + z * 4, m.y + (z % 2 ? 5.0f : -5.0f)), ImVec2(m.x - 4 + z * 4, m.y + (z % 2 ? -5.0f : 5.0f)), k, 2.0f);
            break;
        }
        ImGui::PopID();
    }
}

bool ModelEditor::ui_frame_section(edit::BeamGroup& g) {
    // a frame element's section: material, shape, size, ends; with what they make of it (per metre and for a
    // 0.5 m member)
    bool ch = false;
    int nm = 0;
    const phys::FrameMaterial* mats = phys::frame_materials(nm);
    int mi = 0;
    for (int i = 0; i < nm; i++)
        if (g.frame_material == mats[i].name) mi = i;
    prop("Material", "Steel: mild structural; Chromoly: 4130 of roll cages; Aluminium 6061-T6; Titanium; Carbon: CFRP tube (strong, light, brittle); Wood");
    std::vector<const char*> names;
    for (int i = 0; i < nm; i++) names.push_back(mats[i].name);
    if (ImGui::Combo("##fmat", &mi, names.data(), nm)) g.frame_material = mats[mi].name, ch = true;
    prop("Shape", "Tube: round, outer diameter and wall; box: square hollow section, side and wall; rod and bar: solid round and square");
    static const char* shapes[] = {"tube", "box", "rod", "bar"};
    ch |= ImGui::Combo("##fshape", &g.frame_shape, shapes, 4);
    float outer = g.frame_outer * 1000.0f, wall = g.frame_wall * 1000.0f;
    prop(g.frame_shape == 0 || g.frame_shape == 2 ? "Diameter" : "Side");
    if (ImGui::SliderFloat("##fouter", &outer, 5.0f, 200.0f, "%.1f mm", ImGuiSliderFlags_Logarithmic)) g.frame_outer = outer * 0.001f, ch = true;
    if (g.frame_shape == 0 || g.frame_shape == 1) {
        prop("Wall");
        if (ImGui::SliderFloat("##fwall", &wall, 0.3f, std::max(0.5f, outer * 0.5f), "%.2f mm", ImGuiSliderFlags_Logarithmic)) g.frame_wall = wall * 0.001f, ch = true;
    }
    prop("Joint at A", "How the member's first end is joined to its node (a beam can have its own: the Joint tool, Properties)");
    ch |= joint_combo("##fja", g.frame_end_a);
    prop("Joint at B");
    ch |= joint_combo("##fjb", g.frame_end_b);
    if (g.frame_end_a == phys::FJ_ELASTIC || g.frame_end_b == phys::FJ_ELASTIC) {
        prop("Joint stiffness", "Elastic joints: the rotational spring (a rubber bushing some 1e3 - 1e4 N m/rad, a bolted joint 1e5 and up)");
        ch |= ImGui::SliderFloat("##fjk", &g.frame_joint_k, 100.0f, 1.0e7f, "%.3g N m/rad", ImGuiSliderFlags_Logarithmic);
    }
    if (g.frame_end_a != phys::FJ_RIGID || g.frame_end_b != phys::FJ_RIGID) {
        prop("Joint damping", "A released joint resists turning against its node: a door's hinge 2 - 5 N m s/rad, a ball joint's friction a few (0: free)");
        ch |= ImGui::SliderFloat("##fjd", &g.frame_joint_damp, 0.0f, 50.0f, g.frame_joint_damp > 0 ? "%.2g N m s/rad" : "none");
    }
    prop("Breaks at", "A mount that tears off its node A when the force at its ends stands past this for 5 ms (a bolt, a hinge, a bracket; 0: never, "
                      "the member breaks only as its material fails)");
    ch |= ImGui::SliderFloat("##fbrk", &g.frame_break, 0.0f, 50000.0f, g.frame_break > 0 ? "%.0f N" : "never", ImGuiSliderFlags_Logarithmic);
    prop("Invisible", "Not drawn in the game");
    ch |= ImGui::Checkbox("##invisible", &g.invisible);
    const phys::FrameSection s = phys::make_frame_section(g.frame_material, (phys::FrameShape)g.frame_shape, g.frame_outer, g.frame_wall);
    const double EI = (double)s.E * s.Iz, L = 0.5;
    prop("Mass");
    ImGui::TextDisabled("%.2f kg/m", s.mass_per_m());
    prop("Stiffness", "Bending EI and axial EA; a 0.5 m cantilever of it gives at its tip 3 EI / L^3");
    ImGui::TextDisabled("EI %.3g N m2, EA %.3g N", EI, (double)s.E * s.A);
    prop("");
    ImGui::TextDisabled("0.5 m cantilever: %.3g N/mm", 3 * EI / (L * L * L) * 1e-3);
    prop("Yields at", "The plastic moment (a hinge forms), the axial yield force; it breaks when the plastic strain reaches the material's elongation");
    ImGui::TextDisabled("%.0f N m bending, %.3g N axial", s.Mp, s.Np);
    return ch;
}

int ModelEditor::preset_variant(int group, int type, bool hold) {
    // the preset with the same numbers and this type / hold (made if there is none)
    edit::Model& M = m_model;
    const edit::BeamGroup& g = M.groups[group];
    if (g.type == type && (g.hold_rotation == hold || type == edit::BEAM_FRAME)) return group;
    for (int i = 0; i < (int)M.groups.size(); i++) {
        const edit::BeamGroup& h = M.groups[i];
        if (h.type == type && h.hold_rotation == hold && h.spring == g.spring && h.damp == g.damp && h.deform == g.deform && h.brk == g.brk && h.plastic == g.plastic &&
            h.invisible == g.invisible && h.frame_material == g.frame_material && h.frame_shape == g.frame_shape && h.frame_outer == g.frame_outer &&
            h.frame_wall == g.frame_wall)
            return i;
    }
    edit::BeamGroup v = g;
    v.type = type, v.hold_rotation = hold;
    if (type == edit::BEAM_FRAME) v.hold_rotation = false, v.color = vec4(0.55f, 0.72f, 1.0f, 1); // (welded by default: the section's joints)
    v.name = g.name + (type == edit::BEAM_ROPE ? " rope" : type == edit::BEAM_SUPPORT ? " support" : type == edit::BEAM_FRAME ? " FEM" : "") +
             (type != edit::BEAM_FRAME && hold ? " held" : "");
    M.groups.push_back(v);
    return (int)M.groups.size() - 1;
}

void ModelEditor::ui_layers() {
    edit::Model& M = m_model;
    ImGui::PushTextWrapPos(0);
    ImGui::TextDisabled("Layers hold nodes and elements: shown or hidden, editable or locked. New elements go to the checked layer.");
    ImGui::PopTextWrapPos();
    const float bs = ImGui::GetFrameHeight();
    int remove = -1;
    if (ImGui::BeginTable("layers", 4, ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH)) {
        ImGui::TableSetupColumn("##vis");
        ImGui::TableSetupColumn("##lock");
        ImGui::TableSetupColumn("##name", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("##act");
        for (int i = 0; i < (int)M.layers.size(); i++) {
            edit::Layer& l = M.layers[i];
            ImGui::PushID(i);
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            if (icon_button("##v", l.visible ? Icon::Eye : Icon::EyeOff, false, l.visible ? "Shown (click: hide)" : "Hidden (click: show)", bs)) l.visible = !l.visible, m_dirty = true;
            ImGui::TableNextColumn();
            if (icon_button("##l", l.locked ? Icon::Lock : Icon::Unlock, l.locked, l.locked ? "Locked: not selectable (click: unlock)" : "Editable (click: lock)", bs))
                l.locked = !l.locked, m_dirty = true;
            ImGui::TableNextColumn();
            if (ImGui::RadioButton("##cur", m_layer == i)) m_layer = i;
            ImGui::SetItemTooltip("New elements go here");
            ImGui::SameLine();
            char name[48];
            snprintf(name, sizeof name, "%s", l.name.c_str());
            ImGui::SetNextItemWidth(-1);
            if (ImGui::InputText("##n", name, sizeof name)) l.name = name, m_dirty = true;
            ImGui::TableNextColumn();
            if (icon_button("##s", Icon::Select, false, "Select what is on this layer", bs)) {
                clear_selection();
                for (int n = 0; n < (int)M.nodes.size(); n++)
                    if (M.nodes[n].layer == i && node_pickable(n)) m_sel.push_back(n);
                for (int k = 0; k < (int)M.beams.size(); k++)
                    if (M.beams[k].layer == i && elem_pickable(Elem::Beam, k)) m_sel_beams.push_back(k);
                for (int k = 0; k < (int)M.tris.size(); k++)
                    if (M.tris[k].layer == i && elem_pickable(Elem::Tri, k)) m_sel_tris.push_back(k);
            }
            ImGui::SameLine();
            if (icon_button("##p", Icon::Plus, false, "Put the selection on this layer", bs, !selection_empty())) set_selection_layer(i);
            if (i > 0) {
                ImGui::SameLine();
                if (icon_button("##x", Icon::Trash, false, "Remove the layer (what is on it goes to the first one)", bs)) remove = i;
            }
            ImGui::PopID();
        }
        ImGui::EndTable();
    }
    if (remove > 0) {
        push_undo();
        M.remove_layer(remove);
        if (m_layer >= (int)M.layers.size()) m_layer = 0;
        clear_selection();
    }
    if (ImGui::Button("New layer")) {
        push_undo();
        M.layers.push_back({"Layer " + std::to_string(M.layers.size() + 1), true, false});
        m_layer = (int)M.layers.size() - 1;
    }
    ImGui::SameLine();
    if (ImGui::Button("Solo")) {
        for (int i = 0; i < (int)M.layers.size(); i++) M.layers[i].visible = i == m_layer;
        m_dirty = true;
    }
    ImGui::SetItemTooltip("Show the checked layer only");
    ImGui::SameLine();
    if (ImGui::Button("Show all")) {
        for (auto& l : M.layers) l.visible = true;
        m_dirty = true;
    }
    section("Node groups");
    ImGui::PushTextWrapPos(0);
    ImGui::TextDisabled("Named sets of nodes (Editorizer's ;grp: comments), shown or hidden with what is on them.");
    ImGui::PopTextWrapPos();
    int gremove = -1;
    if (ImGui::BeginTable("groups", 3, ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH)) {
        ImGui::TableSetupColumn("##vis");
        ImGui::TableSetupColumn("##name", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("##act");
        for (int i = 0; i < (int)M.node_groups.size(); i++) {
            edit::Group& g = M.node_groups[i];
            ImGui::PushID(1000 + i);
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            if (icon_button("##v", g.visible ? Icon::Eye : Icon::EyeOff, false, "Shown / hidden", bs)) g.visible = !g.visible, m_dirty = true;
            ImGui::TableNextColumn();
            char name[48];
            snprintf(name, sizeof name, "%s", g.name.c_str());
            ImGui::SetNextItemWidth(-1);
            if (ImGui::InputText("##n", name, sizeof name)) g.name = name, m_dirty = true;
            ImGui::TableNextColumn();
            if (icon_button("##s", Icon::Select, false, "Select its nodes", bs)) {
                clear_selection();
                for (int n = 0; n < (int)M.nodes.size(); n++)
                    if (M.nodes[n].group == i && node_pickable(n)) m_sel.push_back(n);
            }
            ImGui::SameLine();
            if (icon_button("##p", Icon::Plus, false, "Put the selected nodes in this group", bs, !m_sel.empty())) {
                push_undo();
                for (int n : selection_with_twins()) M.nodes[n].group = i;
            }
            ImGui::SameLine();
            if (icon_button("##x", Icon::Trash, false, "Remove the group (its nodes stay)", bs)) gremove = i;
            ImGui::PopID();
        }
        ImGui::EndTable();
    }
    if (gremove >= 0) {
        push_undo();
        M.remove_group(gremove);
    }
    if (ImGui::Button("New group from the selection")) {
        push_undo();
        M.node_groups.push_back({"Group " + std::to_string(M.node_groups.size() + 1), true});
        for (int n : selection_with_twins()) M.nodes[n].group = (int)M.node_groups.size() - 1;
    }
}

void ModelEditor::ui_elements() {
    edit::Model& M = m_model;
    auto list = [&](const char* title, int count, Elem kind, auto row) {
        char hdr[64];
        snprintf(hdr, sizeof hdr, "%s (%d)###%s", title, count, title);
        if (!ImGui::CollapsingHeader(hdr)) return;
        ImGui::BeginChild(title, ImVec2(0, std::min(240.0f, 8 + count * ImGui::GetTextLineHeightWithSpacing())), ImGuiChildFlags_Borders);
        ImGuiListClipper clip;
        clip.Begin(count);
        while (clip.Step())
            for (int i = clip.DisplayStart; i < clip.DisplayEnd; i++) {
                ImGui::PushID(i);
                const bool sel = kind == Elem::None ? is_selected(i) : elem_selected(kind, i);
                const bool shown = kind == Elem::None ? node_shown(i) : elem_shown(kind, i);
                if (!shown) ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.5f, 0.5f, 0.5f, 1));
                if (ImGui::Selectable(row(i).c_str(), sel)) {
                    const bool shift = ImGui::GetIO().KeyShift, ctrl = ImGui::GetIO().KeyCtrl;
                    if (kind == Elem::None) select_node(i, shift, ctrl);
                    else select_elem(kind, i, shift, ctrl);
                }
                if (!shown) ImGui::PopStyleColor();
                ImGui::PopID();
            }
        ImGui::EndChild();
    };
    char buf[128];
    list("Nodes", (int)M.nodes.size(), Elem::None, [&](int i) {
        const edit::Node& n = M.nodes[i];
        snprintf(buf, sizeof buf, "%4d  %7.3f %7.3f %7.3f  %s%s%s", i, n.p.x, n.p.y, n.p.z, n.load_bearing ? "l" : "", n.no_ground ? "c" : "", n.fixed ? " fixed" : "");
        return std::string(buf);
    });
    list("Beams", (int)M.beams.size(), Elem::Beam, [&](int i) {
        const edit::Beam& b = M.beams[i];
        snprintf(buf, sizeof buf, "%4d  %d - %d  %s", i, b.a, b.b, M.groups[std::clamp(b.group, 0, (int)M.groups.size() - 1)].name.c_str());
        return std::string(buf);
    });
    list("Shocks", (int)M.shocks.size(), Elem::Shock, [&](int i) {
        snprintf(buf, sizeof buf, "%4d  %d - %d  k %.0f d %.0f", i, M.shocks[i].a, M.shocks[i].b, M.shocks[i].spring, M.shocks[i].damp);
        return std::string(buf);
    });
    list("Steering rods", (int)M.hydros.size(), Elem::Hydro, [&](int i) {
        snprintf(buf, sizeof buf, "%4d  %d - %d  factor %.2f", i, M.hydros[i].a, M.hydros[i].b, M.hydros[i].factor);
        return std::string(buf);
    });
    list("Joints", (int)M.joints.size(), Elem::Joint, [&](int i) {
        snprintf(buf, sizeof buf, "%4d  parent %d child %d  k %.3g", i, M.joints[i].parent, M.joints[i].child, M.joints[i].k);
        return std::string(buf);
    });
    list("Triangles", (int)M.tris.size(), Elem::Tri, [&](int i) {
        snprintf(buf, sizeof buf, "%4d  %d, %d, %d  %s%s", i, M.tris[i].a, M.tris[i].b, M.tris[i].c, M.tris[i].fem ? "FEM" : M.tris[i].shell ? "shell" : "cab",
                 M.tris[i].collision || M.tris[i].fem ? "" : " (no collision)");
        return std::string(buf);
    });
    list("Wheels", (int)M.wheels.size(), Elem::Wheel, [&](int i) {
        snprintf(buf, sizeof buf, "%4d  axle %d - %d  r %.2f  %s", i, M.wheels[i].n1, M.wheels[i].n2, M.wheels[i].radius, M.wheels[i].propulsion ? "driven" : "");
        return std::string(buf);
    });
}

void ModelEditor::gfx_pick_node(int n) {
    edit::Model& M = m_model;
    int *ref = nullptr, *x = nullptr, *y = nullptr;
    vec3 *off = nullptr, *rot = nullptr;
    std::vector<int>* forset = nullptr;
    if (m_gfx_kind == 1 && m_gfx_sel >= 0 && m_gfx_sel < (int)M.flexbodies.size()) {
        edit::Flexbody& f = M.flexbodies[m_gfx_sel];
        ref = &f.ref, x = &f.x, y = &f.y, off = &f.offset, rot = &f.rot, forset = &f.forset;
    } else if (m_gfx_kind == 2 && m_gfx_sel >= 0 && m_gfx_sel < (int)M.props.size()) {
        edit::Prop& p = M.props[m_gfx_sel];
        ref = &p.ref, x = &p.x, y = &p.y, off = &p.offset, rot = &p.rot;
    }
    if (!ref) {
        m_gfx_pick = 0;
        return;
    }
    push_undo();
    if (m_gfx_pick >= 1 && m_gfx_pick <= 3) {
        // the mesh stays where it is: its offset and rotation follow the new frame
        vec3 pos0(0);
        mat3 orient0;
        const bool keep = m_gfx_keep && M.ref_valid(*ref) && M.ref_valid(*x) && M.ref_valid(*y);
        if (keep) {
            vec3 X, Y, N;
            mat3 base;
            frame_of(M.ref_position(*ref), M.ref_position(*x), M.ref_position(*y), X, Y, N, base);
            pos0 = M.ref_position(*ref) + X * off->x + Y * off->y + N * off->z;
            orient0 = base * to_mat3(quat_euler_xyz_deg(rot->x, rot->y, rot->z));
        }
        (m_gfx_pick == 1 ? *ref : m_gfx_pick == 2 ? *x : *y) = n;
        if (keep && *ref != *x && *ref != *y && *x != *y) {
            vec3 X, Y, N;
            mat3 base;
            frame_of(M.ref_position(*ref), M.ref_position(*x), M.ref_position(*y), X, Y, N, base);
            const mat3 F(X, Y, N);
            if (std::fabs(determinant(F)) > 1e-9f) *off = inverse(F) * (pos0 - M.ref_position(*ref));
            *rot = euler_xyz_deg(transpose(base) * orient0);
        }
        m_status = std::string(m_gfx_pick == 1 ? "ref" : m_gfx_pick == 2 ? "x" : "y") + " node: " + std::to_string(n);
        m_gfx_pick = 0;
    } else if (forset) {
        auto it = std::lower_bound(forset->begin(), forset->end(), n);
        if (m_gfx_pick == 4 && (it == forset->end() || *it != n)) forset->insert(it, n);
        if (m_gfx_pick == 5 && it != forset->end() && *it == n) forset->erase(it);
        m_status = std::to_string(forset->size()) + " nodes in the forset (Esc ends)";
    }
}

std::vector<std::string> ModelEditor::home_meshes() const {
    std::vector<std::string> out;
    if (!dir_exists(folder())) return out;
    for (const std::string& f : list_dir(folder(), true, false)) {
        const std::string e = path_ext_lower(f);
        if (e == ".mesh" || e == ".obj") out.push_back(f);
    }
    std::sort(out.begin(), out.end());
    return out;
}

void ModelEditor::ui_views() {
    ImDrawList* bg = ImGui::GetBackgroundDrawList();
    // the splitters of the four views
    if (m_quad && m_maximized < 0) {
        const float sx = m_area[0] + m_area[2] * m_split_x, sy = m_area[1] + m_area[3] * m_split_y;
        const ImU32 c = IM_COL32(40, 44, 52, 255), hc = kAccentU;
        bg->AddRectFilled(V(sx - 1.5f, m_area[1]), V(sx + 1.5f, m_area[1] + m_area[3]), (m_split_hover & 1) ? hc : c);
        bg->AddRectFilled(V(m_area[0], sy - 1.5f), V(m_area[0] + m_area[2], sy + 1.5f), (m_split_hover & 2) ? hc : c);
    }
    for (int i = 0; i < 4; i++) {
        const View& v = m_views[i];
        if (!v.shown) continue;
        // the active view's frame
        if (i == m_active_view && (m_quad && m_maximized < 0)) bg->AddRect(V(v.rx + 0.5f, v.ry + 0.5f), V(v.rx + v.rw - 0.5f, v.ry + v.rh - 0.5f), IM_COL32(245, 158, 41, 160), 0, 0, 1.5f);
        // the title: its name, blow up / back
        ImGui::SetNextWindowPos(V(v.rx + 4, v.ry + 4), ImGuiCond_Always);
        ImGui::SetNextWindowBgAlpha(0.55f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(4, 2));
        char id[32];
        snprintf(id, sizeof id, "##viewtitle%d", i);
        if (ImGui::Begin(id, nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
                                         ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove)) {
            ImGui::TextColored(i == m_active_view ? kAccent : ImVec4(0.85f, 0.87f, 0.9f, 1), "%s", kViewNames[i]);
            ImGui::SetItemTooltip("%s\nDouble click: %s", kViewTips[i], m_quad ? (m_maximized == i ? "back to four views" : "blow up") : "four views");
            if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                if (!m_quad) m_quad = true, m_maximized = -1;
                else m_maximized = m_maximized == i ? -1 : i;
            }
            if (m_quad) {
                ImGui::SameLine();
                const float s = ImGui::GetTextLineHeight() + 2;
                if (icon_button("##max", m_maximized == i ? Icon::Quad : Icon::Frame, false, m_maximized == i ? "Back to four views" : "Blow up this view", s)) m_maximized = m_maximized == i ? -1 : i;
            }
            if (v.ortho == 0) {
                ImGui::SameLine();
                if (ImGui::SmallButton("iso")) m_views[i].yaw = 0.9f, m_views[i].pitch = 0.35f;
                ImGui::SetItemTooltip("Back to the standard 3D angle");
            }
        }
        ImGui::End();
        ImGui::PopStyleVar();
    }
}

void ModelEditor::ui_labels() {
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    const edit::Model& M = m_model;
    // node numbers in every view
    if (m_mode == Mode::Edit && (m_show_ids || M.nodes.size() <= 40)) {
        for (int vi = 0; vi < 4; vi++) {
            const View& v = m_views[vi];
            if (!v.shown) continue;
            dl->PushClipRect(V(v.rx, v.ry), V(v.rx + v.rw, v.ry + v.rh), true);
            for (int i = 0; i < (int)M.nodes.size(); i++) {
                if (!node_shown(i)) continue;
                vec2 s;
                if (!project(vi, to_world(M.nodes[i].p), s)) continue;
                char t[12];
                snprintf(t, sizeof t, "%d", i);
                dl->AddText(V(s.x + 6, s.y - 15), is_selected(i) ? IM_COL32(255, 170, 60, 255) : IM_COL32(235, 235, 235, 190), t);
            }
            dl->PopClipRect();
        }
    }
    ImDrawList* fg = ImGui::GetForegroundDrawList();
    // the box: blue inside (left to right), green touching (right to left)
    if (m_drag == Drag::Box) {
        const bool crossing = m_mouse.x < m_drag_start.x;
        const ImU32 fill = crossing ? IM_COL32(80, 220, 120, 36) : IM_COL32(80, 150, 255, 36), edge = crossing ? IM_COL32(80, 220, 120, 230) : IM_COL32(80, 150, 255, 230);
        fg->AddRectFilled(V(m_drag_start.x, m_drag_start.y), V(m_mouse.x, m_mouse.y), fill);
        fg->AddRect(V(m_drag_start.x, m_drag_start.y), V(m_mouse.x, m_mouse.y), edge, 0, 0, 1.5f);
    }
    // the inference at the cursor (SketchUp's tooltips) and the typed value
    if (m_mode == Mode::Edit && !ImGui::GetIO().WantCaptureMouse && view_at(m_mouse) >= 0) {
        std::string t;
        switch (m_pick.kind) {
        case PK_Node: t = "Node " + std::to_string(m_pick.node); break;
        case PK_Mid: t = "Midpoint"; break;
        case PK_Edge: t = "On beam"; break;
        case PK_Axis: t = m_pick.axis == 0 ? "On red axis (x)" : m_pick.axis == 1 ? "On green axis (y)" : "On blue axis (z)"; break;
        case PK_Ref: t = "On the reference"; break;
        default: break;
        }
        if (m_gfx_view_hover >= 0) {
            int kind, index;
            gfx_of_code(m_gfx_view_hover, kind, index);
            t = gfx_name(kind, index) + (m_gfx_pick == 6 || ImGui::GetIO().KeyAlt ? "   (click: select)" : "");
        }
        if (m_axis_lock >= 0) t += " [locked]";
        if (!m_vcb.empty()) t += (t.empty() ? "" : "   ") + m_vcb + "_";
        else if (m_op && m_tool != Tool::Tape && m_tool != Tool::Rotate && m_op_value != 0) t += (t.empty() ? "" : "   ") + format("%.3f", m_op_value);
        if (!t.empty()) {
            const ImVec2 ts = ImGui::CalcTextSize(t.c_str());
            const ImVec2 p(m_mouse.x + 16, m_mouse.y + 18);
            fg->AddRectFilled(V(p.x - 3, p.y - 2), V(p.x + ts.x + 3, p.y + ts.y + 2), IM_COL32(20, 22, 26, 210));
            fg->AddText(p, IM_COL32(255, 255, 255, 240), t.c_str());
        }
    }
}

} // namespace bl
