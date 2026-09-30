// The in-app model editor: builds and edits node / beam / triangle / wheel models (editor_model.h) on its own stage
// (the "editor" scene: a flat square, a neutral background), saves them as .truck files and spawns them for a test
// drive or a physics test without gravity (pull nodes with the mouse). Files: editor.cpp (the model, files, modes,
// the graphics preview), editor_input.cpp (views, camera, picking with inference, the tools), editor_draw.cpp (the
// frame), editor_ui.cpp (panels and icons).
//   Tools (SketchUp-like): Select, Line (beams, nodes made where a click lands off a node), Node, Rectangle, Circle,
//   Push/Pull, Move, Rotate, Scale, Tape measure, Eraser, Triangle, Shell, Shock, Rod, Wheel, Collision volume. Points snap to nodes,
//   beam midpoints and edges, the red / green / blue axes from the last point, the reference mesh and the work plane
//   of the view; a typed number (a length, "w,h", an angle, a factor) finishes the operation (Enter).
//   Camera: WASD / Q E fly (the 3D view) or pan (the others), right drag orbits, middle drag pans, the wheel zooms.
#pragma once

#include "game/camera.h"
#include "game/editor_model.h"
#include "gfx/mesh.h"
#include "gfx/renderer.h"

#include <algorithm>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

struct GLFWwindow;
struct ImFont;
struct ImDrawList;

namespace bl {

class Game;
class Vehicle;

class ModelEditor {
public:
    explicit ModelEditor(Game& g);
    ~ModelEditor();

    void open();                     // the editor's stage with the current model (a cart the first time)
    void open_vehicle(Vehicle* v);   // the editor with a copy of this vehicle (graphics included)
    void close();                    // back to the scene the editor was opened from
    bool active() const { return m_active; }
    bool driving() const { return m_active && m_mode == Mode::Drive; } // a test drive: the game takes the input

    void frame(GLFWwindow* win, float dt); // input, views, tools (before the game update)
    void render(Renderer& r, int fb_w, int fb_h); // the frame: the view(s) with the model
    void ui(ImFont* small);                // the panels
    void menu();                           // items of the main menu's Editor menu
    void toast(const std::string& s) { m_toast = s; m_toast_time = 3.0f; }
    void test_drive();               // saves the model and spawns it as the player's vehicle
    void test_physics();             // the model simulated without gravity: pull its nodes with the mouse

    enum class Icon {
        Select, Line, Node, Rect, Circle, PushPull, Move, Rotate, Scale, Tape, Erase, Tri, Shell, Shock, Rod, Wheel,
        Undo, Redo, Save, Drive, Physics, Close, Eye, EyeOff, Lock, Unlock, Plus, Trash, Edit, Copy, Quad, Grid, Floor,
        Symmetry, Snap, Frame, Chain, Pairs, Fill, Mirror, Connected, Grow, Invert, Hide, Show, Triangulate, Mesh, Link, Ids,
        Width, Blast, Shoot, Laser, Merge, Joint, Divide, FemTri, Volume, Count
    };

private:
    enum class Mode { Edit, Deform, Physics, Drive };
    enum class Tool { Select, Line, Node, Rect, Circle, PushPull, Move, Rotate, Scale, Tape, Erase, Tri, Shell, Shock, Rod, Wheel, Merge, Joint, FemTri, Volume, Count };
    enum class Elem { None, Beam, Shock, Hydro, Tri, Wheel, Joint };
    enum class Drag { None, Box, MoveFree, MoveAxis, Orbit, Pan, SplitX, SplitY, SplitXY, Grab, Erase, VolumePoint };
    struct View {                    // a view of the model: the perspective one orbits, the others are orthographic
        float yaw = 0.9f, pitch = 0.35f, dist = 7.0f;
        vec3 target{0, 0.6f, 0};
        int ortho = 0;               // 0 perspective, 1 front (along x), 2 side (along z), 3 top (along y)
        Camera cam;
        float rx = 0, ry = 0, rw = 1, rh = 1; // the view's rectangle (window coordinates)
        bool shown = true;
    };
    enum PickKind { PK_None, PK_Plane, PK_Node, PK_Axis, PK_Mid, PK_Edge, PK_Ref };
    struct Pick {                    // a point under the mouse, with what it snapped to
        int kind = PK_None;
        vec3 p{0, 0, 0};             // model space
        int node = -1;               // PK_Node
        int beam = -1;               // PK_Mid / PK_Edge: the beam
        float t = 0;                 // along it
        int axis = -1;               // PK_Axis: 0 x (red), 1 y (green), 2 z (blue)
        bool valid() const { return kind != PK_None; }
    };

    // ---- editor.cpp: model, history, files, modes, graphics preview
    void push_undo();
    void undo();
    void redo();
    void set_model(const edit::Model& m, const std::string& file);
    std::string folder() const;      // assets/vehicles/<home>
    std::string save_path() const;
    bool save(std::string* err = nullptr);
    bool load(const std::string& path);
    bool import_vehicle_file(const std::string& path);
    void enter_stage();              // loads the editor's scene
    void place_on_floor();           // m_origin: the model's lowest point on the floor
    void end_mode();                 // back to editing from a test
    bool alive(const Vehicle* v) const;
    std::string preview_text() const;
    Vehicle* spawn_from_model(bool player);
    void update_preview(float dt);
    void drop_preview();
    uint64_t preview_signature() const;
    // selection and edits
    std::vector<int>& sel_of(Elem k);
    const std::vector<int>& sel_of(Elem k) const { return const_cast<ModelEditor*>(this)->sel_of(k); }
    bool elem_selected(Elem k, int i) const { const auto& v = sel_of(k); return std::binary_search(v.begin(), v.end(), i); }
    void select_node(int n, bool add, bool toggle);
    void select_elem(Elem k, int i, bool add, bool toggle);
    void clear_selection();
    bool selection_empty() const;
    int selected_elements() const;
    void delete_selection();
    void duplicate_selection(vec3 offset);
    void mirror_selection();
    void connect_selection(bool all_pairs);
    void frame_selection();
    void select_connected();
    void select_invert();
    void select_grow();
    void select_by_preset(int group);
    // the selection filter: the kinds a selection holds (its triangles as shells, cab faces and FEM shells); only: keep
    // that kind alone, else drop it (Alt+1 - 9, Alt+Shift+1 - 9)
    enum class SelKind { Nodes, Beams, Shells, Cab, Shocks, Rods, Wheels, Joints, Fem, Count };
    int sel_count(SelKind k) const;
    static SelKind tri_sel_kind(const edit::Tri& t) { return t.fem ? SelKind::Fem : t.shell ? SelKind::Shells : SelKind::Cab; }
    static const char* sel_kind_name(SelKind k, bool many);
    void select_filter(SelKind k, bool only);
    void ui_selection_filter();
    void hide_selection();
    void unhide_all();
    void fill_selection();
    // merging: the selected nodes into one (at their centre, or a fixed one's place; with symmetry their twins into one
    // on the other side), the nodes closer than tol (the selection's, or all), a node into another (the Merge tool)
    void merge_selection();
    void merge_by_distance(float tol);
    void merge_into(int src, int dst);
    // frame elements' joints: a beam's at an end (0 a, 1 b: its own or its preset's), set (-1: its preset's; with
    // symmetry the twin beam's end at the mirrored node too); the joint tool's end under the mouse
    int beam_joint(int beam, int end) const;
    void set_beam_joint(int beam, int end, int type);
    bool joint_hover(int& beam, int& end) const;
    // the selected beams (and their twins) cut into n equal beams each (a frame element's joints stay at its ends)
    void divide_selected_beams(int n);
    // collision volumes (Model::volumes, the Volume tool): the active one (m_vol, -1: none), its hull (the game's
    // phys::convex_hull, cached), a point's mirror twin (-1: none, or on the plane), the nodes inside it; the edits
    struct VolumeHull {
        std::vector<vec3> pts;       // (the points it was made of)
        std::vector<vec4> planes;
        std::vector<std::vector<uint8_t>> faces;
        bool ok = false;
    };
    const VolumeHull& volume_hull(int v) const;
    int volume_point_twin(int v, int k) const;
    std::vector<int> volume_nodes_inside(int v) const;
    int active_volume() const { return m_vol >= 0 && m_vol < (int)m_model.volumes.size() ? m_vol : -1; }
    void volume_from_selection();                // a new volume: the selected nodes its anchors, a box inside them its hull
    void volume_box_from_selection(int v);       // its hull: the box of the selected nodes, inset
    void volume_points_from_selection(int v);    // its hull: the selected nodes' points (those on the hull kept)
    void volume_anchors_from_selection(int v, int how); // 0 set, 1 add, 2 remove
    void volume_toggle_anchor(int v, int node);  // (with symmetry the twin too)
    void volume_add_point(int v, vec3 p);        // (with symmetry the mirror too)
    void volume_remove_point(int v, int k);      // (and its twin)
    void volume_offset(int v, float d);          // each point moved d out from the centre
    void volume_mirror(int v);                   // the missing mirror points added: symmetric
    void volume_prune(int v);                    // the points inside the hull dropped
    void delete_volume(int v);
    void ui_volumes();
    void finish_merge(const std::vector<std::vector<int>>& groups, const std::vector<vec3>& at, const char* what);
    void set_view(int preset);       // 0 3D, 1 front, 2 side, 3 top
    bool node_shown(int n) const;    // visible and not hidden
    bool node_pickable(int n) const; // shown and its layer not locked
    bool elem_shown(Elem k, int i) const;
    bool elem_pickable(Elem k, int i) const;
    int add_node(vec3 p, bool twin); // (+ its mirror twin with symmetry); returns the node
    int node_from_pick(const Pick& pk); // an existing node, or a new one (splitting a beam when the pick was on it)
    int twin_or_self(int n) const;   // the mirror twin, the node itself on the plane, -1 without
    void add_beam_sym(int a, int b);
    void add_tri_sym(int a, int b, int c, int kind = 0); // 0 cab, 1 shell (sheet element), 2 FEM shell
    void triangulate_selection(int mode); // 0 beams, 1 cab triangles, 2 shell triangles, 3 FEM triangles
    int face_kind() const { return m_face_mode == 3 ? 2 : m_face_mode == 2 ? 1 : 0; } // (the new faces' add_tri_sym kind)
    void hull_selection();                // the convex hull of the selected nodes as collision hull triangles
    void move_selection(vec3 delta);
    void op_apply(const std::function<vec3(vec3)>& f); // move / rotate / scale: the start positions mapped by f
    std::vector<int> selection_with_twins() const;
    std::vector<int> selection_nodes_all() const; // the selected nodes and the nodes of the selected elements
    bool is_selected(int n) const;
    // building tools
    void make_rect(vec3 a, vec3 b, int view);
    void make_circle(vec3 c, vec3 r, int view);
    void pushpull_region(int tri, std::vector<int>& region, vec3& n) const;
    void pushpull_apply(float d);
    vec3 plane_normal(int view) const; // the drawing plane of a view (the 3D view: the ground, or the chosen plane)

    // ---- editor_input.cpp: views, camera, picking, tools
    void layout(float W, float H);   // the views' rectangles in the work area between the panels
    void update_cameras();
    int view_at(vec2 m) const;
    bool project(int view, vec3 world, vec2& screen) const;
    bool project(vec3 world, vec2& screen) const { return project(m_active_view, world, screen); }
    void mouse_ray(int view, vec3& ro, vec3& rd) const;
    vec3 to_world(vec3 model) const { return m_origin + model; }
    vec3 to_model(vec3 world) const { return world - m_origin; }
    void camera_input(float dt);
    void update_hover();
    Pick pick_point(bool has_anchor, vec3 anchor, bool nodes = true);
    vec3 snap(vec3 p) const;
    int gizmo_axis_hit(vec2 m) const;
    void tool_input();
    void tool_cancel();
    void set_tool(Tool t);
    bool vcb_active() const;         // a typed value would finish the operation in progress
    void vcb_input();
    void vcb_apply();
    void op_begin_nodes();           // move / rotate / scale: remember the nodes' start positions
    void op_restore();
    void op_revert();                // the model back to the snapshot the operation took (cancel)
    void op_move(vec3 delta);
    void op_rotate(float angle);     // about the view's plane normal through m_op_a
    void op_scale(vec3 f);           // about m_op_a
    float rotate_angle(vec3 p) const;
    void erase_hovered();
    void finish_picks();             // the triangle / shock / rod / wheel of the picked nodes
    void box_select(bool add, bool remove);
    void pan_view(View& v, vec2 md);
    vec3 gizmo_center() const;
    vec3 plane_point(int view, bool has_anchor, vec3 anchor) const;
    void gfx_pick_node(int n);       // editor_ui.cpp: a node clicked for the graphics binding
    // ---- editor_gfx.cpp: the graphics part by part. A part: kind 1 flexbody, 2 prop, 3 wheel (tyre and rim), 4 cab
    // submesh (index submeshes.size(): the editor's own triangles, the skin); its code in the preview's visual
    int gfx_code(int kind, int index) const;
    static void gfx_of_code(int code, int& kind, int& index);
    int gfx_count(int kind) const;
    bool gfx_hidden(int kind, int index) const;
    void gfx_set_hidden(int kind, int index, bool hidden);
    bool gfx_enabled(int kind, int index) const;
    std::string gfx_name(int kind, int index) const;
    std::vector<int> gfx_nodes(int kind, int index) const; // the nodes it is bound to (refs)
    void gfx_select(int kind, int index);
    void gfx_frame(int kind, int index);   // the views' cameras on it
    void apply_part_states(Vehicle* v);    // hidden, switched off, selected, hovered: the visual's part states
    bool gfx_view_input(bool click);       // Alt (or the pick button): the mesh under the mouse; true: the click is taken
    void ui_gfx_parts();
    void ui_gfx_selected();
    // only the nodes of the graphics shown (and the elements among them): m_gfx_mask, made each frame
    void update_gfx_mask();
    bool gfx_filtered(int n) const { return m_gfx_only && n >= 0 && n < (int)m_gfx_mask.size() && !m_gfx_mask[n]; }
    void set_gfx_only(int scope);
    void hotkeys();
    void physics_input(float dt);

    // ---- editor_draw.cpp
    void draw(Renderer& r, int view);
    void draw_grid(Renderer& r, int view);
    void draw_tool_preview(Renderer& r, int view);
    void draw_graphics_binding(Renderer& r, int view);
    void build_fill_mesh();

    // ---- editor_ui.cpp
    bool icon_button(const char* id, Icon icon, bool active, const char* tip, float size = 30.0f, bool enabled = true);
    bool action_row(Icon icon, const char* label, const char* shortcut, bool enabled = true);
    static void draw_icon(ImDrawList* dl, Icon icon, float x, float y, float s, uint32_t col);
    void ui_bar(ImFont* small);
    void ui_left(ImFont* small);
    void ui_tools();
    void ui_tool_options();
    void ui_presets();
    void ui_preset_window();
    void ui_utilities();
    void ui_right(ImFont* small);
    void ui_properties();
    void ui_transform();
    void ui_elements();
    void ui_layers();
    void ui_graphics();
    void ui_reference();
    void ui_vehicle();
    void ui_status(ImFont* small);
    void ui_labels();
    void ui_views();
    void ui_mode_banner();
    void set_mode_ui(int mode);      // the bar's mode switch: 0 edit, 1 deformation demo, 2 physics test, 3 test drive
    void ui_file_menu();
    void ui_view_popup();
    void ui_template_window();
    edit::Model make_template(int tpl) const; // (the New model window's templates, with its sizes)
    void ui_copy_window();
    void ui_physics_panel();
    void ui_structure();
    // ---- editor_deform.cpp: the deformation demo (the preview's nodes displaced by hand, the model unchanged)
    void enter_deform();
    void deform_input(float dt);
    void draw_deform(Renderer& r, int view);
    void deform_preset(int kind);    // 0 twist, 1 bend, 2 front crash, 3 dents
    vec3 demo_pos(int n) const;      // a node where the demo has it (world)
    void demo_push_undo();
    void demo_reset();
    void apply_demo_to_preview();
    void ui_deform_panel();
    // ---- editor_shellmat.cpp: the shell triangles' materials
    vec3 shell_preset_color(int preset) const;
    void apply_shell_preset_to_selection(int preset);
    void ui_shell_presets();
    // the FEM shells (Model::fem_presets): the list, its window, the colour a preset's triangles are drawn in
    void ui_fem_presets();
    void ui_fem_window();
    vec3 fem_preset_color(int preset) const;
    void apply_fem_preset_to_selection(int preset);
    void ui_shell_window();
    void apply_preset_to_selection(int group);
    void set_selection_layer(int layer);
    int preset_variant(int group, int type, bool hold);
    bool ui_frame_section(edit::BeamGroup& g); // (the preset window's frame element fields; true: changed)
    bool joint_combo(const char* id, int& joint);
    void ui_joint_palette(int& joint);
    static vec3 joint_color(int joint);        // (the marks of the joints at the frame elements' ends) // the preset with group's numbers and this type / hold
    std::vector<std::string> home_meshes() const;       // the mesh files of the model's folder
    // the reference mesh
    void ref_load(const std::string& path);
    void ref_update_mesh();
    mat4 ref_matrix() const;
    bool ref_hit(vec3 ro, vec3 rd, vec3& p) const; // world ray -> world point on the mesh
    vec3 ref_closest(vec3 p) const;                // world point -> nearest point on the mesh
    void snap_to_surface();

    Game& m_game;
    bool m_active = false;
    Mode m_mode = Mode::Edit;
    int m_scene = -1, m_prev_scene = -1;
    edit::Model m_model;
    std::vector<edit::Model> m_undo, m_redo;
    bool m_dirty = false;
    std::string m_file;              // where the model was saved / loaded
    std::vector<std::string> m_notes; // import / validation notes
    vec3 m_origin{0, 0, 0};          // the model's origin in the world
    View m_views[4];                 // 0 perspective, 1 front, 2 side, 3 top
    int m_active_view = 0;
    int m_single = 0;                // the view shown without the quad split
    bool m_quad = false;
    int m_drag_view = 0;             // the view a camera drag or a grab started in
    int m_split_hover = 0;           // the quad splitters under the mouse: 1 vertical, 2 horizontal, 3 both
    bool m_rclick = false;           // a right click (not a drag) this frame
    int m_axis_lock = -1;            // the inference locked to an axis (arrow keys)
    int m_maximized = -1;            // quad: one view blown up to the whole area (double click on its title)
    float m_split_x = 0.5f, m_split_y = 0.5f;
    float m_area[4] = {0, 0, 1, 1};  // the work area between the panels: x, y, w, h
    float m_left_w = 280.0f, m_right_w = 390.0f, m_top_h = 40.0f, m_status_h = 28.0f;
    CameraController m_saved_cam;
    bool m_saved_paused = false;
    // tests
    Vehicle* m_test = nullptr;       // the test drive's or the physics test's vehicle
    vec3 m_saved_gravity{0, -9.81f, 0};
    bool m_phys_gravity = false, m_saved_debug_beams = false;
    int m_phys_tool = 0;             // the physics test's tool: 0 grab, 1 destroy, 2 shoot, 3 laser (Game::tool)
    bool m_phys_held = false;        // (its button held since a click in a view)
    float m_fire_timer = 0;
    int m_saved_game_tool = 0;
    float m_phys_time = 1.0f, m_saved_time_scale = 1.0f; // the physics test's simulation speed (kTimeScales), the game's before
    void set_phys_time(float ts);
    // tools and interaction
    Tool m_tool = Tool::Select;
    std::vector<int> m_sel;          // selected nodes (sorted)
    std::vector<int> m_sel_beams, m_sel_shocks, m_sel_hydros, m_sel_tris, m_sel_wheels, m_sel_joints; // selected elements (sorted)
    std::vector<int> m_none;
    std::vector<char> m_hidden;      // nodes hidden for now (Ctrl+H), not saved
    int m_layer = 0;                 // the layer new elements go to
    int m_hover_node = -1;
    Elem m_hover_kind = Elem::None;
    int m_hover_elem = -1;
    int m_hover_axis = -1;
    int m_hover_tri = -1;            // the triangle under the mouse (push / pull)
    Pick m_pick;                     // the tool's point under the mouse
    int m_chain = -1, m_chain_start = -1; // line tool
    std::vector<int> m_picks;        // triangle / shell / shock / rod / wheel tools: the nodes so far
    bool m_op = false;               // rectangle / circle / move / rotate / scale / tape / push-pull: after the first click
    int m_op_view = 0;
    vec3 m_op_a{0, 0, 0}, m_op_b{0, 0, 0}; // the operation's first point (and the rotate reference)
    int m_op_stage = 0;              // rotate: 1 center picked, 2 reference picked
    float m_op_value = 0;            // the live value (a length, angle, factor, distance)
    std::vector<int> m_op_nodes, m_op_twins; // the selected nodes (and their mirror twins, -1 without)
    std::vector<vec3> m_op_pos0;
    bool m_op_pushed = false;
    std::vector<int> m_pp_region;    // push / pull: the face (coplanar triangles)
    vec3 m_pp_n{0, 1, 0};
    bool m_tape_done = false;
    vec3 m_tape_a{0, 0, 0}, m_tape_b{0, 0, 0};
    std::string m_vcb;               // a typed value
    Drag m_drag = Drag::None;
    vec2 m_drag_start, m_mouse, m_prev_mouse;
    int m_drag_axis = -1;
    vec3 m_drag_anchor;              // world point under the mouse at the drag start
    float m_drag_t0 = 0;
    std::vector<int> m_drag_nodes;
    std::vector<vec3> m_drag_pos0;
    bool m_drag_pushed = false;
    bool m_lmb = false, m_rmb = false, m_mmb = false;
    double m_last_click = 0;
    GLFWwindow* m_win = nullptr;
    // options
    bool m_symmetry = true, m_snap = true, m_show_ids = false, m_fill = true, m_grid = true, m_floor = true;
    float m_snap_size = 0.05f, m_work_y = 0.0f;
    int m_face_mode = 1;             // new faces (rectangle, circle, push / pull): 0 none, 1 cab triangles, 2 shells, 3 FEM
    bool m_rect_diagonals = true, m_circle_center = true, m_ref_snap = true, m_ref_show = true;
    float m_merge_tol = 0.01f;       // merge by distance: nodes closer than this (m)
    int m_joint_type = 1;            // the joint tool's joint (phys::FrameJoint)
    int m_divide_n = 2;              // divide the selected beams into this many
    // the Volume tool: the active volume, its selected point, the point under the mouse (its volume), the drag's twin
    // and start positions; the new volumes' inset from their nodes' box; the nodes inside the active one lit
    int m_vol = -1, m_vol_point = -1, m_vol_hover = -1, m_vol_hover_point = -1, m_vol_drag_twin = -1;
    vec3 m_vol_p0{0, 0, 0}, m_vol_twin_p0{0, 0, 0};
    float m_vol_inset = 0.08f;
    bool m_vol_show_inside = true;
    mutable std::vector<VolumeHull> m_vol_hulls;
    bool m_shell_beams = false;      // shell faces (rectangle, circle, push / pull) get beams too: off, the shells hold themselves
    int m_rect_div[2] = {1, 1};
    int m_circle_sides = 12;
    int m_plane_mode = 0;            // the 3D view's drawing plane: 0 ground, 1 side (xy), 2 front (yz)
    vec3 m_bg{0.30f, 0.34f, 0.40f};
    // how the model is drawn: the beams' width and the nodes' size (points on screen; [ ] and Shift+[ ] change them)
    float m_beam_px = 1.5f, m_node_px = 6.0f;
    float m_skel_alpha = 1.0f;       // the beams' and nodes' opacity (0: only the selected and the hovered drawn)
    vec3 m_move_by{0, 0, 0}, m_rotate_by{0, 0, 0};
    float m_scale_by = 1.0f;
    int m_group = 0;                 // the beam preset for new beams
    bool m_preset_window = false;
    int m_preset_edit = -1;
    bool m_preset_new = false;       // the preset window makes a new preset (m_preset_draft)
    bool m_template_window = false, m_copy_window = false;
    int m_shell_preset = 0;          // the material of new shell triangles (Model::shell_preset)
    bool m_shell_window = false, m_shell_new = false;
    int m_shell_edit = -1;
    int m_fem_preset = 0;            // the shell of new FEM triangles (Model::fem_preset)
    bool m_fem_window = false;
    int m_fem_edit = -1;
    edit::BeamGroup m_preset_draft;
    bool m_drag_pushed_ui = false;   // a panel's drag field took its undo snapshot
    bool m_show_binding = false;     // the Graphics tab is open: the bindings are drawn
    bool m_open_graphics_tab = false;
    // templates
    int m_tpl = 0;
    int m_tpl_n[3] = {3, 2, 2};
    vec3 m_tpl_size{2.0f, 1.0f, 1.0f};
    float m_tpl_mass = 400;
    float m_tpl_mm = 1.0f;           // the FEM templates' steel thickness
    // drawing of the filled triangles
    GpuMesh m_tri_mesh;
    int m_tri_mesh_count = -1;
    bool m_tri_mesh_ready = false;
    MaterialPtr m_tri_mat;
    // the graphics preview: the model spawned (not simulated) with its meshes, bound to the model's nodes
    Vehicle* m_preview = nullptr;
    uint64_t m_preview_sig = 0;
    float m_preview_wait = 0;
    bool m_show_gfx = true;
    float m_gfx_alpha = 0.55f;
    int m_gfx_kind = 0, m_gfx_sel = -1; // the selected mesh binding: 1 flexbody, 2 prop
    int m_gfx_pick = 0;              // the next clicked node: 1 ref, 2 x, 3 y of the binding, 4 into / 5 out of the forset; 6 a mesh
    int m_gfx_ui_hover = -1, m_gfx_view_hover = -1; // the part under the mouse in the list / in a view (codes)
    bool m_gfx_highlight = true;     // the selected part tinted and opaque
    float m_gfx_others = 0.6f;       // the other parts' opacity (times the graphics' opacity) while one is selected
    bool m_gfx_scroll = false;       // the list scrolls to the selected part
    char m_gfx_filter[64] = {};
    // the nodes shown: 0 all, 1 those the (shown) graphics are bound to, 2 those of the selected part; the elements
    // among them; and the nodes this many beams around them. Selected nodes and nodes made since stay shown
    int m_gfx_only = 0, m_gfx_only_grow = 0, m_gfx_only_base = 0, m_gfx_mask_count = 0;
    std::vector<char> m_gfx_mask;
    bool m_gfx_keep = true;          // re-binding keeps the mesh where it is (the offset follows)
    char m_mesh_buf[128] = {};
    std::string m_preview_error;
    // the deformation demo
    std::vector<vec3> m_demo_off, m_demo_off0; // per model node: the displacement (world metres), at the drag's start
    std::vector<std::vector<vec3>> m_demo_undo;
    std::vector<float> m_demo_w;     // the drag's weights
    std::vector<int> m_demo_grab;    // the dragged node (or a beam's two)
    std::vector<vec3> m_preview_rest; // the preview's body nodes where they were spawned (world)
    int m_demo_tool = 0;             // 0 drag, 1 push, 2 pull
    float m_demo_radius = 0.4f, m_demo_strength = 0.6f, m_demo_amount = 0.5f, m_demo_time = 0;
    int m_demo_hover_node = -1, m_demo_hover_beam = -1, m_demo_drag_view = 0;
    bool m_demo_drag = false, m_demo_wave = false, m_demo_rest_shape = true, m_demo_brush_ok = false;
    vec3 m_demo_anchor{0, 0, 0}, m_demo_plane_n{0, 0, 1}, m_demo_brush{0, 0, 0};
    float m_demo_saved_alpha[2] = {0.55f, 1.0f}; // (the meshes' and the skeleton's opacity before the demo)
    // the reference mesh (Model::ref_*)
    std::vector<vec3> m_ref_v;
    std::vector<uint32_t> m_ref_idx;
    std::string m_ref_loaded;        // the path the mesh was loaded from
    GpuMesh m_ref_mesh;
    MaterialPtr m_ref_mat;
    char m_ref_buf[256] = {};
    std::string m_toast;
    float m_toast_time = 0;
    std::string m_status;
    char m_title_buf[96] = {};
};

} // namespace bl
