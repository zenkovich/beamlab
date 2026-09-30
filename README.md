# BeamLab

A soft-body vehicle physics prototype in the style of **Rigs of Rods / BeamNG**, written from scratch in
C++20 with its own OpenGL 4.1 renderer and its own physics. It loads original **Rigs of Rods vehicle mods**
(`.truck/.car/.trailer` + OGRE meshes/materials) and simulates them as node/beam networks at 2 kHz, together
with a world of beam-based objects: breakable bridges, trees on orientation-preserving joints, crates, ropes.

Third-party code: GLFW (window/input), Dear ImGui (UI), [imgui_perfmon](https://github.com/zenkovich/imgui_perfmon)
(performance widget), stb_image / stb_image_write. No game or physics engine is used.

## Build and run

Requirements: macOS (Apple Silicon or Intel), Xcode command line tools, CMake 3.20+, Ninja, Python 3.

```bash
python3 tools/fetch_vehicles.py          # downloads the 22 vehicle mods into assets/vehicles (once)
python3 tools/fetch_rbr_stage.py         # optional: downloads + converts 4 Richard Burns Rally stages (~1.5 GB, once)
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build --target beamlab
./build/beamlab
```

The RBR stages are community stages by RALLY Guru (https://rallyguru-tracks.blogspot.com/), free for non-commercial
use. `tools/fetch_rbr_stage.py` downloads them from the author's Google Drive and converts them locally into
`assets/stages/<name>/` (vegetation textures reduced to 1024 px as the author asks, credits in `SOURCE.txt`). The
converted stages are not redistributable and `tools/package_macos.sh` leaves them out.

VS Code: the single launch configuration **BeamLab** (`.vscode/launch.json`) builds Release (the pre-launch task
also runs the vehicle download, which skips vehicles that already exist) and starts the app under the debugger.

To send a build to someone else: `tools/package_macos.sh` builds a universal (Apple Silicon + Intel, macOS 11+)
`BeamLab.app` without host-specific CPU tuning, packages only the vehicle files the app actually reads (it loads every
vehicle once with `BL_TRACE_FILES`), signs it ad hoc and writes `dist/BeamLab-macOS.zip` (~270 MB) with a README for the
recipient.

Useful command line options (`./build/beamlab --help`):

| option | meaning |
|---|---|
| `--scene <id>` / `--vehicle <folder/file>` | start scene and vehicle (`--list` prints both) |
| `--bench <frames>` | benchmark at a fixed 60 Hz frame step, prints physics timings per zone |
| `--screenshot <png> --frames N --camera <preset>` | render N frames, save a screenshot, exit (`chase`, `side`, `orbit:yaw,pitch,dist`, `look:ex,ey,ez,tx,ty,tz`) |
| `--spawn x,z,yaw`, `--drive thr,steer[,brake]` | scripted spawn point and constant driver input with telemetry |
| `--perf`, `--hidden`, `--threads N` | open the performance widget, hidden window, thread count |
| `--shoot kind,speed,interval`, `--destroy radius` | scripted tools: fire projectiles / destroy along the camera ray |
| `--laser x0,y0,x1,y1[,first frame,frames]` | scripted laser sweep across the screen (fractions, y down), prints the links cut |
| `--action <text>`, `--launch <km/h>` | run a Scene menu action (e.g. `Autopilot`; several: `text@frame; text@frame`, at frame 5 by default), launch the vehicle at a speed |
| `--camera glook:ex,ez,eh,tx,tz,th` | free camera with heights above the ground |
| `--editor`, `--editor-test` | open the model editor after the load; `-test` also saves its model and starts a test drive (the scene test) |
| `--record dir[,every[,first]]`, `--noui` | save every n-th frame from `first` as `dir/frame_00000.png` ... (a video: `ffmpeg -framerate 30 -i dir/frame_%05d.png out.mp4`); no menus, labels or panels |
| `--crane lift[,frame[,roll,pitch]]` | hang the player vehicle `lift` m up by the top of its frame (its highest nodes), tilted (degrees), let go at that frame (-1: never): the suspension's droop; `BL_SHOCKDBG=<frame>` prints the wheels' place, camber and toe in the body's frame |
| `--shots <file>` | screenshot series in one run: one shot per line, `<frame> <camera preset> <png> [beams] [noui] [pause]` (`beams`: wireframe on top, `noui`: no menus or labels, `pause`: physics stops from that frame) |

Diagnostics: `BL_DRIVEDBG=1` prints the player's engine, clutch and the torque reaching each wheel twice a second; `BL_WELDDBG=1` prints each weld that lets go (which, how far it was pulled, the force). The status line of a scripted run (`--action`, `--drive`) gives the panels' shake on their welds: the welded nodes' centre against the frame's point, as its speed (rms and most) and as it moves from one frame to the next (what the eye sees); `BL_MOVEDBG=<m/s>` names each weld that moves faster, `BL_JITTERDBG=1` the ones whose speed passes 0.25 m/s and the membrane's edges out of their band. `BL_HULL=0` leaves the hull triangles out of the collisions (the two-car tests still measure how far one frame got into the other's hull: the Scene status line). `BL_SHOCKDBG=<frame>` prints every shock of every body at that frame (length against its rest length and
bounds, spring and stop stiffness); `BL_SHOCKLOG=1` prints the player's springs every frame, each as the share of its stroke from the short bound (0) to the long (100; below 0: in its bump stop); `BL_EDITOR_GRAVITY=1`, `BL_EDITOR_SPEED=<x>` and `BL_EDITOR_CRANE=<m>` set up the model
editor's physics test (with `BL_EDITOR_VEHICLE` / `BL_EDITOR_FILE` and `BL_EDITOR_PHYSICS`); `BL_SHELLDBG=<frames>` prints the triangle-element sheets every N frames (triangles per refinement level,
refinements, cracks, pieces, total area, a closed sheet's volume and centre, the fastest awake node with its stability
budget); `BL_BARREL_SEG=<n>` (segments round a barrel's wall), `BL_BARREL_SHIFT=<0-2>` (its short steps),
`BL_BARREL_MEMBRANE=<N/m>` (0: springs only), `BL_BARREL_BENDYIELD`, `BL_BARREL_BEND`, `BL_BARREL_DISH=<m>`,
`BL_BARREL_IMP=<m>`, `BL_BARREL_SLOP=<m>`, `BL_BARREL_PUSHMAX=<m/s>`, `BL_BARREL_REST=<1/s>`, `BL_BARREL_RESTVIB=<1/s>`,
`BL_BARREL_ROLLRES=<m/s2>`, `BL_BARREL_RESTFRIC=<x>`, `BL_BARREL_HARDEN`, `BL_BARREL_HARDMAX`, `BL_BARREL_SPHERES=1` (the
drums meet each other as spheres), `BL_NOGUARD=1` (no energy guard on the drums with rings; `BL_GUARDDBG=1` prints its
cuts), `BL_NOCREASE=1` (the drums shaded smooth across their edges), `BL_BARREL_FEM=1` (the scenarios with frame rings) vary the Steel Barrels; `BL_MEMBRANE_ITERS=<n>` (2) sets the membrane projection's sweeps,
`BL_SPHERE_OUT=<m/s>` (0.2) how fast sphere contacts push an overlap apart (`BL_SPHERE_PUSHV=1`: with velocity);
`BL_CAR_SHIFT=<0-2>` (the welded car's sheet's short steps), `BL_AXEDBG=1` / `BL_LASERDBG=1` (the axe's cut, the members a
cut crosses); `BL_ENERGYDBG=<body>` prints a body's energy every frame (its motion as a whole, the rest, its height; its rest state:
resting, speed as a whole, contacts, sleep and rest timers);
`BL_EDITOR_BARREL=r,h,sides[,y]` has the model editor draw a circle of sheet (at height y) and pull it into a drum (with
`BL_EDITOR_PHYSICS=1 BL_EDITOR_GRAVITY=1`: dropped); `BL_REPAIRDBG=1` prints
every node the repair resets (its triangles, budget, position, velocity, force); `BL_NODEDBG=a,b,c` prints those nodes
of the player's vehicle every frame (mass, position, velocity); `BL_LIFTDBG=1` prints the node that sets a vehicle's
spawn height; `BL_SHEET_MAXLEVEL=<n>` overrides the refinement depth of every sheet (scenes and vehicle bodies); `BL_BPDBG=1` prints slow broadphase rebuilds with their parts; `BL_FASTDBG=1`
prints every fast-body pair search (fast bodies, triangles in reach, grid cells and node tests, pairs, time);
`BL_CACHEREPORT=<frame>` prints the cache model of the sheet force kernel (see below) for the biggest sheet at that
frame.

Tests (no window, no GPU): `cmake --build build --target test_physics && ./build/test_physics` checks the sheet force
kernel against a frozen copy of the original one (forces, plastic state, strains, overload events on intact, refined,
cracked and heavily deformed sheets of four materials), momentum and angular momentum, coarsening (uniform
refinements of depth 1 - 3 back to the authored triangles; random refinements, cracks, a laid glass pattern, a
clamped border: area and mass, topology, the kernel, refining again), rigid pieces (momentum and angular momentum
kept at make_rigid, a spinning shard in free flight falls like a point mass and keeps its shape, glass shattered
over a floor: every piece rigid, none through the ground, half asleep in 3 s), the topology invariants after
random refinements and cracks, the Morton renumbering, a second ball refining the sheet like the first, sheets of
seven shapes and sizes (rectangle 3.6 x 2.4 m, disc, ring, triangle, L, half-pipe, dome: mesh area against the exact
one, masses, kernel, topology, two balls through each), the laser (a full cut, a slit, a hanging sheet cut in two, a
ball body, 12 random cuts on three materials, cuts along the mesh's grid lines, a sweep in 40 steps across a gate,
shapes: the parts lie on their own sides, area kept, every free edge the border or the cut), the fracture patterns (a
glass web laid: refined, nodes on the lines, codes the same on both sides of every edge, the kernel as the reference;
balls into glass, steel and plywood: the cracks on the lines / along the grain against the same impact without the
pattern, one pattern per ball, area, topology, single- and multi-threaded runs identical), the stability of a finely
refined sheet, and a ball through a sheet in the world (cracks, the ball passes, area kept, single- and multi-threaded
runs bitwise identical), and the FEM frame elements (see Physics). `./build/test_physics only <section>` runs one
section (kernel, momentum, topology, reorder, repeat, shapes, shape_impacts, patterns, laser, stability, world, frame).
`./build/test_physics bench` adds the kernel timing and the cache model, `kernel [s]` loops the kernel alone (for
sampling profilers), `team` measures the cost of a parallel phase. `python3 tools/test_scenes.py [--quick]` runs the
scenes through the application: the car through the three lead sheets (each cracks, pieces fall off, area kept), the
Materials Lab (glass shatters, rubber holds, the rest crack), cannonballs into the steel sheet, the Sheet Shapes (the car
through the 6 x 4 m gate and the dome, cannonballs through each panel, a laser cut across the gate; area kept), no
numerical warnings in any scene, and the autopilot's time on the Rally Stage.

Profiling: `BL_PROFCSV=<file>` writes one line per frame (the physics zones in CPU ms summed over threads, the heaviest
island of the frame, triangle / beam evaluations, refinements, cracks, pieces, pair tests, pair rebuilds, substeps,
vehicle speed and position). `--realtime` makes screenshot / benchmark runs step by the real frame time (and wait for
1/60 s like vsync) instead of a fixed 1/60 s, which shows how a slow frame makes the next one simulate more substeps.
`BL_SHEET_MAXLEVEL=<n>` overrides the refinement depth of all sheets. The performance widget (F2) breaks the physics
down into CPU time per kind of work (summed over threads: beam forces, sheet elements, the sheets' per-node gather,
narrow phase, contact response, broadphase and fast-body pairs, pairs inherited after cracks, static collisions,
integration, sheet refinement / cracks / detaching / renumbering, sheet meshes) with per-frame counters, and shows the
heaviest island (the frame's critical path) with the wall time of its phases (forces, gather, collisions, integration,
serial merges, topology), how long its owner waited for helpers at phase ends, and the cache model of the sheet
kernel ("Measure cache": L1 hit rate, bytes brought from L2 per triangle, share of every fetched cache line actually
used, working set; the kernel against the original data layout). The profiling CSV has the same columns.
Its last columns are the widget's element counts (`bodies`, `bodies_awake`, `pieces`, `pieces_awake`, `nodes`, `nodes_awake`, `beams`, `beams_awake`, `beams_broken`, `shells`, `shells_awake`, `shells_L0` .. `shells_L4`,
`shells_x1` / `x2` / `x4`, `hinges`, `hinges_awake`, `edges_border` / `crack` / `cut`, `tris`, `tris_awake`, `impacts`,
`node_steps`, `shell_evals`, `hinge_evals`, `beam_evals`, `contact_pairs`); `shell_steps` is the kernel's own count of
the triangles it evaluated.

## Controls

| input | action |
|---|---|
| W / S, A / D | throttle / brake (hold S at a stop to reverse), steer |
| Space | handbrake |
| 1 .. 0 | vehicle commands (cranes, doors, tippers) |
| R / Shift+R | recover the vehicle on the spot / respawn at the scene start |
| C / F | camera: chase, orbit, cockpit, free / free camera on-off (free: WASD + Q/E, Shift = faster; it starts from the current view) |
| Tab | control the next vehicle of the scene |
| right mouse drag, wheel | look around, zoom |
| G / X / B / L | tool: **grab** (drag a node) / **destroy** (hold LMB: breaks beams, joints and surfaces under the cursor) / **shoot** (LMB, hold for auto-fire) / **laser** (hold LMB and sweep: cuts along the cursor's path, no blast) |
| Z, `,` / `.`, Delete | projectile type (steel ball, rubber ball, crate, cannonball, plank; the balls one collision sphere each - a node and a capsule of no length, `build_ball`, bouncing by `SoftBody::bounce` - no beams), speed 5 .. 150 m/s, remove projectiles |
| `[` / `]` (keypad - / +) | slower / faster time: 0.01x, 0.02x, 0.05x, 0.1x, 0.25x, 0.5x, 1x, 2x |
| T / Backspace | slow motion 0.2x on-off / back to 1x |
| P / N | pause / single step |
| F5 | reload the scene (reruns the crash test); the camera stays where it is |
| Ctrl+E | the model editor (its own keys are listed under Model editor below) |
| F1 / F2 / F3 / F4 / H | help / performance widget / beam view / hide meshes / HUD |
| gamepad | left stick steer, triggers throttle/brake, A handbrake |

## UI

Everything is ImGui. The menu bar at the top of the screen holds:

- the **World** and **Vehicle** dropdowns, menus of submenus. World: Driving, Rally, Crash, Test cars (the Frame Car's
  and the Buggy's tests in submenus of their own), Physics lab, Stress, Tools. Vehicle: Cars, SUVs / vans / pickups,
  Off-road, Trucks, Buses, BeamLab test cars, Editor models, Trailers and loads; a vehicle of several variants a
  submenu of them. Picking an item switches immediately, with no dialog.
- **Reset**, **Pause / Step** and the simulation speed.
- the **View**, **Physics** and **Scene** menus:
  - **View**: camera, debug views (beams coloured by stress, nodes and frames, collision geometry, wheels, awake/asleep bodies), lighting.
  - **Physics**: gravity, wind, tyre grip, traction assist, steering response (steering speed, return to centre,
    how much slower it gets at speed; presets Rigs of Rods / Sharp / Very sharp), solver switches, deformation and
    breaking for the player vehicle; **Sheets**: overrides of every sheet's material for experiments (refinement
    depth, minimum triangle edge, minimum piece), the refinement quota per frame, coarsening (on/off, the quiet time
    before it, the strain below which triangles merge, merges per frame), rigid pieces, fracture patterns
    (`WorldSettings`).
  - **Scene**: drop crates, jelly balls, metal blocks, planks; add AI vehicles; launch the vehicle at a wall; send a heavy truck over the bridge;
    in the *Vehicle vs Vehicle* scene: the crash setup (vehicle A and B, their speeds, layout, slow motion).
  - **Tools**: grab / destroy / shoot / laser, grab strength (0.05x - 50x) and radius (a sphere round the picked node: every body's nodes in it pulled together, keeping their places, the pull falling off to its edge; drawn under the cursor and while pulling; 0: one node), destroy radius and blast, projectile type, speed and fire rate, laser range.

- **Editor**: the model editor (Ctrl+E), below.

The tool panel at the top left switches the mouse tool. The HUD shows a speedometer, a tachometer with the gear, throttle and brake bars, and damage; View > HUD (H) shows or hides it.

### Model editor

**Editor > Model editor** (Ctrl+E) builds and edits any node / beam / triangle / wheel model on a stage of its own
(the *Model Editor* scene: an empty floor under a plain sky with a horizon; Close goes back to the scene it was
opened from). A model is what a `.truck` file holds (`src/game/editor_model.h`): nodes, beams in presets, shock
absorbers, steering rods (hydros), orientation joints, cab triangles (collision surfaces) and shell triangles
(elements of a sheet body), wheels, the graphics bound to the nodes, the masses, the drivetrain and the cameras.

- **Layout**: square-cornered panels docked to the window's edges (the left and right ones resized by dragging their
  inner edge), the views between them, a one-line status bar. The top bar holds the model's name, **File** (save, new
  from a template, open a model, copy a vehicle, close), undo / redo / save, the **mode** (Edit, Deform, Physics,
  Drive), symmetry and snap, the views (**Quad** and the view: 1 - 4) and **View** (the grid, the floor, filled faces,
  node numbers, the beams' width and the nodes' size and opacity with Thin / Normal / Bold / Heavy, only the graphics'
  nodes, the meshes' opacity, the snap step, the work height), the graphics' eye and Close. The left panel follows the
  mode: the tools, their options (only when a tool has some), the beam presets and the utilities while editing; the
  demo's or the physics test's controls otherwise. The right panel has four tabs: **Properties** (the selection, in a
  label / value grid; nothing selected: the model at a glance and its checks), **Structure** (layers and groups, all
  the elements, the reference mesh), **Graphics** and **Vehicle**. In the quad views the dividers are dragged (a
  double click evens them out) and a view's title blows it up; the flat views show the model over a grid, the 3D view
  the floor and the horizon.
- **Beams and nodes on screen** (View): the width 1 - 10 px, the size 2 - 20 px, the opacity; `[` / `]` step the width,
  Shift+`[` / `]` the size. The selected and hovered beams are drawn wider, held ends, shocks, rods and joints a little
  wider; the physics test draws its beams and nodes the same way (strips facing the camera, `Renderer::thick_line`).
- **Camera**: WASD fly the 3D view and pan the flat ones (Shift: faster; Q / E down / up while the right button is
  held), right drag orbits (flat views: pans), middle drag pans, the wheel zooms toward the mouse, F frames the
  selection, Home everything.
- **Tools** (left, icons; SketchUp-like): *Select* (Space: click, Shift adds, Ctrl toggles; a box left to right takes
  what is inside, right to left what it touches; drag a selected node or a gizmo arrow to move; double click: the
  connected nodes, a whole face; the **selection filter**: Properties lists what is selected by kind (nodes, beams,
  shells, cab triangles, shocks, rods, wheels, joints, FEM triangles), *Only* keeps one kind, x deselects it; Alt+1 - 9
  and Alt+Shift+1 - 9 do the same), *Line* (L: beams from click to click, a node made where a click lands off a node, on
  a beam the beam is split; the start closes a loop), *Node* (N: in any of the four views: the 3D and top views put it
  on the work plane, the front and side views at the depth of the selected node, the new node is selected),
  *Rectangle* (R: divisions, diagonal beams, cab or shell faces, in the view's plane), *Circle* (C: sides, a centre
  with spokes, faces), *Push / Pull* (P: a face of coplanar triangles extruded into a braced box; shell faces make shells
  only: a sheet holds its shape itself, beams along it only with "Beams too"; the rectangle and the circle likewise, and
  the preview and the physics test write no cockpit node, whose eight beams would lie along a flat shape's edges), *Move* (M; Ctrl at
  the first click moves a copy), *Rotate* (O: centre, reference, angle; 15 degree steps with snap), *Scale* (K; an
  arrow key: one axis), *Tape measure* (U), *Merge* (a node, then the node it goes into: its elements go along, with symmetry the twins too), *Eraser* (E:
  click or drag), *Triangle* (T), *Shell* (Y), *FEM triangle* (Q: three nodes, a triangle element of the FEM frame of
  the FEM shell chosen in the options; Rectangle, Circle and Push / Pull make them with Faces: FEM triangles), *Shock* (J),
  *Steering rod* (H), *Wheel* (B), *Joint* (frame elements: a click near a member's end joins it to its node with
  the joint chosen in the options, welded / ball / hinge_v / hinge_h / swivel / elastic; Shift+click: its preset's
  joint again; the ends are drawn as dots in the joint's colour), *Collision volume* (Z: `editor_volumes.cpp`; its
  options list the model's volumes - New makes one of the selected nodes, its anchors, with a box inset inside them -
  and the active one's name, break rms, anchors (= / + / - the selection, select them) and hull (a box round the
  selection, the selected nodes' points, shrink / grow 1 cm, mirror, the points inside dropped, each point's
  coordinates); in the views its points are dragged with the snapping and the inference (with symmetry the mirror
  point too), Ctrl+click adds one, Delete takes the selected off, Shift+click on a node makes it an anchor or no more;
  the hull is drawn as the game builds it (phys::convex_hull), its anchors ringed, the nodes inside it marked - a
  part's node in it would not be held off; the other tools show the volumes faint). Points snap to nodes, beam midpoints and edges, the red / green / blue axes from the
  last point (arrow keys lock one: right x, up y, left z, down frees), the reference mesh and the view's plane; the
  inference shows at the cursor. A typed value finishes the operation (Enter): a length, `w,h`, `dx,dy,dz`, an angle,
  a factor. **Symmetry** (X) mirrors every edit across z = 0 onto the twin nodes; **Snap** (G) and its step.
- **Beam presets** (left, a scrolling list): the checked one is used by new beams; the preset window (+, the pencil or
  a double click) sets the name, the colour, the **type** (normal, rope: pulls only, support: pushes only, **frame
  element**: a real tube or bar, below), the **ends** (free: a pin, the beam turns about its nodes; held: each end
  keeps its angle to the structure, written as a pair of joints, with its stiffness), the spring, damping,
  deformation and break forces (log sliders, "never") and the plastic coefficient, with the node stability figure
  (k dt^2 / m). A **frame element** preset has a section instead: material (steel, chromoly, aluminium, titanium,
  carbon, wood), shape (round tube, square box section, solid rod or bar), diameter or side and wall, the joints at its
  ends (welded, the default; ball, hinges, swivel, elastic with its stiffness), with its mass per metre, EI and EA, a 0.5 m cantilever's stiffness and the
  loads at which it yields; new models have "Steel tube 40x2". To turn a model of held beams (joints) into a frame:
  set its preset's type to frame element. "Apply" moves the selection's beams to
  the checked preset; the Properties tab changes their type or ends directly (a preset variant is made).
- **FEM shells** (left, under Shell materials): the material and thickness of FEM triangles (`set_fem_shell`): the
  list with each one's colour and count (its area and mass in the tooltip), + from ready-made ones (body steel 0.8 / 1
  mm, floor steel 1.5 mm, reinforcement 2.5 mm, aluminium, carbon, plastic) or a copy, the window (name, frame material
  with its modulus, yield and elongation, thickness with its kg/m2, colour), delete, "Apply to selection" (the selected
  triangles and the triangles among selected nodes become FEM triangles of it). A triangle's Kind in Properties has
  "FEM shell" and its shell; the utilities cover a selection with FEM triangles; the physics test shows the triangles'
  peak load and how many yielded or tore. File > New: FEM plate and FEM box (hollow, n x n cells a face) of a steel
  thickness.
- **Utilities** (left, a row each with its icon and key): beams along the selection (Ctrl+B) or between all pairs
  (Ctrl+Shift+B), a face over 3 - 4 nodes (Ctrl+F), triangulate the selection with beams, cab triangles or shells, a
  collision hull round the selection (the convex hull of the selected nodes as hull triangles: select a frame's outer
  corners; a triangle's Kind in Properties is cab / shell / hull, hull triangles are amber in the views), mirror
  (Ctrl+M), duplicate and move (Ctrl+D), delete, merge the selected nodes into one (Ctrl+J: at their centre, a fixed
  one stays put; with symmetry their twins on the other side too), merge the nodes closer than a distance (a weld of the
  selection or of everything; the distance in the Merge tool's options), select connected / grow / invert, frame,
  hide / show all, snap to the grid, onto the reference surface. A merge moves the elements of the merged nodes to the
  one left (`edit::Model::merge_nodes`); beams, triangles and joints that fold up and the doubles go.
- **Right panel**: *Properties* (nodes: position, flags, own mass, group, layer, the meshes bound to them, a numeric
  transform; elements: preset, type, ends, a shock's spring / damping / bounds / stops, a rod's factor and spring, a
  joint's stiffness and break force, triangles cab or shell, collision, flip; a wheel's kind, sizes, rays, mass, tyre,
  rim, arm, brake, drive, its beams and grip), *Structure* (layers shown / hidden, editable / locked, new elements'
  layer; node groups; every element in lists, clicking selects; the reference mesh: a Wavefront `.obj` mockup with its
  opacity, offset, scale, yaw and snapping), *Graphics*, *Vehicle* (masses, the sheet material of the shells, the
  drivetrain, the camera nodes; *Cockpit node*: a cinecam node above the centre on eight beams to the nearest nodes,
  off unless ticked, since it is a node and beams of its own that take a share of the dry mass: it used to be written
  for every model; a copy of a vehicle with a cinecam keeps it, an older editor file reads back without it). File > "Copy a vehicle" and File > "New from a template" (cart, box, plate, sheet,
  cylinder, empty, FEM plate, FEM box) open windows of their own; the checks are in Properties with nothing selected.
- **Graphics**: a copy of any vehicle (File > "Copy a vehicle", or the game's Editor menu > "Edit the current vehicle")
  keeps its graphics: the flexbodies, the props, the cab submeshes with their texture coordinates, the managed
  materials, the mesh wheels; the model is saved in the vehicle's own folder (where its meshes are) under a name of
  its own, never over its file. The model is drawn with its meshes see-through (the eye and the opacity slider on the
  bar), rebuilt a moment after a change and moving with the nodes while they are dragged. How a mesh follows the
  nodes:
  - a *flexbody* is placed in the frame of three nodes (ref: the origin; x, y: the axes; the offset along ref->x and
    ref->y in multiples of those distances and along their normal, then the rotation); each vertex is then tied to the
    three nearest well-spread nodes of its *forset* and keeps its place in their frame, so the mesh bends and crumples
    with them (fewer than three: it moves rigidly);
  - a *prop* is rigid in the frame of its ref, x and y nodes; a mesh wheel's rim turns with its axle, the tyre is
    drawn over the wheel's own nodes; a cab submesh is the cab triangles, textured through their nodes.

  The *Graphics* tab lists the parts by kind (flexbodies, props, wheels, cab submeshes) with a filter: the eye hides a
  part in the editor (a view setting, kept as a comment), the box switches a flexbody or prop out of the vehicle (kept
  in the file as a comment, drawn faint here, back on at any time); Show all / Hide all / Solo / Invert, an eye per
  kind. A click selects a part (a double click puts the views on it), the list's row under the mouse lights the part,
  **Alt** + the mouse over a mesh in a view lights it and names it, Alt + click selects it (or "Pick a mesh in a
  view"); Alt+H hides / shows the selected, Alt+Shift+H shows all. The selected part is tinted and opaque, the others
  at a lower opacity; the node-beam model's opacity can be turned down to nothing (the selected and hovered stay).
  **Nodes: only the graphics'** (Alt+G, or the Display popup) shows only the nodes the shown meshes are bound to
  (forsets, ref / x / y, a wheel's axle, a submesh's triangles) and the elements among them, or only the selected
  part's, plus the nodes 1 - 4 beams around them if asked; the selected nodes and the nodes made since stay shown, F
  frames what is shown, and hidden nodes cannot be picked or box-selected. The
  selected flexbody or prop shows its frame in the views (ref yellow, x red, y green, the mesh's origin and axes, the
  forset nodes as white squares) and edits the mesh file, the ref / x / y nodes (typed, picked in a view, or the
  selected node; re-binding keeps the mesh in place), the offset (or onto the selection's centre) and rotation, the
  forset (ranges, set / add / remove the selection, add or remove nodes by clicking); duplicate, mirrored copy (on the
  twin nodes), delete; a new flexbody or prop takes a mesh of the folder on the selected nodes. A wheel shows its
  meshes, a cab submesh its triangles. The Properties tab lists the meshes bound to the selected nodes. (The editor's
  preview builds its visual part by part, `VehicleVisual::split_parts`, and sets each part's state: normal, hidden,
  selected, hovered, faint; `VehicleVisual::pick` finds the part under a ray.)
- **Deformation demo** (the Deform mode): the model's meshes shown opaque over a faint skeleton; drag a node or a beam
  (the nodes around it follow with a smooth falloff of a chosen radius), or hold the Push / Pull brush over the meshes;
  presets twist, bend, crash the front, dent the model or run a wave through it (the amount slider). The meshes follow
  the displaced nodes as they would in the game (a flexbody's vertices on their forset, a prop on its ref / x / y, a
  wheel's tyre and rim on its axle); the moved nodes turn orange, the rest shape stays faint. Ctrl+Z steps back, R
  resets, Esc goes back to editing; the model itself is never changed.
- **Shell materials** (left panel): what the shell triangles are made of. The first is the model's sheet material (the
  globals' `sheet/...`), more are added from ready-made ones (steel panel 0.8 mm, aluminium panel, window glass,
  plastic bumper, plywood, rubber, soft top) or copied; each has its material (Steel, Aluminium, Lead, Glass, Acrylic,
  Plywood, Rubber, Cardboard, Fabric: its stiffness, ductility, strength, bending), its real thickness (the areal mass),
  its drawn thickness, its colour (or the material's own look; metals take the vehicle's paint) and its refinement
  depth. New shells (the Shell tool, shell faces of the rectangle and the circle) take the checked one; "Apply to
  selection" makes the selected triangles shells of it, the Properties tab changes the selected shells' material; the
  wires are drawn in its colour. In the file: `set_shell_material name, Material, kg/m2, drawn thickness, r, g, b,
  refinement depth` in the shells section before the triangles of that material. In the physics each triangle keeps
  its material (`phys::Shell::mat`, `SoftBody::shell_mat_extra`: its springs, mass, yield, plastic bend, fracture and
  bend limits, refinement thresholds, tension-only); the fracture pattern, crack rate and smallest piece stay the
  body's. A vehicle's metal sheets are tuned ductile (they dent and keep it), glass and plastics keep their
  brittleness (a window shatters), divide the selected beams into 2 - 8 (a frame element gets nodes to bend, dent and
  be hit between its joints; its joints stay at its ends).
- **Welds** (file only): `welds` lines `anchor, sheet node, radius m, strength N[, stiffness N/m[, anchor2, t]]` hold a
  sheet on a frame node (or on the point t of the way along the member to anchor2) at a point: the anchor holds the
  weighted centre of the sheet's nodes within the radius (reached across its triangles, weights (1 - d / radius)^2)
  at its rest offset, turned with the anchor's frame node (the pull's moment turns the node back); past the strength
  it lets go. A vehicle whose panels are
  welded has their plane held by the membrane projection (steel's and glass's strength).
- **Hull triangles**: cab option `h` (with `c`): a one-sided, solid collision triangle facing out (its winding), not
  drawn on a sheet body; see Inter-body collisions. View > Collision geometry draws them amber with a tick along the
  outward normal (torn ones are no longer drawn).
- **Tests**: **Physics test** (Ctrl+P) simulates the model where it stands without gravity (G toggles it, Space
  pauses, R restarts): use the game's tools in any view (1 - 4: grab a node, destroy by sweeping, shoot projectiles, cut with the laser; their radius, projectile, speed, rate, range in the left panel; the grab's strength 0.05x - 50x with 0.2x / 1x / 5x buttons, or the mouse wheel while pulling, and its radius); the speed of the simulation from 0.01x to 2x (the slider, - and =, Backspace: real time; paused, Step or N moves on by 10 ms), kept from one test to the next, the game's own speed back afterwards and watch it flex, the beams drawn through the
  see-through meshes (without the joints' frame axes, which looked like stray beams), frame elements in steel blue
  (orange once bent for good) with the frame's peak load against its yield; Esc comes back to editing. **Test drive** (Ctrl+T) saves the model and drives it on the stage;
  Esc comes back.
- **Save** writes `assets/vehicles/<folder>/<name>.truck` (the editor's own models: `assets/vehicles/editor`) and
  rescans the vehicle registry: the model is in the Vehicle list. The file is a plain Rigs of Rods truck (globals,
  minimass, nodes, beams with set_beam_defaults, shocks, hydros, wheels of the five kinds, joints, cab and submeshes,
  flexbodies with forsets, props, managed materials, engine, brakes, cameras, cinecam) plus the BeamLab `joints`
  (parent, child, stiffness N/m, break force N[, ref x node, ref y node]) and `shells` sections and frame elements:
  beams with the option `F` take the section of the last `set_frame_section material, shape, outer m, wall m[, joints[,
  joint N m/rad[, break force N[, joint damping N m s/rad]]]]` (shape tube / box / rod / bar; joints rigid / ball /
  hinge_v / hinge_h / swivel / elastic for both ends or `a/b` for each, pinned1 / pinned2 as before; a break force makes
  the members mounts that tear off their first node when the force at their ends stands past it for 5 ms - a bolt, a
  hinge, a bracket; the damping makes a released joint resist turning against its node - a door's hinge, a ball joint's
  friction; both in the frame preset's panel) and a member's own joints after its options (`a, b, F, ball,
  rigid`), and `mounts` (node a, node b, break force N[, stiffness N/m[, turning damping N m s/rad[, kind[,
  parameter]]]]: a part's frame node b held on the frame at a distance, see Frame elements; kind p a point, c a clamp
  and its break moment N m, h a hinge and the part's second node on its line, s a stop, r a strap and its length as
  a factor of the rest distance), `fem_tris` (n1, n2,
  n3: triangle elements of the FEM frame, of the last `set_fem_shell material, thickness m[, r, g, b]` before them - a
  triangle before any is 1 mm steel; the colour paints the vehicle's plates), `collision_volumes` (`volume name[, break
  rms m]`, its `anchors n1, n2, ...` and its hull's `vertex x, y, z` lines: see Collision volumes); the beam defaults a
  vehicle's shocks, hydros and wheels were defined under (set_beam_defaults_scale applied where the builder applies it),
  its node friction on the wheels and `enable_advanced_deformation` are kept; so are `welds` (the sheet panels on their
  frame), `mounts` (the bolted parts), `slidenodes` (a steering rack's ends) and the nodes' `set_default_minimass`,
  drawn in the views (welds amber ticks, mounts orange lines, slide nodes green) and counted in Properties: a Buggy
  copied into the editor without them fell apart in the physics test. Layers, groups, the reference and the presets
  whole (`;editor-beam-preset:`, `;editor-shell-preset:`, `;editor-fem-preset:` lines with each run's preset index:
  their names, colours and the unused ones; from the directives alone presets of the same numbers merged, got
  names from their numbers, and the unused ones were lost) are comments the game's parser ignores. `tools/test_ror_parser.cpp` writes every template, the sheet car and two
  vehicles with their graphics through the editor's document and parses them back.
- The default beam presets are softer than Rigs of Rods' 9e6 N/m default: a node carries a stiffness proportional to
  its mass (k dt^2 / m per beam summed below ~0.45 at the 0.5 ms step), and a 10 kg node (the default minimum) takes
  about six 3e6 N/m beams; a stiffer preset needs heavier nodes (own mass, minimum node mass).
- Headless checks: `BL_INPUT_SCRIPT` plays mouse and key events (`frame:action;...`: `m|d|u x y` move / left down /
  up, `r|R x y` right down / up, `w x y n` wheel, `k|K|U NAME` press / hold / let go a key, `c TEXT` typed text);
  `BL_EDITOR_LOG` prints the views and every change of the model, `BL_EDITOR_EMPTY`, `BL_EDITOR_QUAD`,
  `BL_EDITOR_SELECT`, `BL_EDITOR_VEHICLE=<folder/file>`, `BL_EDITOR_GFXSEL=<n>`, `BL_EDITOR_SKEL=<opacity>`, `BL_EDITOR_DEFORM=<preset 0-3 | x>`, `BL_EDITOR_GFXONLY=1|2` (`BL_EDITOR_GFXGROW=<n>`), `BL_EDITOR_PHYSICS`, `BL_EDITOR_REF`
  and `BL_EDITOR_DUMP=<path>` (the definition it spawns) set it up for `--editor` screenshots.

**Performance** toggles the compact imgui_perfmon widget. The widget shows:

- a per-frame zone profiler;
- graphs for FPS, frame time, physics time, GPU time, simulation speed and active nodes;
- the physics wall time and its heaviest island, a colour bar of the CPU by what it is spent on; the zone table, the
  heaviest island's phases and the cache model under "Physics details";
- **Elements**: how many bodies (and pieces cracked off sheets), nodes, triangles, links between triangles (shared
  edges: a bending hinge each), beams and collision triangles there are, how many are awake (the rest sleep and cost
  nothing) and how much work each kind took this frame: node integrations, triangles and hinges the force kernel
  evaluated (counted by it: a refined sheet evaluates its coarse triangles less often), beam evaluations. Below: the
  awake triangles by level and by the short steps their body takes per substep (1 / 2 / 4), the sheets' free edges
  (border, cracks, laser), contacts and pair tests, the frame's topology events. Four graphs of the last 600 frames on
  one time axis (triangles; awake links; work per frame; physics wall time and the CPU time of the sheets, the
  collisions and the topology); the mouse over any of them marks that frame in all four and shows its values. The
  levels, steps, edges and the frame's events are under "Element details".
  "freeze graphs" stops them; "Record CSV" writes the same log as `BL_PROFCSV` (see below) to
  `beamlab_stats_<date>_<time>.csv` in the working directory (or the home directory).

## Scenes

| scene | content |
|---|---|
| Proving Ground | vehicle handling course: slalom cones, three ramps + landing, speed bumps, washboard, stairs, 20% and 40% hill climb, axle twister, mud / ice / gravel pads, ring road |
| Forest | hills with ~220 trees and bushes (pine, deciduous, birch, dead) with leaves, dirt road, light wind |
| Canyon Bridges | chasm with a steel truss bridge (holds cars and vans, breaks under the Tatra/bus/trucks) and a wooden girder bridge on trestles (light cars only) |
| Offroad Trail | rough terrain, rocks, mud and sand (soft ground), logs, trees |
| Rally Stage | 1.5 km gravel stage through fields and a forest: detailed road (wandering wheel ruts, camber, bumps, stones, potholes with mud, washboard before corners), hairpin, two crests, bale chicane; tape on stakes along the corners, round bales, square bale walls, haystacks, ~350 trees; flattening grass along the verges; stage timer (START / FINISH gates); Scene > "Autopilot" lets the AI drive the stage |
| Materials Lab | the same tests for 10 materials (steel, aluminium, lead, glass, acrylic, plywood, rubber, cardboard, foam, concrete; fabric): 1.7 m sheets on stands take a 60 kg ball from 6 m; cubes (5 m) and balls (8 m) are dropped and cylinders crushed by a 1.5 t press; 3.2 m bars on two supports take 300 kg from 3 m; a lane of 8 gates with panels to shoot through. The sheets and panels are triangle elements: they refine and crack where they are hit, torn-off pieces fall as bodies of their own (F3 shows the mesh). No vehicle: free camera only, with the grab / destroy / shoot tools. The tests run on their own after the load (Scene menu: again); labels name every sample |
| Sheet Run | three lead sheets (6 mm, 590 kg) clamped in gates 30 m apart on a straight lane: drive (or launch) through them; the profiling scene for sheet fracture |
| Sheet Shapes | triangle-element sheets of other shapes and sizes along a lane: a 6 x 4 m steel gate (1.1 t) across it, an aluminium half-pipe (2.6 m radius, 6 m long) to drive through, panels on stands (lead disc, glass ring, plywood triangle, aluminium L), an acrylic dome 4 m across; for the car, the balls and the laser |
| Yaris body-in-white (vehicle `yaris_biw`) | a real car body from a public crash-test finite element model, simplified for real time by `tools/import_lsdyna_car.py`: the 2010 Toyota Yaris coarse LS-DYNA model of the Center for Collision Safety and Analysis (George Mason University, sponsored by the FHWA; `assets/vehicles/yaris_biw/SOURCE.txt`). Its 137 000 structural shell elements (rails, floor, tunnel, firewall, rockers, pillars, roof rails, cross members, wheel wells, bumper beams, door / hood / boot inner panels, hinges) are clustered on a 0.3 m grid into a 266-node frame of 3000 beams (element adjacency, a 2-ring of bracing and 8 long braces per node: a single layer of clusters is a plate lattice that bends freely), the outer skins (doors, hood, fenders, body sides, roof, boot lid, bumper covers) are decimated by vertex clustering at 0.15 m into 450 triangle elements hung on the frame nodes within reach, and four wheels with a steering rack are put at the model's tyres. Frame beams yield at 8 kN (`enable_advanced_deformation`: the format raises smaller thresholds to 400 kN otherwise) so a wall crash crumples instead of bouncing. The car is in the Vehicle list, group "Crash-test FE models"; the crash test: `--scene crash --vehicle yaris_biw/yaris_biw --spawn -6,260,0 --launch 80` |
| Sheet Car | a hatchback whose body panels are a triangle-element sheet (2 mm steel, 380 triangles) on a deformable space frame: the player's car and a parked one 130 m down a lane with a wall, poles and cones; "Launch at 80 km/h" crashes them (the bodies dent, refine and crack, the cage bends; F3 shows the mesh). The vehicle `sheet_car/sheet_car` is written by `tools/make_sheet_car.py`; it drives and steers like any other vehicle and can be picked in other scenes |
| Frame Car | a five-door hatchback (4.1 m, 1036 kg) on a welded space frame of FEM frame elements (`tools/make_frame_car.py`: chromoly tube 50 x 2.5 floor rails from crash beam to crash beam, sills and cross members, 45 x 2.5 A, B, C and D pillars, roof rails and bows, the cowl, the aprons; rear rails and posts, towers) with its panels welded on: sheet steel (roof, quarters, rear panel, floor), glass (windscreen, quarter windows), trim (the grille). Each panel has nodes of its own, the metal standing 3.5 cm off its tubes (out along its normal), the glass in its frame's plane on soft welds (its seal: 10 kN/m); along each tube it runs a frame node lies under every other panel node, and each panel node over a tube is a weld (`welds`, `SoftBody::Weld`, 848 of them; the bumpers' covers also where they run near their frame, the doors' skins on their beams) that holds the weighted centre of the panel's nodes round it at that offset and lets go at 2.5 kN; the welded panels' membrane is damped (`SoftBody::membrane_damp` 0.6: their rattle a third of what it was). The parts that come off are frames of their own with their skins welded on, held on the body at a distance by mounts (`mounts`: no member between them, see Frame elements; each part a component of the frame solved on its own) that let go past their break force held 5 ms, far below anything of the body's frame: four doors (a frame with one intrusion beam across, the skin welded to it too; two hinges on a vertical line, 20 kN, their turning damped 4 N m s/rad; a latch that breaks at 12 kN, a check strap); the hood (a frame round its skin, two ribs along under it and one across; two hinges at the cowl, 15 kN, damped; a latch at the front, 6 kN; stays to the towers: it opens 55 degrees); the tailgate alike (hinged at the roof, 75 degrees); the front fenders (a frame round each and round the wheel arch, on four bolts, 9 kN each); the bumpers (a polypropylene cover, a profile swept round the corners to the wheel arches, on a frame of plastic tube - two rails and ribs - on plastic clips 8-18 cm off the crash beam and the cross above it, 2.5 kN); the mirrors (a plastic box on a plastic arm on three feet on the front door that bends and tears off at 500 N). The latches are springs of 3e5 N/m between components (explicit: 1e6 shook a lid's light latch node loose as the car stood). Head and tail lights are little glass boxes, every node on beams to four frame nodes round the box, tearing at 2.5 kN. Engine 420 N m, all-wheel drive: 0-100 km/h in about 7 s. The engine and gearbox (140 kg) weigh on the floor rails' nodes in the engine bay, the tank and battery (40 kg) on the rear floor's, the interior (280 kg) on the body's floor-level frame nodes alike; 60 kg of dry mass is spread over the members (spread over them all, the parts' frames took their share: a hood of 35 kg). Double wishbones on ball joints (damped 3 N m s/rad: a wishbone torn loose does not swing freely), preloaded coil-overs with travel stops; a steering rack behind the axle (a bar whose ends slide along its housing, one hydro moves it - a negative factor, the rack goes right for a right turn; arms of the rack out to the wishbones' pivot line carry the tie rods' inner ball joints, so the tie rods are as long as the arms), steering stops 4 degrees past the lock, rear toe stops. A collision hull of 50 big triangles on the frame's own nodes (floor, sides, the doors on their own frames, roof, windscreen, under the hood, front, the tailgate's opening, tail: cab option `h`, a few centimetres inside the panels, not drawn) keeps two cars' frames apart. Three collision volumes (see Collision volumes) close under its parts, 3 cm off their frames - the engine (on the floor rails, the towers, the firewall's feet; under the hood's frame and ribs), the cabin (on its floor, the A and B pillars; inside the doors, under the roof), the boot (under the tailgate, on its opening's line; ahead of the bumper's top) - hold other cars and its own panels (the doors, the hood, the fenders, the tailgate, the bumpers, with their skins) off: its latches let go, the hood and the tailgate rest on them. On a test pad: a lane to a concrete wall, a pole, a 15 degree ramp, a curb. Scene menu: drop it from 5 or 10 m, on the roof, a barrel roll at 50 km/h, trip it over the curb at 40 km/h, launch it at the wall at 60 km/h, at the pole at 50 km/h or off the ramp at 70 km/h, head-on into another Frame Car (50 km/h each), another into its side at 50 km/h, a 5 t concrete slab on it from 2.5 m, the giant axe (a 2 t pendulum that swings down under its weight; the sector its blade's edge sweeps is cut, `World::laser_cut`, and the car falls in two), hang it from a crane (level or tilted: the wheels hang in their travel, `tools/test_scenes.py` checks it), grab the roof, let the latches go (the parts' latch mounts and the latch beams between a part and the body), let every part go (the bolts and hinges too: the parts slide off the collision volumes, none falls into the car; the Buggy's and the Shell Car's pads have both); F3 shows the frame (steel blue, orange where bent for good). `python3 tools/profile_frame_car.py [scenario ...]` runs these stress tests headless and prints the physics time (mean / 95% / worst), the frame's share, splits, tears, failed solves; `BL_NOPANELS=1` draws the frame without the panels |
| Frame Car: Head-on / Side impact / Wall / Pole / Drop 10 m / Slab / Giant axe / Barrel roll | the Frame Car's crash tests (the report's videos) as scenes of their own: the Frame Car scene with that Scene menu test started on the first frame and the free camera on it; F5 runs it again, the other tests stay in the Scene menu |
| Buggy | an unlimited-class desert racer (a Class 1 buggy, 4.5 m, 1510 kg, 3.1 m wheelbase, 2.1 m track, 39 inch tyres; `tools/make_buggy.py` on the Frame Car's generator library `tools/fem_car.py`) on a welded chromoly cage of FEM frame elements: floor rails and sills, the main hoop braced back to the rear shock hoop, a triangulated front clip with a tube bumper (a box of rails and posts folded up as a mechanism once its joints yielded, and two of them head-on went into each other), the tail kicked up behind the transaxle (a tail at 0.64 m struck a kicker's face with the rear bottomed out). Aluminium panels welded on (roof, a 2 mm floor pan) and parts on bolts at a distance that let go: the nose (5 kN), the side panels and the engine cover (4 kN; on soft 2.5-4e5 N/m mounts: the cage flexing a few millimetres between a panel's bolts at a 5 m drop tore stiffer ones). Long-travel suspension from the linkages' kinematics: double wishbones in front (46 cm up, 21 down), trailing arms at the rear pivoting at the main hoop's foot (52 up, 26 down; the shocks leaning forward on the hoop over the floor's rear cross: a rising rate, 0.75 to 0.82), coil-overs of 1.1 and 0.95 Hz preloaded to hold the design height, damped at 0.45 of critical, their last 10 cm of wheel travel in a bump stop (spring and damping growing to 8 x the static load), hard stops and droop straps as shocks at the design length; the hubs and arms triangulated (a knuckle's outer axle node braced to both ball joints and the steering arm, the
rear hub to the arm's pickups, the trailing arms' pivots ball-jointed and braced: pin-jointed members left
mechanisms, and driven fast round a rough field the hubs folded, the wheels to 60-77 degrees of camber, now under
10; the rear anti-roll bar 30 mm like the front one: with the camber held the car rolled over on the circle instead of
spinning), the pickups and towers of heavier tube; anti-roll bars of spring-steel tube (a torsion tube in two bearings - a member along its axis, a swivel at the bar's end - levers and drop links to the arms; soft springs alone rolled it over at 0.8 g); the rear wheels' torque reacted on the frame over the axle (from the shock's top, 31 degrees off the vertical, the reaction's pair of forces pushed the wheel down and lifted the car 0.1-0.3 m on the throttle). V8 850 N m, four speeds, rear-wheel drive: 0-100 km/h in about 7 s (on asphalt the tyres hold it to ~0.45 g). The Frame Car's test pad with a gravel lane: ten 0.5 m whoops every 8 m and a tabletop jump (a 2.2 m kicker curving up to 12 degrees, a 16 m table, a 20 m landing); the Scene menu has the Frame Car's tests and the whoops at 80 km/h and the jump at 90. Two collision volumes (see Collision volumes) close under its panels, 3 cm off their frames - the cockpit (on its floor, the door bars' ends, the harness bar; inside the side panels, under the roof) and the engine bay behind the main hoop (the engine, the transaxle, the fuel cell: on the rear lower rails, the rear shock hoop; under the engine cover) - the nose left hollow - hold other cars and its own panels (the nose, the sides, the engine cover) off. |
| Buggy: Head-on / Side impact / Wall / Drop 10 m / Slab / Barrel roll / Whoops / Jump | the Buggy's tests as scenes of their own (the Frame Car's tests' scenes alike) |
| FEM Shells | triangle elements of the FEM frame (see Frame elements): a 2 x 2 m sheet of 3 mm steel (16 x 16 cells) on two supports, a 1.5 x 0.5 m cantilever of 8 mm steel clamped in a block, a hollow 1 m cube of 2 mm steel (5 x 5 cells a face). Scene menu: a 40 kg steel ball on the sheet from 3 or 10 m, a 500 kg block on it from 1 m (it folds to the ground), 100 kg welded under the cantilever's tip (a box of lead: it springs) or 250 kg (past its plastic moment: it folds at the root), 1 t dropped on its tip, the cube dropped 5 m on a face or a corner, 1 t spread on its lid, a 1 t slab dropped on it, the cube thrown at the wall at 50 km/h. F3 draws the triangles' edges on the plate's face towards the camera (blue, orange where yielded; F4: by load) |
| Shell Car | a saloon on the BMW E36's lines (the mod's, 4.9 m, a 3.0 m wheelbase, ~1460 kg; `tools/make_shell_car.py`), all FEM elements (no sheets). Its body-in-white is a thin spatial structure: FEM members of box section along its load-carrying lines (sills 110 x 1.8 mm, hinge pillars 100 x 2.0, B pillars 80 x 1.8, the A pillars on the E36's line and the C pillars, the roof rails and three bows, the cowl and the floor's cross members, the front rails with the upper rails along the fenders' line and the radiator support, the rear rails, the bumpers' beams on crash boxes, the rear doors' posts, the arches' lips) and single sheets of FEM triangles between them (the floor, the firewall, the roof, the aprons, the rear wheelhouses, the quarter panels, the rear panel, the parcel shelf): 175 nodes, 191 members and 216 triangles, ~310 kg of steel (the first version's double-skinned body sides and boxes: 342 nodes, 748 triangles, 30 -> 20 ms a frame). The hood, the fenders, the doors (with window frames of FEM members), the trunk lid (an L: its top and its rear face), the bumpers and the tail lights are the E36's panels as FEM triangles - steel, the bumpers and the lamps polypropylene in their own colours (`set_fem_shell`'s) - the bumpers scanned round their outlines (the corners' wrap), the quarters' feet and the rear panel's corners tucked in under the rear bumper. All are parts on mounts of their kinds: hinges (the part turns about its hinges' line only), latches, clamped bolts (the fenders, the bumpers' brackets, the lamps: they give 0.2 rad at their break moment and let go past it - a part left on one bolt does not swing about it), buffers (with its latch gone the hood rests on them, it does not fall into the engine bay; the doors' edges rest on their openings' flanges, so laid on its side it keeps its doors) and opening stays. The suspension of FEM tubes (the Frame Car's layout, stiff): double wishbones on ball joints inside the wheels, the lower ones 0.45 m long on the subframes' cross members, the upper ones 0.28 m in frames of members on the aprons or the rear wheelhouses (at the rear triangulated down to the lower pivots), uprights with the steering arms level with the lower ball joints, a rack sliding in its housing and tie rods in front, toe links on brackets at the rear - their inner ends placed by a kinematic sweep of the linkage: no bump steer (under 0.05 degrees over +-10 cm; the previous layout's rear toe ran to 40 degrees at full droop, the car on its side), camber -3 degrees at full droop; maraging steel (70 x 14 uprights, 45 x 5 arms) that stays elastic through every test; coil-overs of 1.6 Hz, 9 cm of droop; 280 N m rear-wheel drive, the engine's inertia a real one's (0.2 kg m2: RoR's `engoption` figure is taken in rpm, the 0.10 it had was ten times that - in first gear as much again as the car's mass to spin up), tyres of 24 rays (200 kN/m, 300 N s/m: a third less rolling resistance): 0-50 km/h in 3.5 s (5.5 before, with the FEM's lost push). Three collision volumes (see Collision volumes) - the engine (on the firewall, the rails' middle and the towers), the cabin (30 cm under the roof, 15 inside the doors; on the floor, sills, B pillars, cowl), the trunk - keep other cars, the slab, poles and balls out of it once its panels give. The volumes hold its own panels off as well (7-10 cm clear of them standing): the doors on the cabin in the side impact, the hood and the front bumper on the engine at the wall, the lid on the trunk under the slab. The Frame Car's test pad with the gravel lane and the rough field; the Scene menu has the crash tests, the drops, the roll, the curb, the ramp, the slab, the axe, the whoops, the jump, lays it on its side and lets the latches go; the test scenes under Test cars/Shell Car run one each (head-on, side impact, wall, pole, drop, on the roof, on its side, slab, axe, barrel roll, curb, whoops, jump, the latches). ~7 ms of physics a frame with the FEM at 2 kHz (8-11 in the crashes) |
| Steel Barrels | 200 l steel drums (572 x 880 mm, 1.0 mm steel of 200 MPa, 16 kg; `build_barrel`): the wall and both ends one closed sheet of 480 triangle elements (a 20 x 9 grid round the wall, rings and a fan on the ends), the two rolling hoops pressed out 8 mm at a third and two thirds, the ends dished in 2 cm inside a flat band at the rim (a drum stood on another rests on the band; on a slope down to the middle it was pushed off) and every node up to 3 mm off the true surface (a flat end and a true cylinder cannot start to buckle: they held 60 times what a real drum takes and nothing dented); the membrane is projected (a quarter of steel's yield force in the plane: 9 cm triangles stand for a wall that folds in waves of a few centimetres), bending is plastic from 0.01 rad with some work hardening; the ends are shaded apart from the wall (a crease at 45 degrees). A second kind (Scene menu: "with FEM rings", green in the front row) has the chimes and hoops as rings of 8 mm frame elements too (their stretch left to the membrane: a hundredth of their axial stiffness): the ends and hoops stay round, the wall between dents. A pad with a 15 degree ramp and a wall; the Scene menu drops one on its bottom from 1 or 5 m, on its side from 2 or 10 m, on the rim from 2 m, rolls it down the ramp, kicks it into the wall, throws one at another at 8 m/s, stands one on another, drops one onto another from 1.5 or 3 m, drops a 40 kg steel ball on one from 3 or 8 m and shoots it at one against the wall at 20 m/s. Scene menu too: tip one over (a push at its top). Drops dent the chimes and pop the dished ends out, the ball dents it deep (20% of its volume from 3 m; from 8 m it tears) and rolls off (projectiles meet the drums as spheres: `SoftBody::sphere_ball`, `sphere_target`; the drums meet each other node against triangle), a ball shot at one against the wall crushes it to half; knocked over or dropped they come to rest where they land, and left alone they stay where they are. `tools/test_scenes.py` checks each by the drum's volume (`BL_SHELLDBG` prints it), the frame rings, the ball's clearance, the tip, coming to rest and the drift at rest |
| Stress: Barrel pile | 15 steel drums on their sides in a pile (5 - 4 - 3 - 2 - 1, chocks at the bottom row), three more thrown into it one after another at 10 m/s (the sheet drums; `BL_BARREL_FEM=1`: with frame rings). `tools/test_scenes.py` checks the physics time |
| Tape Maze | 590 m gymkhana course marked only with tape: five lanes joined by hairpins, then a chicane; 450 stakes in 60 tape sections, stage timer, autopilot |
| RBR: Verkiai SSS / Undva / Travanca do Monte / Fernet Branca | Richard Burns Rally community stages (RALLY Guru), converted by `tools/fetch_rbr_stage.py`: a 1 km super special in a Vilnius park, 10 km of narrow Estonian forest gravel, 2.2 km through Portuguese hills, 6 km through the Cordoba hills. Original meshes, textures and baked vertex lighting; the collision mesh becomes a sparse 0.25-0.35 m heightfield with the stage's surfaces (tarmac, gravel, grass...); signs, banners and boards are light bodies carrying their own meshes (knocked over by the cars; boulders weigh 2.5 t), round bales are soft bales; the stage's tree trunks, stumps, walls and bushes are static boxes (bendable trees and bushes yield: a car pushes through at speed, not at walking pace); clock from the stage's own start / finish pacenotes, autopilot along the RBR driveline (tarmac or gravel limits from the surface under it) |
| Crash Test | runway, concrete wall, crate barrier, breakable street poles; "Launch at 50/80 km/h" |
| Vehicle vs Vehicle | two vehicles collide at a marked point: pick both vehicles, their speeds (0-160 km/h) and the layout (head-on, offset, T-bone, 45 degrees, rear-end); slow motion around the impact, peak g and damage report; F5 reruns |
| Primitives | shooting range with triangle-element sheets in frames: fabric (tears), steel plate (dents plastically, tears at high energy), plastic (brittle, cracks); primitives: crate pyramid, jelly balls, plastically denting metal blocks, truss tower + wrecking ball, elastic / plastic / breakable cantilevers (orientation joints), trampoline net, pendulums, dominoes, breakable plank |
| Stress: 16 vehicles / Demolition derby / 512 crates / Windy forest / Bridge convoy / Barrel pile | load tests |

## Vehicles

22 detailed mods from the official Rigs of Rods repository (70 variants), picked for mesh detail and for driving well in
BeamLab. See `assets/vehicles/*/SOURCE.txt` for authors and links; licences are as published by their authors.

- **Cars:** BMW E36 and E39 M5, 1988 Audi Quattro (and Rally), Audi 80 (5 variants), Mercedes-Benz CLK (4 variants),
  SEAT Ibiza, Toyota AE86 Trueno / Levin, Dodge Viper GTS.
- **Vans, SUVs, pickups:** Mercedes-Benz Vito, Mercedes-Benz G-Class W460 (5 variants), Mitsubishi Pajero,
  Ford F-250 1999 and 2014 Super Duty.
- **Off-road racing:** Unlimited class trophy truck.
- **Trucks:** LCF medium trucks (box, dump, flatbed, rollback), Autocar Xpeditor (11 bodies: dump, roll-off, hooklift,
  front loader, semi), Freightliner FLA semi, Kenworth T800 wrecker, TATRA 815 6x6, KME Predator fire engine.
- **Buses:** Thomas Saf-T-Liner HDX school bus, MAN Caetano Enigma coach.

Supported from the RoR format: nodes/nodes2, beams (rope/support), set_beam_defaults(+scale), set_node_defaults,
minimass, globals, cameras, cinecam, engine/engoption/torquecurve, brakes, axles, TractionControl, AntiLockBrakes,
speedlimiter, wheels, wheels2, meshwheels, meshwheels2, flexbodywheels, shocks/shocks2/shocks3, hydros (steering),
commands (keys 1-0), slidenodes/railgroups, fixes, ropes, submesh/texcoords/cab/backmesh, contacters, flexbodies+forset,
props (incl. dashboard steering wheel when the mesh exists), managedmaterials, sections/configurations (only the
selected section is spawned, node numbering counts all sections like RoR), PSD/DDS/PNG/JPG/TGA textures. Numbers beyond
the float range (`99999999999999999999999999999999999999999`) mean "never deforms / never breaks", as in RoR.

A vehicle can carry a sheet body: a `globals` cab material of `sheet/<Material>[/kg per m2[/thickness[/depth]]]` (the Materials
Lab material names; `:` is a separator of the format) makes `Game::spawn_vehicle` turn its `cab` triangles into
triangle elements (`apply_vehicle_sheet_body` in scenes.cpp -> `Vehicle::make_sheet_body`): elements on the vehicle's
own nodes (the frame nodes keep their beams and mass, the panel nodes get the sheet's on top of their own), the
texture coordinates as the material plane, a body version of the material (yields at 0.6 %, tears at 70 %, refines
only for a real crumple, 3 levels, no fracture pattern, pieces of 30+ triangles). The body then dents, refines and
cracks like a Materials Lab sheet, `reset` restores the authored triangles, and the vehicle drives like any other.
`tools/make_sheet_car.py` writes such a vehicle (RoR axes: -x forward, +z left): a profile of 18 points swept across
seven rows with a rounded shoulder and a tapered plan for the top strip, side walls with belt, mid and sill rows and
wheel arches (columns zipped into triangles, none narrower than 0.12 m: a sliver's stiff hinges would cap the bending
stiffness of the whole body), all triangles wound outwards. The cage is beams that yield at 30 kN and never break:
floor rails, spine and sills with cross members, A/B/C pillars, roof bows, belt and roof rails with shear diagonals,
bulkheads, bonnet and boot supports, and a strut tower over every wheel (the rails and the axle are nearly in one
plane: without the towers the wheel load folded the mounts and the wheels splayed). Steering: the outer axle node of
a front wheel swings about the kingpin (inner axle node - tower) and a `hydros` tie rod to a rack node turns it.
Frame nodes have minimass 5 kg, skin nodes 1.5 kg (`set_default_minimass`; glass, trim and doors), wheels 45 kg
(a 1 kg wheel node on a 9e6 N/m tyre spring plastically stretched its ring beams and broke them).

## Architecture

```
src/core      math, job system (fork/join, work boards for teams inside a job), profiler (zones + imgui_perfmon), utilities
src/gfx       shaders, textures (PNG/JPG/TGA + DDS/DXT with compressed upload), meshes, forward renderer
              (3-cascade shadow maps, sky, fog, ACES, alpha-to-coverage foliage, frustum/cascade culling)
src/phys      SoftBody (nodes, beams, shocks, slide nodes, wheels, oriented frames + joints),
              StaticWorld (heightfield, boxes, cylinders, ground models), World (islands, contacts, sleeping)
src/vehicle   RoR parser, OGRE .mesh / .material loaders, vehicle builder (ActorSpawner rules),
              drivetrain (engine, clutch, auto gearbox, differentials, TC/ABS), visual (cab, flexbodies, props, tyres)
src/world     terrain generation, beam objects (boxes, spheres, ropes, bridges, trees, poles, towers, nets), scenes, AI,
              imported stages (stage.h: bundle loader, scenery meshes)
src/game      application loop, input, camera, UI, the model editor (editor.cpp, editor_model.cpp)
```

### Physics

- **Rigs of Rods model.** The design follows the RoR sources: explicit symplectic Euler at 0.5 ms (2000 Hz). Beams are
  spring-dampers with compressive/tensile yield. Rest-length plasticity uses work hardening, tension weakens the beam,
  and beams break above their strength.
- **Rates** (`WorldSettings`, the physics settings panel). The beams, shocks, wheels, sheets and the contact forces
  every substep (2 kHz); the FEM frames' implicit step (members and triangles evaluated, the system assembled and
  factored) every `frame_every` substeps (1: 2 kHz, the default), in a lagging frame every `frame_every_lag` (2:
  1 kHz, h = 1 ms, its numerical dissipation per step halved: the same per second). A frame lags when the last one's
  physics took longer than a frame or it has more than 1.5 frames of substeps to catch up (`World::step_frame`,
  `lag_rate`; runs at a fixed frame time - the scripted tests, recordings - never lag: they stay deterministic). At
  1 kHz, in the substep between, the frame answers that substep's forces beyond what its step took them to be (the
  contacts, the welds, the beams on its nodes) through the same factorization, a forward and back substitution
  (`FemFrame::held_begin / held_component`: held for the next step instead, one step late, the Frame Car's welds let go
  as it stood and a thin shell's light nodes, pushed alone by a contact, tore). The mounts' springs are the frame's,
  their stiffness at most what the nodes' masses take at its step (omega h 1). At 1 kHz a spinning free plate strains
  a little more (1% of its energy against 0.1%), a cantilever's free vibration keeps 85% after 0.6 s. Collision
  detection at most `collision_hz` (120) times a second:
  the candidate pairs (every 17 substeps, their margins covering the travel until the next) and of them the pairs
  within reach (`World::near_pairs`: the contact radius plus both sides' travel at 1.25 times their speeds), whose exact
  contact is found every substep. Physics per frame at 1 kHz frames against 2 kHz and the adaptive broadphase: the
  old Shell Car driving 8.5 against 15.5 ms, the demolition derby 9.7 against 15.7, FEM Shells 6.3 against 9.1; two
  Frame Cars or two Buggies head-on the same (their explicit welded panels are most of it). The 120 Hz detection alone gains nothing measurable (the
  adaptive rebuild was ~125 Hz already; the narrow phase costs a quarter of the CPU time, off the critical path). What
  1 kHz frames change (`BL_FRAME_EVERY=2`, `tools/test_scenes.py`: 8 of 108 checks): the Buggy accelerates a quarter slower (the tyres'
  impulse is the same, the car's momentum gains less: lost round the rear hubs and arms), two Frame Cars head-on end
  1.6 m apart (2.2), its rear toe after the wall, a ball dropped 10 m on the 3 mm sheet dents it deeper
  (`BL_FRAME_EVERY=n` / `BL_COLLISION_HZ=n` or the settings panel set the rates: with the frames at 2 kHz and the
  detection at 120 Hz all 108 pass).
- **Shocks.** SHOCK1/2/3 force models, including progressive and soft-bump shocks.
- **Node masses.** Dry mass is distributed by beam length, with load nodes and minimass.
- **Frame elements (FEM, not RoR; `phys/frame_fem.*`).** For a stiff skeleton (a car's space frame, a roll cage) on
  which soft sheet panels hang: members are 3D Timoshenko beams between oriented nodes (axial, torsion, bending in two
  planes with shear deformation), welded to their nodes by default. Co-rotational: the member's frame follows its chord
  and the mean of its ends' orientations, the small deformations in it (elongation, twist, end rotations against the
  chord) meet the linear element stiffness, so large motions are exact. Sections from the material and shape (tube,
  box, rod, bar: A, I, J, shear areas, plastic moment and forces); the materials: Steel (S355), Chromoly (4130),
  Aluminium (6061-T6), Titanium, Carbon, Wood, Plastic, SpringSteel (51CrV4, 1.2 GPa), Maraging (maraging steel 300,
  1.9 GPa: suspension uprights and arms that stay elastic).
  - *Joints* at a member's end: rigid (welded), ball (all three rotations free), hinge_v / hinge_h (free about the
    member's z / y axis), swivel (the twist free), elastic (a rotational spring, N m/rad); a freed rotation is
    condensed out of the member's stiffness, a spring's flexibility adds to it. An end that frees the twist has no say
    in the member's co-rotated frame: with it, the member's energy changed as that node turned with no torque there to
    show for it, and that pull out of nowhere drove the uprights on their ball joints round in circles for as long as
    the car stood (a flutter only damping twice the real one would hide).
  - *The implicit step.* Members are far too stiff for the explicit step (a 40 x 2 mm tube 0.5 m long is 1e8 N/m along
    it, its end rotations 7e4 N m/rad on grams of rotational inertia), so the frame is solved implicitly once per
    substep: the members' tangent stiffness (B^T D B in the current frame, written as 3 x 3 outer-product blocks, plus
    the geometric stiffness of tension), the body's beams, shocks and hydros on frame nodes (their current tangent: a
    shock near its bump stop, the stop's) and the ground contacts (a stiff constraint along the normal, so the members
    cannot drive a light node back into the ground) are assembled over the frame nodes' 6 degrees of freedom and
    solved by a sparse block Cholesky (6 x 6 blocks, minimum degree ordering, double precision, inverse diagonals):
    (M + (theta h^2 + beta h) K + theta h^2 Kg) dv = h f - (theta_d h^2 + beta h) K v. For a linear oscillator the
    step's determinant is (1 + (theta - theta_d) a) / (1 + theta a), a = (w h)^2: with theta_d = 0 no energy is lost,
    a little (0.02, `WorldSettings::frame_dissipation`) damps what rings at the step's own rate; unconditionally
    stable for theta >= 1/4 + theta_d / 2 (`frame_theta` 0.5); beta is the members' material (Rayleigh) damping. The
    frame nodes keep double-precision positions that set their float ones after each step (along its axis a member
    is so stiff that float rounding far from the origin would be kilonewtons of noise and would random-walk the
    nodes). A body whose sheets take short steps (refined after a hit) solves its frame once per substep and holds
    the frame nodes at their new velocities in between: the members' forces count for the substep, the other smooth
    forces (gravity, the sheets, beams) are taken for the substep as they are at its start and what they turn out to
    be in the later short steps corrects the next solve, contacts count for their short step (all of them for the
    whole substep, as before, counted the later ones twice).
  - *Plasticity.* Past its yield a member forms plastic hinges at its ends (radial return; the plastic moment hardens
    by 25% over a radian of hinge rotation), yields in tension, buckles (Euler, 0.7 L) or yields in compression, and
    twists plastically. Once a hinge's plastic rotation passes 0.02 rad it is turned into the member's rest frame at
    that end, so the rotations the element measures stay the small elastic ones its formulation is exact for (bent
    half a radian and measured as such, its forces were not the gradient of its energy and pumped its fast modes).
    Damage is the largest excursion, not the path (shaking at the yield surface after an impact does not wear a joint
    through): a hinge tears off its joint past 10 x the material's elongation in radians (steel 2.0, chromoly 1.5,
    aluminium 1.0), a member tears when stretched past the elongation (crushed, it folds and does not tear). A
    yielding hinge's tangent is a pin's while it flows (more than 1e-4 rad in the step); if the step unloads it, the
    system is solved again with its elastic tangent (the changed members' difference applied to the assembled system,
    not the whole assembly again) and it keeps the elastic tangent for 16 steps.
  - *Splits and tears.* A member that forms a hinge is split once where it bends (a new node on its Hermite shape,
    turned as the shape is there), no shorter than max(0.15 m, 8 x its half-size), and its halves are not split again.
    A tear duplicates the joint's node for the member's end, like a sheet's crack. Pieces made of frame members alone
    that come off the body and weigh less than `frame_debris_mass` (40 kg) or a tenth of the body leave as rigid
    bodies drawn as tubes (FemFrame::detach_debris); the frame goes on without their members, and big parts of a car
    torn in two stay deformable frames.
  - *Cuts.* The destroy tool tears the members within reach and the laser the ones crossing its sweep, where they are
    hit: off the joint near an end, else split there and torn (FemFrame::cut).
  - *Safety.* A frame node's velocity change is kept under 40 m/s a step and its spin under 3000 rad/s (`clamps`
    counts the cut ones), a failed factorization leaves the members' forces explicit for that step (`solve_failures`).
  - Mass: the section's own rho A L on top of a share of the dry mass (frame members count with the beams); a node
    held by frame members alone takes no minimass.
  - Checks (`test_physics only frame`): a cantilever's tip deflection and rotation, twist and stretch equal the theory
    to 0.01%, a portal frame's sway within 1.1%, the first bending frequency within 0.9%, a spinning free frame keeps
    its energy (0.001%) and shape, plastic hinges hold Mp, sink to the theory's balance and keep their set, and break
    past their ductility; every joint type frees what it should; a split member keeps its shape, a tear frees the end;
    a frame box with a sheet roof dropped on the ground settles, the same on one thread and many; a stick torn off a
    frame box leaves it as a rigid piece and comes to rest on the ground. The Frame Car (181 frame nodes, 273 members,
    579 factor blocks): about 27 us for the members' forces and 66 us for the solve per substep on an M-series core.
    `BL_FRAMEDBG=1` prints why a member splits or breaks; `BL_FRAME_THETA`, `BL_FRAME_DISS` override the step.
  - *The body's springs on frame nodes* (shocks, bump stops, a wheel's spokes on its hub) join the implicit step on the
    frame's side: (theta h^2 k + h d) e e^T. One with its other end off the frame (a tread node on a hub) has that end's
    share of its mass condensed into the frame node's row (c mb / (mb + c) e e^T, its force's share on the right side)
    and the end's own change put back after the solve (`FemFrame::one_sided_rhs / one_sided_react`); taken on the
    frame's side alone, as if the other end stood still for the step, it took c e (e . dv) of the frame node's momentum
    every step - a driven FEM car lost a third of its tyres' push (the Frame Car 60%), a frame with a node hung on a
    spring fell at 0.92 g (`BL_ONESIDED_OLD=1`: as before, diagnostics).
  - *Components and mounts.* The frame's parts joined by members (and by the body's springs between their nodes) are
    its components: each is ordered (minimum degree), assembled, factored and solved on its own, the components of all
    the island's frames beside each other on the team (`FemFrame::solve_begin / solve_component / solve_end`; the
    members' forces in chunks beside the body's other forces, `begin_forces / eval_forces / end_forces`). A part held
    by mounts (`FemFrame::mounts`, `FrameMount`, the truck's `mounts`) is a component of its own: node b of the part
    held on a spring with damping at the point of the body's where it stood when made, carried by node a and three
    frame nodes round it (their positions' affine combination: no node's turning carries it - a 16 cm bracket on a
    light node's frame spun the node), b free to turn about it (a ball joint: two on a line are a hinge); the spring
    is explicit (on a steel part's stiff frame 1.5-2e6 N/m, else the most the nodes' masses take at the step), it
    lets go past its break force held 5 ms, or when a member tears off one of its nodes (its seat torn: the spring on
    a node left dangling flung it), and the two nodes' turning against each other can be damped (a hinge: implicit on
    each side against the other's turning at the step's start). A shock on a frame node lets go likewise when that
    node is torn down to one member or none (`Shock::seat`: the Buggy's shock towers torn off in a head-on left a top
    of half a kilo on a single tube and the stop's 2e7 N/m, flung at 400 m/s). Its kinds (`MountKind`): a point (b
    alone, as above); a clamp - b and up to three of the part's nodes round it each held at its point of the anchor, so
    the part does not swing about a single bolt: with a break moment the nodes round b are on springs that let the part
    turn `FemFrame::kClampGive` (0.2 rad) at it, a bolted flange's give, and it lets go past the break force or the break
    moment about the held nodes' middle; a hinge - b and the part's second node on the hinge's line (a hood, a lid, a
    door turns about that line only, and on one hinge left it still does); a stop - b kept from coming nearer to a than
    it stood, pushing only (a lid resting on its buffers: with its latch gone the Shell Car's hood rests on them, it does
    not fall into the engine bay); a strap - b kept within a factor of that distance, pulling only (a lid's opening
    stay). `FemFrame::release_latches` lets the point mounts of the hinged parts go (the Shell Car's Scene menu). Checks
    (`test_physics`): a square hung from one point mount swings down, from one clamp it holds; a clamp of 20 N m holds
    20 N at the far corner and lets go at 80 N below its break force; on one hinge the part turns about its line, the
    line's far node within 0.1 mm; a lid hinged over the frame falls through it with no stop and rests on one; a strap
    of 1.2 ends a swing that went out to 2 times its rest distance. The Frame Car's 36 mounts made 13 components of 558
    members (the body 233 nodes): the solve 3.1 -> 2.9 ms a frame summed, beside each other, driving 10.6 -> 9.1 ms.
    `test_physics fembench [steps]` prints the frame's cost per substep against its members for a ladder truss, a
    car-like cage, a cubic lattice and the cage cut into 1-13 parts (CSV).
  - *Mounts and damped joints.* A section's break force (`FrameSection::break_force`) makes its members mounts: the
    force at their ends (axial and shear) past it accumulates as an overload over time (the force over the break force,
    less one, integrated; it bleeds off below), and at `FemFrame::kOverloadTime` (5 ms: twice the force for 5 ms, 1.5
    times for 10) the member tears off its first node - on the peak alone a car dropped 5 m on its wheels rang its
    stiff frame and lost all four doors, the hood and the fenders. A section's joint damping (`joint_damp`) makes a
    member with one released end (a hinge, a ball joint) resist its node's turning against the member's welded end's,
    implicitly (h c on the two nodes' rotations in the system, against each other): explicit, 2 N m s/rad on a lid's
    light frame node spun it up and tore the lid off as the car stood. Checks: a mount at 0.8 of its force holds, a
    peak of 4 times it for 1 ms holds, twice it lets go after 5 ms; an arm on a ball joint swings on undamped, comes to
    rest with 0.5 N m s/rad and hardly moves with 1e5, sound.
  - *Triangle elements (FEM shells; `FrameTri`, `ShellSection`, `phys/fem_shell.*`).* A thin shell of a frame
    material (steel, chromoly, aluminium, titanium, carbon, wood, plastic) and a real thickness, solved in the same
    implicit step as the members, on the same 6-dof frame nodes (a body can be all triangles: a hollow box, a car's
    body with no frame under it). Each triangle is a flat co-rotational element: its in-plane frame fitted to its
    corners (the rotation that best maps its rest shape onto them), membrane CST (E t / (1 - nu^2)), bending DKT (Batoz'
    discrete Kirchhoff triangle, three points; D = E t^3 / 12 (1 - nu^2)), the corners' rotations measured against
    their rest orientations in that frame, a weak drilling spring (5% of G t A per corner) against the element's
    in-plane turning; the stiffness in the element's axes turned into the world's block by block, with the geometric
    stiffness of the membrane's tension, and stiffness-proportional damping (`ShellSection::damping` 3e-4 s: 1% of
    critical at 10 Hz; at 1e-4 a thin plate's low modes rang for seconds). Plasticity: the membrane's von Mises stress
    past the yield is returned radially (the rest shape follows the stretch), the bending moment at the three points
    past the plastic moment sigma_y t^2 / 4 likewise (the corners' rest rotations follow); the damage is the rest
    shape's largest principal plastic stretch against the authored shape (a state, not a path: a panel shaken on its
    yield surface did not tear), past the material's elongation the triangle tears - nothing is removed: a crack opens
    along its edge most across its plastic stretch's largest principal direction, the edge's end nodes duplicated
    (`FemFrame::tear_tri`: at each end the triangles on the torn element's side of the crack take the copy, a third of
    the node's mass at least; a node inside the sheet has the crack go on along its edge straightest on from the torn
    one; the members, mounts, welds and springs keep the node; the collision triangles follow), each further tear of
    the same triangle half the elongation later, three and it is a piece. The laser and the axe part the shell along
    its edges nearest the cut (`FemFrame::part_tris`: the triangles round the cut sorted by the side their middles are
    on, every corner with triangles on both sides duplicated), the destroy tool tears radially. Splits, compaction and
    debris handle the triangles like members. A triangle's mass
    rho t A goes a third to each corner, its rotary inertia to the nodes. A limit: the contacts with other bodies are
    explicit, a thin shell's node pushed as a free node of a few hundred grams against its members' forces, so a heavy
    body resting on a thin shell keeps it trembling a little (a 1 t slab on the 2 mm cube: its nodes at a few tenths of
    a m/s, the cube creeping a few mm a second; taking the node's compliance from the implicit step for resting
    contacts stopped the creep but made impacts harder and the Frame Car's frames worse). `ShellMesher`
    builds shells (nodes shared by position, grids, a two-sided collision triangle each). Checks (`test_physics only
    fem_tri`): a rigid turn makes no force, a cantilever strip's tip across between the plate and the beam results
    (14.9 mm: 13.9 / 15.2) and along to 0.6%, a clamped square plate's middle within 2% of Timoshenko's table, a free
    plate spun about three axes keeps its energy (0.9%), a strip at half its first yield springs back, at 0.9 of its
    collapse load keeps a set, at 1.3 folds, pulled at twice its yield tears (no triangle removed, the nodes along the
    crack duplicated, the strip in pieces), a plate cut across by the laser parts into two; a 1 m box of 2 mm steel dropped 1 m
    rests, silent in 4 s. The cost: a closed shell's factor grows as n^1.5 (the Shell Car's 316 nodes: 20k block
    updates, 9 ms a frame; `BL_FACTORDBG=1` prints a frame's factor pattern); `BL_FEMDBG=n` prints every n frames each
    body's triangles (torn, yielded, peak load, the fastest node).
- **Wheels.** Node-ring soft tyres generated exactly like `ActorSpawner`. Drive torque is applied as tangential
  tread forces with a reaction on the suspension arm. Brakes use a one-step "stop torque" clamp.
- **Tyre grip (not RoR).** Tread nodes use a velocity-level Coulomb model: sticking cancels the applied tangential
  force and removes the slip (up to ms x load), sliding keeps ~85% of the peak on a Stribeck curve, and the contact
  patch of each wheel shares its grip (a node ring has only one or two, often lightly loaded, nodes on the ground).
  About 1.0 g cornering and 1.2 g braking on asphalt; Physics > Tyre grip scales it. RoR's adhesion
  (1 - exp(-v / 3 m/s)) made a slipping wheel find almost no grip again.
- **Steering.** RoR's digital steering (the rate falls with speed, self-centring) with adjustable rates; the default
  is twice RoR's, counter-steering adds the centring rate. The hydros follow within ~0.05 s (38 degrees of lock in
  0.2 s at a standstill).
- **Drivetrain.** Engine torque curve, automatic clutch and gearbox, and open / locked / viscous / split
  differentials, as in RoR. The clutch capacity follows the current gear (RoR used 1st gear: a torque spike after
  every upshift), the automatic shifts early at part throttle and late at full throttle, with kick-down. A generic
  traction assist covers mods without TractionControl (Physics menu). The engine's inertia (`engoption`) is in the
  drivetrain's rpm units: RoR mods use 0.05 - 0.1 with a clutch force of 300 - 1000; the generated cars and the
  editor's default had 0.35 - 0.4 and 200 (the engine revved ~5x too slowly, the clutch slipped: the Frame Car
  reached 26 km/h in 5 s) and now use 0.08 and 1000. A body stepping in short steps (a refined sheet: 2 or 4 a
  substep) runs its controllers once a substep for the whole substep, and its wheels keep the drivetrain's torque for
  every short step (the short step's length and the torque zeroed after the first ran the engine, the gearbox and the
  steering 2-4x slow and drove at a fraction of the power: a car with one dented panel crawled).
- **Aerodynamics.** RoR's per-node drag (-0.05*|v|*v on every node, ~9 kN at 75 km/h on a 440-node car) is applied
  only to each node's velocity relative to the body, so it still damps flapping parts; the body gets real drag
  0.5*rho*CdA*|v|*v from its frontal area.
- **Contacts.** RoR `primitiveCollision` with adhesion + Stribeck friction and ground models. Mud and sand are fluid
  layers. A road can carry a detailed surface (`phys::RoadSurface`): the heightfield holds the smooth bed, the ruts,
  camber, bumps, stones, potholes and washboard are an analytic shape baked into a 0.1 m grid that both the
  contacts and the road render mesh sample.
- **Inter-body collisions.** Node-vs-triangle, node-vs-capsule and capsule-vs-triangle, with exact barycentric
  effective mass. The contact cancels the effective force on the relative motion from both sides, so a light
  triangle touched by several nodes of a heavy body in the same substep is not corrected several times over. The broadphase is a spatial hash of contacter nodes (Teschner et al. 2003) plus a Verlet-style
  candidate list rebuilt at the collision detection's rate with a velocity-based margin. Vehicles without collision cabs get
  a convex-hull shell made of their own nodes.
  - A collision triangle is a thin two-sided surface as thick as the node's body's collision radius: its normal points
    from the nearest point to the node, so a node pushed past its middle is pushed on through. Thin panels and the
    frames' nodes between them let two cars go through each other (two Frame Cars head-on ended with their centres
    0.3 m apart). **Hull triangles** (cab option `h`, `Triangle::two_sided` false) are one-sided and solid: their winding
    faces out, and a node up to the body's `hull_depth` (0.3 m) behind one, over its face, is pushed back out along its
    normal. Laid on a frame's own nodes as a coarse closed shell round the body, they keep two frames apart. Behind the
    face the response only brings the node's outward speed up to the push-out speed (a fifth of the depth a step, at
    most 2 m/s), counting the forces on both sides so far; RoR's response cancels every inward force of a node in
    contact and, held deep in a hull for many steps, pumped a ball out by its own springs at 6.5 m/s. A body with hull
    triangles reaches its hull depth past its box (islands, partners). Hull triangles tear when an edge is stretched
    2.5 times, when a laser or the axe's cut crosses them (on a sheet body too: before, a sheet body's other collision
    triangles were never cut) and with frame debris breaking off (they do not hold debris on the body).
  - **Collision volumes** (`SoftBody::volumes`, `CollisionVolume`, the truck's `collision_volumes`; `World::collide_volumes`):
    a heuristic for what fills a car - the engine in its bay, the seats and occupants in the cabin, the trunk's load -
    under its thin panels: convex hulls (the hull of the given points) riding on N anchor nodes of the frame, placed
    every substep by the anchors' best rigid fit (the rotation by Mueller's iteration from the last). Other bodies'
    contacter nodes inside one, their balls reaching into it and other volumes' vertices inside it are pushed out along
    its nearest face, so are the nodes of the body's own parts on mounts - the FEM components that hold a mount's part
    side, not the anchors' own, and the sheets welded to them (the hood, the doors, the lid, the fenders, the bumpers
    and the lamps, a door's skin on its frame; not the body-in-white or the suspension; `SoftBody::find_volume_parts`,
    again once a vehicle's sheet is welded on, the nodes inside one as built left out with a warning; a crack's or a
    refined sheet's new node goes with its parent; `BL_VOLPARTS_OFF=1` leaves them to the other bodies alone): a door
    pushed in stops on the cabin, the hood folded back on the engine. A vehicle's reset puts its volumes back whole; and its vertices out of the static world (terrain, boxes) and a pole through its faces; each contact
    is the ground's response on the pair's effective mass, the volume's share spread onto its anchors as the force and
    the moment it makes on a rigid body. Crushed or torn out of shape past its `break rms` (the fit's residual) a volume
    is off for good; a body with `volume_pass` (the giant axe) goes through them. Bodies with volumes share an island
    with anything they could touch. The Scene status line gives each volume's contacts and largest force; with the
    collision view (F-key or `BL_COLLISION=1`, `BL_HIDEMESHES=1` for the view alone) they are drawn in magenta.
  - Each body first gets a list of partners (bodies whose expanded boxes overlap); only nodes inside a partner's box
    enter the hash, and a body's own nodes are skipped in the scans unless it self-collides. A sheet or cloth next to
    other bodies used to scan thousands of its own nodes every rebuild.
  - Fast bodies (above 8 m/s: projectiles, a car, flying pieces) do not set the rebuild interval of the island: their
    pairs are searched on a schedule of their own, about every 6 cm of travel of their bulk speed (mass-weighted RMS of
    the node speeds: the top of a spinning wheel moves at twice the car's speed), and every node and triangle gets a
    margin of its own speed. A fast body's contacter nodes go into a dense grid over its bounds (cells of about twice
    the reach, each node in the cells its travel reaches); a partner triangle scans only the cells of its reach box,
    candidates are deduplicated, then rejected by the distance to the triangle's plane before the closest point is
    computed. A car in a cracked lead sheet: 20 ms of brute force per frame -> ~1.5 ms.
  - After a crack or a refinement the new triangles and nodes take over the pairs of what they came from
    (`SoftBody::TopoLog`): a half of a triangle lies within it, a new node lies on an edge between two nodes (within
    reach of whatever one of them reaches) or on the node it was split from. The island used to search all pairs
    again after every topology change (25 rebuilds per frame while a sheet cracks).
  - The candidate search runs in chunks of triangles, the narrow phase in chunks of pairs (the island's team, see
    Scheduling); the contact response stays serial, in pair order: each contact cancels the forces of the ones before
    it.
- **Orientation-preserving joints (new, not in RoR).** Some nodes carry an orientation (a "frame"). A joint keeps the
  child's full pose (position + orientation) relative to its parent frame using 6-DOF springs, in the spirit of
  oriented particles (Müller & Chentanez 2011) and Cosserat rods (Kugelstadt & Schömer 2016). It is used for
  one-segment-thick trees and cantilevers.
  - Joints yield plastically (they stay bent) and snap only after accumulated plastic strain, so almost no elastic
    energy is released when they fail.
  - Trees stand on a root plate: a free root node with its own frame, held by a soil joint that is stiff in position
    and soft and plastic in angle. A hit tilts the tree (it stays leaning), a hard one uproots it. (A joint on a
    fixed node cannot tilt its child: the fixed frame never rotates and the linear spring pins the child.)
  - Anchored scenery is pre-settled with gravity-sag compensation, then starts asleep.
- **Triangle elements (sheets, `phys::Shell`).** A triangle of three nodes held by its own three edge springs (solved
  like RoR beams: stiffness, plastic yield, fracture strain), bending hinges to the neighbours across its edges, drawn
  1:1 and used as the collision triangle. It never breaks as a whole:
  - Overloaded (a fraction of the fracture strain, plastic flow, or a sharp fold), it is bisected through its longest
    edge (Rivara's longest-edge bisection); the neighbour across that edge is split at the same midpoint (bisected
    first if that edge is not its own longest one), so there are no T-junctions. Right isosceles triangles of the grid
    stay right isosceles: two bisections give the 4-triangle grid of the parent. The new node gets the interpolated
    position and velocity; a third of every triangle's mass sits on its corners, so mass and centre of mass are kept
    exactly; rest lengths, plastic state and texture coordinates follow the parent. Edge stiffness depends only on the
    shape (k * Lmin / L), so refinement keeps the sheet's stiffness.
  - At the finest size (4 bisections: edges a quarter, area 1/16; min edge 2 cm; at most 2^max_level times the
    authored triangle count, enough to refine all of it: a smaller budget ran out after the first hit, and a second
    ball through the same sheet was hardly refined) a crack opens instead: a node shared by several triangles is duplicated along the crack line (between the
    triangles, along their edges, O'Brien & Hodgins 1999 in spirit); the copies keep its position and velocity, the
    mass is split by triangle. Crack tips grow first, at most two new cracks per substep, and no crack may cut off a
    piece smaller than `min_piece` triangles (6 for brittle, 12 for ductile materials): no dust. Parts held only by a
    shared node (no strength) are separated; pieces with no fixed node become bodies of their own (islands, bounds and
    sleep of their own) in the sheet's collision group. The area of a sheet plus its pieces stays exactly the same.
  - Smaller triangles have lighter nodes: a refined sheet takes 2 or 4 short steps of its own per substep, each
    triangle evaluated at the rate its size needs (contacts are held over the short steps; only the contact part of
    the node force is cancelled, cancelling its own spring force of the first short step flung light nodes). Hinges
    of the fine levels are softer (bending waves would need a step ~ size^2), so bending thresholds are moments, not
    angles. Air drag on the faces and a damping of the motion across the sheet let torn flaps and membranes settle;
    clamped edges resist rotation (a pinned border let flaps swing forever). The fold that asks for refinement is
    measured from the rest angle of the hinge, not from flat: a car body has 90 degree edges at rest. A hinge's
    stiffness fades out for a triangle crushed towards a line during the step (`kHingeFadeC`, shell_util.h): its angle
    gradient grows as 1 / height and the stability budget was met at the rest shape, so below 2/3 of the rest height
    the fade is the squared height ratio and the force stays at its rest value (a linear fade left it growing as
    1 / height and flung 0.1 kg nodes of a crushed car body at 370 m/s).
  - **Coarsening** (`SoftBody::coarsen_shells`): the reverse of the refinement where the sheet has settled. A node put
    in by a bisection whose triangles are at rest (strain below 0.3 of the refine threshold, nothing queued, no
    plastic stretch or bend that would ask for the detail again) is taken out and the four triangles round it (two
    on a border) sewn back into their two parents: the rest lengths along the split edge add up (scaled to the
    straight distance in the material plane where a pattern moved the node off it), the outer edges keep their
    plastic and pattern state, the areas and masses add up exactly, the momentum of the node goes to its neighbours.
    Each triangle records which of its edges is a half of the edge each level's bisection split (`Shell::half`), so
    the parents are the true ones (the two diagonals of a 4-8 square are indistinguishable by shape). The world
    coarsens every sheet `coarsen_delay` (0.25 s) after its last refinement or crack, up to `coarsen_per_frame`
    merges each, the bodies in parallel. A hit sheet thus stops paying for its detail once it is at rest: with the
    sleep, the Materials Lab drops from 8 ms to 2.5 ms of physics 8 s after the balls fell. `BL_COARSEN=0` turns it
    off (diagnostics).
  - **Rigid pieces** (`SoftBody::make_rigid`, `ShellMaterial::rigid_pieces`): a piece cracked off a sheet becomes a
    rigid body at once: centre of mass, orientation, velocity and angular velocity (from the nodes' momentum and
    angular momentum), the inertia of its point masses; every step the forces on its nodes (contacts, gravity) add
    up to a force and a torque, and the nodes are placed from the centre and the orientation. It neither bends nor
    cracks further, takes no short steps and costs its contacts only; on the ground its spin is damped fast (the
    penalty contacts give back the energy a real tumble swallows). Fabric, rubber and cardboard keep soft pieces. A
    rigid piece the laser cuts or the destroy tool hits is soft again. `BL_NORIGID=1` turns it off.
  - **Node budget** (`SoftBody::enforce_node_budget`): after every topology change the explicit stability budget of
    each touched node (the springs it carries, and the hinges where they can matter, x short step^2 / mass) is
    enforced by softening its triangles (`Shell::kscale`, only ever down). A node of two or three finest triangles at
    a crack edge is far lighter than the authored calibration assumed; before this such nodes jittered at 1 - 4 m/s
    and kept a sheet at rest from sleeping.
  - **Membrane projection** (`ShellMaterial::membrane`, `SoftBody::project_membrane`, for thin metal: the barrels). An
    explicit edge spring as stiff as 1 mm of steel in its plane needs a step far below 0.1 ms on 70 g nodes, so the
    sheet capped it at what the step allowed, and a drum of it was paper: it folded under its own weight on landing.
    With a membrane yield force (sigma_y t per width, N/m) the plane is held by projection instead: after each short
    step every triangle edge is put back within +-`yield` of its plastic rest length (Gauss-Seidel, two sweeps forth
    and back, done early when nothing moves), the correction at most what the yield force makes in the step, the
    rest flowing plastically into the rest length; the nodes' velocities take the correction over the step (v += dx
    / h). The kernel does not yield such a sheet in its plane (its springs are then a small elastic part; the kernel's
    yield of an unconverged overstrain crept a stacked drum flat), and the stability budget goes to the hinges (0.5 of
    it, the edges 0.15; otherwise 0.2 and 0.45). Bending stays explicit, plastic at the plastic moment.
    `SoftBody::membrane_damp` (0..1, the welded car bodies 0.6, `BL_MEM_DAMP`): in the projection's first sweep the rate
    of stretch of each edge that left its band in the last 64 short steps is taken down by that part (a velocity
    projection along the edge: stable at any step, it keeps the momentum, the angular momentum and a rigid turn; every
    edge in a pass of its own cost 2 ms a frame on the Frame Car). Between the band's soft springs and its hard ends a
    car's welded panels rattled for good - the floors and the door glass half a millimetre a frame as it stood, nodes at
    up to a metre a second between frames - where the kernel's own edge, hinge and flex damping (explicit, capped by
    the step) changed nothing; damped, a third of it (0.3: a half) for 0.4 ms a frame. Checks: `test_physics only membrane`.
    A welded panel bends as the plate it is (D = E t^3 / 12 (1 - nu^2), t from its areal density; the stability cap
    above it): at the cap alone a sheet of even triangles had every hinge at its limit at once and rang from step to
    step (the Buggy's 1.2 mm aluminium eight times too stiff, 7 m/s on its skin as it stood); the Frame Car's cap is
    below its steel's plate (a thin triangle sets it) and its trim's a little above. `BL_PLATE_BEND=<x>` scales it.
  - **Short steps of a stiff sheet** (`SoftBody::shell_min_shift`): a sheet may take 2^n short steps per substep from
    the start (not only once refined), and its hinges get the 4^n larger budget: the drum's 1.1 mm plate bends with
    its real D = E t^3 / 12 (1 - nu^2) at 4 short steps; at one, a 40 kg ball from 8 m flattened it.
  - **Sphere contacts** (`SoftBody::sphere_contacts`, `sphere_contacts.cpp`; off for the drums, `BL_BARREL_SPHERES=1`:
    a pile of them spent most of its step on the spheres between them, and a dented drum's own spheres pushed it round
    on the ground; a `sphere_target` sheet meets only the balls so): two sheets meet as sets of small spheres instead of
    node against triangle (which misses two rims crossing edge to edge): a sphere at each node, at the thirds of each edge and at each
    triangle's centre (`sphere_div` 3), each 0.4 of that spacing across (`sphere_scale`; the gap they leave between
    two drums is about 3 cm, one sphere a triangle left 8). The pairs between bodies are found once a substep (a grid of
    the spheres of the triangles in the two bodies' overlap, the pairs within their radii and the travel of the
    substep) and resolved every short step. The response is a projection: the closing velocity out,
    Coulomb friction (0.4), the overlap pushed apart in position only (it adds no velocity; each node at most 0.2 m/s
    a call over all its contacts: one push per pair, not per node, pumped a drum set into another off to 5 m). A
    sleeping sheet is an immovable obstacle and wakes when its partner moves at three times its rest threshold. A
    projectile ball (`SoftBody::sphere_ball`) is one sphere of its radius at its centre of mass, met at the sheets'
    every short step (node against triangle held a 40 kg ball stuck in a drum's wall).
  - **At rest** (`SoftBody::contact_slop`, `rest_damp`, `rest_vib_damp`, `resting`): a drum left on the ground crept
    and a lying one rocked for good: RoR's contact pushes a node out of the ground at a fifth of the depth a step as
    velocity, and every touch kicked a light node of the stiff sheet off again; the membrane projection pressed the
    nodes on the ground into it (now they stay put in it, `ground_touch`, the other end of the edge moves). A drum
    has no push below 3 mm; on the ground or on another and slower than 0.5 m/s its motion as a whole is damped at
    3/s and its vibration at 10/s (the rolling resistance and the metal's damping); slower than 0.3 m/s it does not
    flow plastically (its weight on a few nodes ratcheted a low yield round). A frame node on the ground that sticks
    (slower than 5 cm/s, its friction holding) is held along the ground in the frame's implicit step as along the
    normal (`FemFrame::contact_n` twice as long): a ring's node slid under its members' forces and a drum with frame
    rings crept along. The membrane does not flow at rest either. Held only where it could stand
    (`SoftBody::over_support`): its centre of mass over the hull of its nodes on the ground this frame (and those within
    5 mm of them), 5 mm to spare; off it there is no rolling resistance and no sleep (tipped over its rim, a drum was
    slowed to a stop at the top and slept balanced on its edge). At rest the ground grips it eight times harder (a drum
    crushed flat by a fall trembles a while, and its trembling nodes walked it about). A sheet with rest damping sleeps
    when still as a whole (or but for a wobble under 0.18 m/s on average for 3 s), together with the bodies it touches
    (`sleep_ready`: one asleep became an immovable obstacle, the jolt woke it again, and a drum on another and the one
    below took turns for good); a neighbour wakes it at three times its rest threshold.
  - **Energy guard** (`SoftBody::energy_guard`, the drums with frame rings): a passive body left to itself cannot gain
    energy; a frame in which it met no other body (nor in the 15 before) and gained more than 2 J and a twentieth of its
    kinetic energy in motion and height has its velocities scaled back to that. The explicit sheet, the implicit rings
    on the same nodes and the membrane projection between them went unstable now and then at rest: within a few frames
    a drum threw itself about at 100 - 400 J and spun round. The rings' axial stiffness is a hundredth of theirs (the
    membrane holds their stretch; at full stiffness they fought the projection on every edge and the drum trembled
    with a few joules for good). The sheet alone gains at most 1 J in a frame (its dents springing back) and needs none.
  - **A vehicle's sheet body keeps its node numbers** (`SoftBody::keep_node_order`): the vehicle places, repairs and
    drives it by the definition's; a body of the sheet alone (no beam: the model editor's drum drawn from a circle of
    sheet and pulled out) was put in Morton order and came apart on its first frame. The kernel's gather lists take up
    to 255 entries a node (the centre of a disc of 48 triangles has 96; a buffer of 64 overflowed). A body of the sheet
    alone of a metal has its plane held by the membrane projection (sigma_y t from its kg/m2) and the barrels' rest
    settings (a car's panels on its frame keep the springs the step allows: alone, a 2 mm steel drum dropped 1.5 m lay
    flat).
  - **Welds** (`SoftBody::Weld`, `welds.cpp`): a panel held on its frame at points instead of sharing nodes with
    it (a panel on a single shared node tore out round it). Each weld is a damped spring between the anchor (a frame
    node, or a point along a member: its pull shared by the member's two nodes) and the weighted centre of the
    panel's nodes round the weld's own, its rest offset turned with the frame node and the pull's moment, (q off) x f,
    put on the node's rotation on the frame's step (without it the offset turning with the node pushed the panel
    and the node never felt it: frames spun up to the solver's clamps); stiffness a quarter of what the short step
    allows on the pair's effective mass, and for a weld held off its node within a tenth of what the node's
    rotational inertia takes in a step (k |off|^2 h^2 / I: a lamp 20 cm off a tube's node rang it to pieces), damping
    a third of critical; past its strength it lets go (a crash tears panels off along their seams). A laser's (or the axe's) cut breaks the welds whose sheet it crosses.
  - **Sheet and frame in one body.** A sheet's renumbering (`reorder_shells`) and its pieces (`detach_pieces`)
    carry the body's FEM frame along (`FemFrame::renumber`, `FemFrame::split_off`: a piece takes its members with
    their state); before, a drum with frame rings came apart into 120 pieces off a 1 m drop.
  - **Refinement quota** (`WorldSettings::refine_per_frame`, 200): bisections per sheet per frame from the kernel's
    overloads; the rest queue again next substep, so an impact's burst is spread over a few frames. The pattern
    lines are refined without the quota (behind the cracks they would run off the pattern).
  - Pieces: no crack may cut off fewer than `min_piece` finest triangles (10 brittle, 20 ductile): no dust, and a
    third fewer bodies.
  - The destroy tool refines the sheet around the cursor to the finest size and shatters it into loose triangles.
  - The laser (`World::laser_cut`, `SoftBody::cut_shells`) cuts along the plane the camera ray swept between two
    frames, inside that sector and within its range; nothing is flung. The triangles it passes through are refined to
    the finest size, each triangle belongs to the side of the plane its centroid is on, the links between the two
    sides are cut and the nodes on the cut get a copy for each side (position and velocity kept). The cut runs along
    triangle edges, a staircase of the finest triangles; the nodes on it (with their copies) are then moved onto the
    plane, along the sheet, as far as the triangles around them keep their shape (at most 0.4 of the shortest edge; no
    triangle squashed below 3/4 of its area or edges, none moved to the other side of the plane), and the rest lengths
    and areas follow the move in the material plane (the sheet's uv in metres: the areas add up to the same however
    the sheet is deformed), so there is no stress from it. A sweep over several frames cuts in steps; a link belongs to
    the step that passes the middle of its edge. Beams, joints and collision triangles of other bodies crossing the
    plane inside the sector break.
  - **Fracture patterns** (`shell_pattern.cpp`, `ShellMaterial::pattern`): where a material's cracks run. Glass and
    acrylic (`Radial`): a web round the point of impact, 7 - 12 wavy radial cracks and 3 - 5 polygonal rings between
    them (a straight chord per sector, some missing further out). Metals (`Punch`): a ring round the point of contact,
    about the size of the body that hit, with 3 - 5 radial tears out of it and sometimes petals inside. Plywood
    (`Grain`): the fibres run one way (`grain_angle`); pulled across them an edge holds `grain_ratio` times less than
    along them, a fold along them splits them, random wavy veins every `vein_spacing` are weaker still, and an impact
    splits the board along the grain through the point of contact. The lines live in the sheet's material plane (its
    uv in metres), so they stay on the material however the sheet bends, and go on in the pieces.
    - A contact faster than `pattern_speed` (the approach speed along the normal, found in the serial contact
      response) lays a pattern at its point; its size grows with the speed and with the size of the body that hit. One
      body going through gives one pattern (its other points of contact within 3 zone radii and 0.25 s are the same).
    - The triangles on the lines are refined a level per substep, down to one level above the finest. A bisection puts
      its new node where a line crosses the split edge (not at the midpoint), nodes near a line are moved onto it, and
      an edge between two triangles whose far corners are on a line is flipped onto the line (the rest shape of the
      quad, its area, mass and momentum kept). Inside a zone the split points are a little off the midpoint at random:
      the crack edges are not straight.
    - Every edge gets a strength code from the pattern (`Shell::es` / `hs`, a byte each, x / 64 of the fracture strain
      of its spring / of the fold across it; the kernel reads them per edge): across a line `pattern_weak`, the other
      edges of the zone `pattern_strong` (the pieces between the lines hold together), and a little softer or stiffer
      (0.9 .. 1.15 of the stability-capped stiffness) so the strain gathers on the lines. A crack prefers a spoke on a
      line (wood: along the fibres). The glass web: about twice as much crack length on the lines as the same impact
      without a pattern, metal about three times; wood about twice as much along the fibres.
    - The patterns move nodes off the 4-8 grid only as far as the explicit step allows: no triangle below 0.6 of its
      nominal size (`Shell::area_nom`, the authored area / 2^level: its lighter nodes would need a shorter step) or
      thinner than 0.72 (4 sqrt3 area / sum of the squared edges; 0.87 for the grid's right isosceles triangles: a thin
      triangle's hinges are stiff), and only on a shape near its rest shape.
  - **Artifacts** (`ShellVisual::update_fx`, a mesh per kind): the pattern's lines on the still intact material (glass:
    the web of cracks, dark hairlines; metal: the crease of the ring), rims along the cracks (the glint of a glass
    edge, fresh metal, fresh wood), the scorched edge of a laser cut, splinters of wood along the grain and burrs of
    torn metal, the mark round a point of impact (crushed glass, a scuffed ring on metal, a bruise on wood). The
    random ones are seeded from where they are on the material, so they stay the same when the mesh is rebuilt.
  - Shapes (`phys::add_sheet_mesh`, `SheetDesc::shape / hole / curve / dome`): the 4-8 grid of the rectangle clipped
    to a disc, a ring, a triangle or an L (triangles whose centroid is inside), the border nodes moved onto a curved
    outline where the triangles around them keep their shape, optionally bent into a cylinder (a half-pipe) or a
    spherical cap (a dome). `SheetDesc::kg_m2` gives the mass per area (a shape's area is not width x height).
  - Debug view (F3): the edges are drawn on the face of the sheet towards the camera (the nodes' plane is inside the
    drawn thickness): intact pale blue (unlike the beams' grey), the authored border brighter blue, plastically
    stretched or shortened orange, cracks and cuts red;
    "color by stress" shows the strain against the fracture strain.
  - **Force kernel** (`shell_kernel.cpp`). The kernel reads a hot copy of the sheet (`ShellKernel`), not the `Shell`
    records the topology code works on: blocks of 4 triangles field by field (corners, stiffness, rest lengths as
    vectors: 72 bytes per triangle), 8 bytes of state per triangle and a 32-byte record per bending hinge, updated
    incrementally when a crack or a bisection touches a triangle. Every hinge is evaluated by one of its two
    triangles, the one in the faster rate class. The nodes are only read: each triangle writes the forces on its
    corners and on the far wings of its hinges to 6 slots of its own, held until it is evaluated again (multi-rate),
    and each node sums its slots through a gather list. No write conflicts, so any number of threads can share a
    sheet, and the sums do not depend on which thread did what. NEON, 4 triangles per iteration: springs, face and the
    hinges accumulate into the same corner vectors (three corners of a hinge are in registers already, only its far
    wing is loaded); plastic bends, overload checks and clamped borders run per lane only where a vector test flags
    them. The hinge angle comes from a 256-interval atan table with linear interpolation (error < 1.3e-6 rad), square
    roots from the reciprocal-square-root estimate and two Newton steps, the fourth hinge gradient from the other
    three (they sum to zero). Sheets are kept in a Morton order of their triangles, the nodes in the order the
    triangles first touch them, and every triangle's corners are rotated so that the edges of the hinges it evaluates
    come first (most SIMD lanes are busy on edges 0 and 1); refinement appends at the end, so a sheet that grew by a
    fifth is renumbered between frames. 33 ns per triangle against 46.5 ns for the original kernel, and 2.1x fewer
    bytes brought into L1 per triangle (cache model below).
  - **Cache model** (`cache_sim.cpp`). The hardware counters of Apple silicon need root, so the kernel's memory accesses
    are replayed through a model of the caches (L1D 128 KB 8-way, L2 16 MB 16-way, 128-byte lines, LRU): the access
    streams walk the data structures in the kernel's order, for the current kernel and for the original layout (Shell
    records, per-class force arrays, separate edge and hinge loops). Reported: L1 hit rate, bytes brought from L2 and
    from memory per triangle, line utilisation (the share of the bytes of every line brought into L1 that were used
    before it was evicted), working set. For a 4 480-triangle sheet: 361 B per triangle from L2 at 80% line
    utilisation, against 761 B at 40% for the original layout.
  - Cloth for the older scenes is a grid of rope links that tears above a strain limit; the two-layer plate builder
    (`build_plate`) is kept for other uses.
- **Materials (Materials Lab).** A material is a target stiffness, capped by the explicit budget of its nodes (a
  light sheet is far softer than the real material), plus yield and fracture strains, ductile (metals stretch) or
  brittle with a random flaw per link (cracks run through the weak links: glass and concrete break into chunks, not
  dust). Sheets and solids get separate strains because their numerical stiffness differs so much.
- **Explicit stability.** Procedural bodies get a per-node budget of `sum(k)*dt²/m` and `sum(d)*dt/m`. Frames get
  their inertia inflated when their lever-arm stiffness is too high. Contact friction may stop a node's tangential
  motion within a step but never reverse it: RoR's friction scales with the normal force, not with the node mass, so
  a 6 g tape node squeezed under a truck tyre used to flip its velocity every substep with a growing amplitude.
  Node speeds are capped at 400 m/s, and a body with runaway nodes has only those nodes put back among their
  neighbours (`World::repair_nodes`); it used to be frozen as a whole (a tape line with all its stakes).
- **Grab tool.** The pull is a critically damped spring whose stiffness follows the grabbed body's mass, and its force
  is capped by the node's own mass as well (a 5 g cloth node pulled with 20 kN used to fly off at km/s). The grab strength
  (`Game::grab_strength` -> `SoftBody::grab_scale`, 0.01x - 100x) scales the stiffness and the cap together; the cap
  never exceeds 40000 m/s² times the node's mass. With a radius (`Game::grab_radius`) the grab takes a sphere round the
  picked node: the nodes of every body in it (`Game::grab_bodies`; each body's own stiffness from its mass), each pulled
  to the target and its offset from the centre, the body's pull shared by weight x mass, the weight (1 - (r/R)^2)^2 -
  all of it at the centre, none at the edge (`SoftBody::grab_nodes`, `grab_offsets`, `grab_w`; renumbered with the body's
  nodes, `SoftBody::remap_node_refs`, not dropped when a sheet refines or cracks).
- **Sleeping.** A body sleeps after 1 s below its sleep speed; a sleeping body is woken by a contact partner that
  moves faster than the partner's own sleep speed (soft bales keep rocking a little: a stack of them used to wake
  each other forever and creep apart). Imported stage props start asleep, as in RBR.
- **Scheduling.**
  - Each frame, bodies are grouped into islands, but only bodies that can actually touch. Each island runs all of
    its substeps as one task on the job system, biggest first.
  - Every short step of an island is a sequence of phases split into work items: internal forces (a body's beams,
    shocks and wheels as one item, its triangles in chunks of 128), the sheets' per-node gather, the contacts (the
    candidate search in chunks of triangles, the narrow phase in chunks of pairs, the response in order), static
    contacts and integration (nodes in ranges of 1024). Topology changes and the merging of results are serial and in
    a fixed order. Every item writes to slots of its own: the result is bitwise the same with any number of threads.
  - An island with a sheet, or with many bodies, opens a *work board* (`JobSystem::Board`, `Team`): its phases are
    posted there, the island's thread works on them too, and threads that ran out of islands take chunks. A chunk is
    claimed with one fetch_add on (generation, next chunk); the phase's function and size are in a descriptor tagged
    with its generation, so a claim that raced with the owner moving on gives up instead of running a stale function.
    The owner never waits for a helper to arrive, only for claimed chunks to finish: no deadlock, whatever the number
    of free threads. Idle helpers spin for 0.1 ms and then park on a futex (C++20 atomic wait) until something is
    posted: 12 threads spinning through an owner's serial parts took the cores from it and the OS preempted threads in
    the middle of chunks (frames up to 137 ms). The workers run at the user-interactive QoS class. Profiler zones in
    parallel phases accumulate in per-thread slots (no shared atomics).
  - While the physics itself makes the frames long (its last frame took more than half an average frame), a frame may
    take at most 1.25x the substeps of an average frame (+1): the simulation runs slow for a moment instead of doubling
    the next frame's work (the spiral that turned a 44 ms frame into 103 ms). Frames that are long for other reasons are
    always simulated in full.
  - Frame pacing: with vsync on a 120 Hz display the swap returns at uneven times (measured frame times alternate ~3 and
    ~13 ms) while the display shows frames evenly; the simulation steps by the average frame time pulled towards the real
    clock (no drift; a real hitch is made up over a few tenths of a second), `App::pace`. `BL_LIVE=1` makes scripted runs
    step like the interactive app (for measuring this); the profiling CSV has the raw frame time as `raw_dt_ms`.
  - Resting islands sleep. Contacts and wind wake them.
- **Terrain.** The heightfield is stored in 64 x 64 tiles; imported stages allocate only the tiles along the
  collision mesh (Undva: 10 km of road in 3 800 tiles instead of 190 M cells), and there is no ground elsewhere.

References for the design are summarised in the development notes. The main ones:

- Rigs of Rods sources;
- Teschner et al. 2003, *Optimized Spatial Hashing for Collision Detection of Deformable Objects*;
- Müller & Chentanez 2011, *Solid Simulation with Oriented Particles*;
- Kugelstadt & Schömer 2016, *Position and Orientation Based Cosserat Rods*;
- Macklin et al. 2019, *Small Steps in Physics Simulation* (why many small explicit substeps work);
- Parker & O'Brien 2009, *Real-Time Deformation and Fracture in a Game Environment*;
- Bridson, Marino & Fedkiw 2003, *Simulation of Clothing with Folds and Wrinkles* (hinge angle and gradient);
- Rivara 1984 (longest-edge bisection); O'Brien & Hodgins 1999, *Graphical Modeling and Animation of Brittle Fracture*;
- Fratarcangeli & Pellacini 2015 / Fratarcangeli, Tibaldo & Pellacini 2016, *Vivace* (parallel constraint work split
  so that no two items write the same node);
- Verlet 1967 / neighbour-list schedules; Morton 1966 and the space-filling-curve orderings used for cache locality of
  meshes and particles.

### Autopilot

The race-line follower behind Scene > "Autopilot" (rally stages, tape maze, RBR stages) takes its speed limits from the
line: lateral grip from the turn radius (tarmac 7 m/s², gravel 4.5 m/s²), crests from the vertical curvature
(v² < 0.9 g R, the wheels stay on the ground) and braking zones before both. It steers towards a look-ahead point
(shorter in hairpins, about 0.75 x the radius, so that it does not cut across walls on the inside) plus a cross-track
term that pulls it back onto the line. A stuck car reverses for a moment. With the BMW E36 it finishes every stage
(Verkiai with two bent beams, the others without damage): Rally Stage 1:37, Tape Maze 1:10, Verkiai 1:48, Undva 10:27,
Travanca 3:31, Fernet Branca 8:31.

## Performance

Apple M4 Max, 12 threads, Release build, 1600x900. Times are per 60 Hz frame (33 physics substeps per frame).

| scene | active nodes | physics ms (avg / p95) | CPU frame ms (avg) |
|---|---|---|---|
| Proving Ground (Audi Quattro, 442 nodes, ~600k-vertex meshes) | 442 | 0.38 / 0.41 | 2.1 |
| Forest (220 trees, wind) | 8 200 | 0.9 / 1.3 | 5.1 |
| Physics Lab | 2 200 | 0.5 / 0.8 | 2.9 |
| Vehicle vs Vehicle | 760 | 0.15 / 0.44 | 1.4 |
| Rally Stage (453 bodies, 68k grass tufts; wind only wakes bodies within 80 m of the camera) | 1 900 | 1.4 / 2.1 | 5.3 |
| Materials Lab (9 sheets hit at once at 2.1 s: the 2 s after, then 4 - 6 s, then at rest; coarsening, rigid pieces) | 12 000 / 10 000 / 2 000 | 6.3 / 10.1, 6.0 / 8.4, 2.5 / 2.6 | 9.0, 9.0, 4.9 (4 frames of 18 - 22 at the impact) |
| Physics Lab, cannonballs at 60 m/s into the steel sheet every 0.4 s | 4 000 | 2.1 / 4.7 | 4.9 |
| Sheet Run (a car at 70 km/h through three lead sheets; max 7.9 ms, p99 6.4 ms) | 2 000 - 6 000 | 2.3 / 5.0 | 4.5 |
| Tape Maze (60 tape sections, wind wakes the ones within 60 m of the camera) | 6 400 | 1.1 / 1.4 | 3.4 |
| RBR: Verkiai (0.35 M triangles of scenery) | 490 | 0.53 / 0.56 | 3.8 |
| Stress: 16 vehicles | 6 800 | 1.5 / 2.5 | 7.1 |
| Stress: Demolition derby | 5 100 | 1.8 / 2.6 | 6.7 |
| Stress: 512 crates (one big pile) | 4 000 | 7.2 / 8.4 | 12 |
| Stress: Windy forest (400 trees) | 19 700 | 2.1 / 2.4 | 9.4 |
| Stress: Barrel pile (18 drums of 480 triangles, 4 short steps; with FEM rings) | 4 400 | 8.9 / 11.4, 16.0 / 19.5 | 9.8, 17.4 |
| Stress: Bridge convoy | 1 900 | 1.9 / 2.7 | 5.2 |

Sheet Run before the sheet optimisations (same machine): 6.9 ms average, 22.6 ms p95, 43.6 ms max, 51 of 571 frames over
16.7 ms (103 ms max when stepping by the real frame time); now 2.3 / 5.0 / 7.9 ms and none (9-12 ms max in real time).

Vehicle rendering: flexbodies are skinned on the CPU in parallel and streamed; rigid parts (rims with tyres, props,
steering wheels) are static GPU meshes drawn with a model matrix, so a 50k-triangle rim costs nothing per frame.

The biggest remaining cost is the pile of crates. Its contact solving is sequential by design: RoR contacts depend
on the forces accumulated so far.
