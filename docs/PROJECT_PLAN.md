# Project Plan: Cooperative Multi-Robot Warehouse Logistics

University Mobile Robotics project, team of four. This file is the shared reference for the
goals, the current status, the architecture hooks and the to-do list. Update it in the same
Pull Request as the work it describes.

**Legend for status**

| Mark | Meaning |
|---|---|
| ✅ **Verified** | We ran it on our own machine and saw the result. |
| 📄 **Upstream** | The original repository (or its README) says it exists. We have not checked it ourselves. |
| 🗓 **Planned** | Only an idea or a plan. Nothing is implemented. |

Do not move an item to ✅ until a team member has run it. A README claim is not a test.

---

## 1. Goal

Simulate a warehouse where two autonomous forklifts with **different capabilities** cooperate
to move pallets and other loads. A central task manager assigns work using the capabilities of
each robot and the properties of each load.

Stack: Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, Nav2, SLAM Toolbox, robot_localization,
ros2_control, gz_ros2_control. Reuse existing software wherever possible.

Base repository: `keshavwxyz/ros2-gazebo-multi-forklift` (Apache-2.0). Our fork:
`borgakk/ros2-collaborative-forklift` (the `origin` remote of this workspace).

## 2. Scenarios

| | Scenario | Summary | Depends on |
|---|---|---|---|
| A | Individual transport | The task manager gives a load to a suitable forklift (weight, size, location, destination, shelf height). A light pallet on a low shelf goes to the fast robot. A heavy pallet or a high shelf needs the high-lift robot. The slow robot can also do long trips. | capability model, scheduler |
| B | Pallet handover | The high-lift robot takes a pallet from a high rack to a **transfer station**. The fast robot picks it up and delivers it. The manager decides if the handover is worth it (distances, availability, completion time). Sequence: place, release, pick up, verify. | A, transfer stations |
| C | Cooperative oversized load | Both forklifts carry one long object at the same time (front-back support). Their motion is synchronized to avoid rotation, oscillation and instability. | robot models, load attachment, cooperative controller |
| D | Dynamic obstacles | Walking and running workers (Gazebo Actors). The forklifts detect them, slow down, stop or replan. | warehouse, sensors |

Scenario C is the most ambitious part. Start it only after A works and the two robots
coordinate reliably.

## 3. Target robots

| | Robot 1: high-capacity | Robot 2: fast |
|---|---|---|
| Drive | Differential | 4-wheel Mecanum (lateral motion) |
| Speed | Lower | Higher |
| Max lift height | High (about 2 m or more, from shelf design) | Low (for example 0.4 m) |
| Load capacity | Higher | Lower |
| Role | Heavy loads, high racks, long trips allowed, leader in cooperative carry | Light loads, fast transport, follower in cooperative carry |
| Config files (planned) | `forklift_high_lift.yaml` | `forklift_mecanum.yaml` |

Both robots carry: front and rear LiDAR, RGB camera, IMU, wheel odometry, ros2_control for
drive and lift.

The capacity limit must exist in **two places**: in the task manager (a software constraint)
and in the physics (realistic mass, inertia, actuator effort limits). A limit that exists only
in the scheduler is not enough.

Use a common xacro with per-robot parameter files and drive configuration. Do not copy the
robot description files.

## 4. Status of the baseline (Phase 1)

Upstream project layout (packages in `src/`):

| Package | Role |
|---|---|
| `forklift_interfaces` | messages, services, actions |
| `forklift_description` | parametric xacro, `forklift_params.yaml`, ros2_control block |
| `forklift_simulation` | Gazebo world `warehouse.sdf`, sensors, bridge, QR load models |
| `forklift_control` | controllers, `twist_mux`, lift action server, pickup/place skills, rear safety |
| `forklift_localization` | EKF (wheel odometry + IMU) |
| `forklift_perception` | QR detection, decoding, PnP pose |
| `forklift_navigation` | Nav2 / AMCL config, saved map, footprint manager |
| `forklift_task_manager` | mission configuration and per-robot mission state machine |
| `forklift_fleet` | fleet coordinator, task registry, allocator, conflict handling, fleet agent |
| `forklift_bringup` | `sim.launch.py`, `multi_sim.launch.py`, `robot.launch.py`, `mock.launch.py` |
| `forklift_test` | mock and Gazebo integration tests |

| Item | Status | Note |
|---|---|---|
| Remote and branch | ✅ | `origin` = our fork, branch `main`. No `upstream` remote yet. |
| Clean build in an isolated ROS environment | ✅ | 11 packages build, `--symlink-install`. |
| `scripts/check_env.sh` | ✅ | All required dependencies present. RTX 4070 detected. |
| Two-forklift mission, no GUI | ✅ | `MISSION_COMPLETE: delivered 3/3 (BOX_A, BOX_B, BOX_C) in 377 s; 0 conflicts` (simulation time, about 10 min wall-clock, rough real-time factor 0.6). |
| Gazebo GUI starts and shows the warehouse | ✅ | Window opens, mission runs. Visual quality on the hybrid Intel/NVIDIA laptop still to be confirmed (see section 8). |
| Physical pickup and placement (forks lift by contact) | ✅ / 📄 | The log shows placement 4–5 cm from target. The "no teleport" guarantee is an upstream claim (`test_no_simulator_shortcuts.py`). |
| Nav2, AMCL, EKF, QR perception, rear safety, fleet coordinator | ✅ (end to end) / 📄 (details) | They worked in the run above. Details and tuning are upstream claims. |
| Mecanum wheel dynamics in Gazebo (single robot, free drive, no GUI) | ✅ | `drive_type: mecanum`: 4 large driven wheels, anisotropic friction. Forward, lateral and diagonal motion and in-place turn agree with ground truth within about 1 cm. Branch `feature/mecanum-drive`. Not yet run with a load, with Nav2 or in the fleet mission. |
| Three robot types: `standard`, `high_lift` (2 m lift, tall mast, slow), `mecanum` (fast, 0.45 m lift, short mast, blue) | ✅ | Selected with `robot_type:=` (`robot.launch.py`) and `robot_type.<id>` in `fleet.yaml`. Lift to 3.0 m (the earlier setting) verified in Gazebo with no tilt; the 2.0 m setting and the faster lift speeds are checked by unit tests only, not yet in Gazebo; Mecanum lift limited to 0.45 m (a 0.6 m request is rejected); Mecanum drives 1.2 m/s accurately. Details: `docs/robot_model.md`. |
| Existing 3-box mission with `forklift_1 = high_lift` and `forklift_2 = mecanum` (no GUI) | ✅ | `MISSION_COMPLETE: delivered 3/3 in 216 s; 0 conflicts` after the faster docking settings (before: 315–327 s; standard pair, original settings: 377 s). Docking was about 85 % of the time of one box: pickup 97 s → 47 s and placement 98 s → 50 s (log seconds, one run; the first pickup of forklift_2 took 76 s). The docking speeds are now approach 0.30, insert 0.15, reverse 0.25 m/s (`load_handler.yaml`), and `platform_face_margin` is 0.02 m (was 0.04). Real final pose of the boxes (Gazebo ground truth): 1.9 / 3.2 / 4.8 cm from the target (before: 5.0 / 6.1 / 5.1 cm); the offset in y is about −1.7 cm (before −4 cm). The camera-measured placement error was 4 / 8 / 5 cm (tolerance 15 cm). Earlier investigation: the camera values of 6–9 cm were not caused by the robot types or by the GPU driver. |
| Free drive of the existing warehouse without mission (`sandbox.launch.py`) | ✅ | No Nav2, no mission. GUI view still to be checked by a team member. |
| Unit tests of `forklift_description`, `forklift_control`, `forklift_simulation`, `forklift_navigation`, `forklift_fleet` | ✅ | 302 tests pass on the `feature/mecanum-drive` branch (robot type tests included). The other packages and tier 3 were not run. |
| `colcon test` (all packages, tier 1 and tier 2) | 🗓 | Not run by us yet for all packages. |
| Gazebo integration tests (tier 3) | 🗓 | Not run by us yet. |
| RViz bringup (`rviz:=true`) | 🗓 | Not tried yet. |

### Where the code is homogeneous today (hooks for Phase 2)

These points assume two **identical** differential-drive robots. They are the places to change.

| File | What to change |
|---|---|
| `src/forklift_bringup/launch/robot.launch.py` | One `controllers.yaml`, one `forklift_params.yaml`, one `nav2_params.yaml` for every robot. Add per-robot config selection. The controller name `forklift_base_controller` and the diff-drive `tf_frame_prefix_enable` fix are hard-coded. |
| `src/forklift_bringup/launch/multi_sim.launch.py` | Builds robots from the `robots` list in `fleet.yaml`. Pass a per-robot `config` here. |
| `src/forklift_fleet/config/fleet.yaml` | Robot list, home poses, coordination policy. No capability fields (speed, lift height, payload). |
| `src/forklift_fleet/forklift_fleet/allocator.py` | Cost-based allocation by travel distance only. Add capability constraints. |
| `src/forklift_description/config/forklift_params.yaml` | Lift: `max_travel: 1.00 m`, `max_velocity: 0.20 m/s`, `max_effort: 10000 N`. Single source for geometry and mass. |
| `src/forklift_description/urdf/forklift_body.xacro`, `forklift_ros2_control.xacro` | Mast, carriage, wheels, joints, hardware interfaces. |
| `src/forklift_control/config/controllers.yaml` | Differential drive controller. |
| `src/forklift_navigation/config/nav2_params.yaml` | Footprint and limits for the differential robot. |
| `src/forklift_task_manager/config/warehouse_mission.yaml` | Pickup poses, destinations, loads (no weight or lift height yet). |
| `src/forklift_simulation/worlds/warehouse.sdf`, `forklift_navigation/maps/` | World and saved map (321 x 242 cells at 0.05 m, about 16 x 12 m). |

> Items in the table were read, not all tested. Read each file fully before changing it.

## 5. Architecture direction

- **High level:** fleet coordinator and task manager (allocation, handover decision, cooperative
  orchestration).
- **Low level:** classical controllers (Nav2, formation control, optionally MPC).
- **Machine learning:** later, and only for high-level choices (task allocation, route or
  strategy selection). Do not train RL to control a physically coupled load in Gazebo.
- **VDA 5050:** optional communication layer on top of the fleet manager. It is not a motion
  controller.

Shared interfaces (topics, actions, config formats) must be agreed **before** two people build
code that depends on each other. Put them in `docs/` and `forklift_interfaces`.

## 6. Team and ownership

| Member | Area | Main work |
|---|---|---|
| 1 | Fleet management | Scheduler, capability model, assignment, handover, cooperative orchestration, optional VDA 5050 |
| 2 | Robot models and control | High-lift and low-lift robots, Mecanum conversion, ros2_control, physics, load attachment |
| 3 | Warehouse environment | Racks, stations, load placement, maps, generation tools |
| 4 | Dynamic environment and testing | Actors, obstacle scenarios, metrics, data collection, plots |

Names and the exact split can change. Less experienced members must get real programming tasks
(see "Good first tasks" in section 7), not only documentation.

## 7. To-do list by phase

### Phase 1: Working baseline (all)
- [x] Verify remote, branch, ROS, Gazebo
- [x] Clean isolated build, fix workspace overlay problem (`~/forklift_env.sh`)
- [x] Run the two-forklift mission without GUI (3/3 delivered)
- [x] Gazebo GUI opens
- [ ] **Every team member** repeats the baseline on their own machine
- [ ] Confirm the GUI looks correct on the hybrid Intel/NVIDIA laptop
- [ ] Run `colcon test` (tier 1 and 2) and record the result
- [ ] Add the `upstream` remote and keep attribution (see "Licence", section 9)
- [ ] Try `rviz:=true`
- [ ] Define branch rules and shared interfaces (section 10)

### Phase 2: Heterogeneous forklifts (Member 2)
- [x] Per-robot parameter overlay and selection in `robot.launch.py` (`drive_type`, `config/forklift_mecanum.yaml`); [ ] the same for the high-lift robot, and selection from `fleet.yaml` in `multi_sim.launch.py`
- [x] Different max speeds per robot (the Mecanum robot: Nav2 1.0 m/s, standard 0.4 m/s); [ ] make the pickup/place approach speeds depend on the robot type
- [x] High-lift robot: `max_travel` 2.0 m, mast 2.85 m, heavier counterweight, lift effort checked; [ ] check it against real shelf heights (the `east_rack` shelves are at 1.0 m and 1.8 m)
- [x] Fast robot: lift range 0.45 m and rated load 150 kg
- [x] Payload capacity is a parameter in the physics (lift actuator effort, `rated_load`); [ ] use it in the task manager / allocator
- [x] Mecanum robot: wheels and joints in xacro, hardware interfaces, `mecanum_drive_controller`, odometry, EKF (done, see `docs/robot_model.md`)
- [x] Mecanum robot: footprint, faster Nav2 limits, pickup/place and the fleet mission (one run)
- [ ] Mecanum robot: Nav2 controller that uses lateral motion (omnidirectional model), lateral-motion safety
- [x] Decide and document the Mecanum simulation approach: sphere wheels with anisotropic friction (the Gazebo mecanum demo method)
- [ ] Validate each robot alone, then both together; keep the original diff-drive mission working

### Phase 3: Warehouse expansion (Member 3)
- [ ] Racks with several shelf heights and correct collision geometry
- [ ] Loading zones, delivery zones, transfer stations
- [ ] Pallets and loads with different weights and sizes (extend mission config with weight, size, required lift height)
- [ ] Narrow and wide aisles, static obstacles
- [ ] Generate repeated objects with a script instead of copying SDF by hand
- [ ] New saved map; check that both robots navigate it
- [ ] Check that loads can really be picked up and placed on each shelf height

### Phase 4: Task allocation (Member 1)
- [ ] Capability model: speed, lift height, payload, drive type for each robot
- [ ] Hard constraints: payload, shelf height, dimensions
- [ ] Cost-based scheduler (travel time, availability)
- [ ] Handover decision: direct delivery vs handover at a transfer station
- [ ] Handover sequence: place, release, pick up, verify
- [ ] Metrics: completion time, idle time, distance (shared with Member 4)
- [ ] Compare scheduling strategies

### Phase 5: Cooperative transport (Members 1 and 2)
- [ ] Long box supported at two points; first prototype with simplified joints
- [ ] Synchronized motion without a load
- [ ] Straight-line transport with a connected load
- [ ] Coordinated turning
- [ ] Obstacle avoidance while carrying
- [ ] Later, optional: friction-based support

### Phase 6: Dynamic obstacles and evaluation (Member 4)
- [ ] Check what Gazebo Actors do (collision, sensor visibility) before relying on them
- [ ] Walking and running workers on aisle and crossing routes
- [ ] Verify detection by LiDAR and the Nav2 reaction (slow, stop, replan)
- [ ] Run repeated experiments; record travel time, success rate, positioning error
- [ ] Plots and a results table for the report

### Phase 7: Optional
- [ ] Reinforcement learning at the high level
- [ ] More advanced cooperative controller (MPC)
- [ ] VDA 5050 layer (MQTT)
- [ ] Extra warehouse scenarios

### Good first tasks (for less experienced members)
- [ ] Write a script that prints the mission result and timing from a run log
- [ ] Add weight, size and lift-height fields to the load model and the mission YAML, with a unit test
- [ ] Write the rack and pallet generator script for the warehouse (SDF output)
- [ ] Write a metrics logger node (task start, finish, distance) that saves a CSV
- [ ] Make a plot script for the CSV files

## 8. Technical risks and decisions

| Topic | Risk or decision |
|---|---|
| **Cooperative load physics** | Friction-only carry is realistic but unstable (sliding, tilting). Rigid fixed joints to two independent robots over-constrain the system. **Start with joint-based connection** (rigid load, compliant or partly free joints). Load inertia and control errors must still act on the simulation. Move to friction later if time allows. |
| **Mecanum simulation** | A physically accurate roller model is hard and expensive. Choose between a simplified omnidirectional model (stable, fast) and real wheel contact. Document the limits of the chosen model. Test the simple model first. |
| **Leader-follower control** | The diff-drive robot follows the route. The Mecanum robot uses lateral motion to keep its support position. Two independent Nav2 controllers on one rigid load is **not** acceptable; a cooperative controller is needed (load pose, support points, tilt, constraint errors). |
| **Support layout** | Front-back first. Changing layout while the load is lifted is out of scope. |
| **Gazebo Actors** | Actors are animated entities. Do not assume collision or LiDAR detection. Verify, and if needed add an invisible collision body or sensor-visible model that follows the actor. |
| **High lift** | A taller mast changes the centre of mass, tipping stability, lift effort and collision geometry. |
| **Performance** | Simulation ran at about 0.6x real time (two robots, headless, rough measure). Avoid heavy meshes, high camera resolution and many actors. Physics speed depends on the CPU and physics settings, not only the GPU. |
| **GPU setup** | The laptop has Intel Iris Xe and an NVIDIA RTX 4070. The log shows `libEGL ... failed to create dri2 screen` warnings. Check that rendering is on the intended GPU. |
| **Hard-coded homogeneity** | The code assumes two identical robots (section 4). Changing one file alone will not be enough. |
| **Overlapping ROS workspaces** | See section 9. |
| **Scheduler vs reality** | A feasible plan in the scheduler can still fail in the physics (lift, reach, stability). Test constraints in the simulation. |

## 9. Development environment

- Workspace: `~/warehouse_ws`. Another ROS project exists in `~/learn_rl/linorobot2_ws`. **Do not modify it.**
- `~/.bashrc` may source other workspaces. Overlays can hide or replace packages and can get stored inside `install/setup.bash` at build time (this caused `Package 'forklift_bringup' not found`).
- Use a clean environment. Example helper (kept outside the repo):

  ```bash
  # ~/forklift_env.sh  - reset ROS variables, then source only ROS Jazzy + this workspace
  source ~/forklift_env.sh
  ```

- Build: `colcon build --symlink-install`, **only** from a shell that has only `/opt/ros/jazzy` sourced. Source `install/setup.bash` afterwards.
- Run: `ros2 launch forklift_bringup multi_sim.launch.py gui:=true start_mission:=true`
- Stop: `scripts/stop_sim.sh`
- If Gazebo closes during start-up, the whole bringup shuts down (`on_exit=Shutdown`). Wait 30–60 s for the GUI. Do not press Ctrl+C.
- **Licence:** the project is Apache-2.0. Keep the `LICENSE` file and the upstream copyright notices, mark changed files as changed, and credit the upstream project in our README.

## 10. Git workflow

- One shared fork, four members, **feature branches**, merge to `main` by Pull Request.
- `main` must always build and run the baseline mission.
- Branch names: `feature/mecanum-drive`, `feature/high-lift`, `feature/warehouse-expansion`,
  `feature/task-allocation`, `feature/cooperative-transport`, `feature/dynamic-actors`.
- Before a Pull Request: build is clean, `colcon test` (tier 1 and 2) passes, the baseline mission
  still completes.
- Shared interfaces (topic names, actions, config schemas) are agreed in `docs/` first.
  A change to a shared interface needs a note to the whole team.
- Documentation is part of the work: update this file and the relevant `docs/*.md`.

## 11. Open questions

- How high are the racks? The final lift height of Robot 1 depends on this.
- What are the real payload values for each robot and for each load class?
- Where are the transfer stations, and how large is the warehouse?
- Mecanum simulation: simplified model or real wheel contact?
- Which metrics go into the final report?
- Is VDA 5050 required, or only optional if time remains?
- Deadlines and milestones of the course (add dates here).
