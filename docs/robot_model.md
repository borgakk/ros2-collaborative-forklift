# Robot Model (`forklift_description`)

Status: **validated, and operational in Gazebo**. The static checks, `check_urdf`, the
URDF→SDF conversion and a live mock-hardware ros2_control test all pass.

![Forklift collision geometry](images/forklift_views.png)

*To make this image again, run `scripts/render_model_views.py` (with the workspace sourced).
The script draws the URDF collision geometry, and adds the LiDAR scan plane, the vertical FOV
of the camera and the center of mass.*

## Configuration
All the dimensions, masses and actuator limits are in
[`src/forklift_description/config/forklift_params.yaml`](../src/forklift_description/config/forklift_params.yaml).
To change the design, copy the file and give it to xacro. You do not need to change the xacro:

```bash
xacro src/forklift_description/urdf/forklift.urdf.xacro params_file:=/abs/path/my_params.yaml
```

| xacro argument | Default | Purpose |
|---|---|---|
| `params_file` | package `config/forklift_params.yaml` | YAML with dimensions and masses (absolute path) |
| `sim_backend` | `mock` | ros2_control plugin: `mock` (`mock_components/GenericSystem`) or `gazebo` (`gz_ros2_control/GazeboSimSystem`). Other values stop the generation. |
| `ros2_control` | `true` | Include the `<ros2_control>` block |
| `extensions` | empty | Absolute path to a xacro for a specified simulator (Gazebo sensors and plugins). This keeps this package independent of the simulator. |

## Kinematic layout
* **Drive:** differential. Two driven wheels (r = 0.18 m, track 0.80 m) on the front axle. The
  wheel *collision* is a sphere, and the visual is a cylinder. In DART, cylinder contacts were
  at the edge of the rim and caused a 13% yaw-odometry error
  ([sensor_setup.md](sensor_setup.md)). The wheels are directly below the mast.
  `base_footprint` and `base_link` are on this axle. Thus the robot turns in position around a
  point 1.38 m behind the fork tips (reason: D2 in [system_design.md](system_design.md)).
* **Support:** two rear spheres (r = 0.10 m) at x = −1.00 m (wheelbase 1.00 m) on fixed joints.
  In the simulation, they have almost no friction. Thus they operate as free casters with no
  swivel-joint jitter. *Approximation:* real casters have rolling resistance and swivel lag.
* **Lift:** `fork_lift_joint`, prismatic along z between `mast_link` and `fork_carriage_link`.
  Joint position *q* ∈ [0, 1.00] m. The height of the top surface of the forks above the floor
  is 0.05 + *q*. Velocity limit 0.20 m/s, effort limit 10 kN (≥1.5× the rated 500 kg and
  110 kg of carriage and forks), viscous damping 200 N·s/m.
* **Forks:** two L-shaped tines, 1.00 m long, 0.10 m wide, 0.04 m thick, 0.40 m apart (center
  to center), rigidly attached to the carriage. There is no side-shift and no tilt (mast tilt is
  not included).

| Joint | Type | Parent → child | Axis | Limits |
|---|---|---|---|---|
| `left/right_drive_wheel_joint` | continuous | `base_link` → wheel | +y | 6 rad/s, 400 N·m |
| `fork_lift_joint` | prismatic | `mast_link` → `fork_carriage_link` | +z | 0–1.00 m, 0.20 m/s, 10 kN |
| all other joints | fixed | refer to [coordinate_frames.md](coordinate_frames.md) | | |

## ros2_control interfaces
| Joint | Command | State |
|---|---|---|
| `left_drive_wheel_joint`, `right_drive_wheel_joint` | `velocity` (±6 rad/s) | `position`, `velocity` |
| `fork_lift_joint` | `velocity` (±0.20 m/s); position goals go through the `SetForkHeight` action | `position`, `velocity`, `effort` (load sensing) |

## Mass properties
| Link | Mass [kg] | Inertia model |
|---|---|---|
| `base_link` (chassis) | 600 | solid box 1.35×0.64×0.50 |
| `counterweight_link` | 400 | solid box 0.30×0.64×0.30 |
| `overhead_guard_link` | 40 | **approximation:** one solid box around the posts and the roof |
| drive wheels | 2 × 30 | solid cylinder |
| casters | 2 × 10 | solid sphere |
| `mast_link` | 120 | **approximation:** one solid box around the two uprights |
| `fork_carriage_link` | 60 | **approximation:** one solid box around the two bars |
| forks | 2 × 25 | tine box and shank box (parallel-axis theorem), mass divided by volume |
| sensors | 0.35 total | small boxes and cylinders |
| **Total** | **1350.3** | center of mass at x = −0.47, z = 0.54 m (no load, forks down) |

The masses are engineering estimates for a compact electric pallet stacker. They are not
datasheet values. The rated load is 500 kg. The mission boxes are much lighter (~30 kg).

**Stability checks** (automated): the center of mass stays in the support polygon with a margin
of 0.15 m or more, with no load and with 50 kg at all lift heights. With the full rated load of
500 kg at 1.0 m lift and 0.5 m/s² braking, the center of mass stays 0.03 m or more behind the
tip line of the front axle.

## Overall dimensions
Length 2.53 m (x ∈ [−1.15, 1.38], counterweight to fork tips), width 0.92 m (across the drive
wheels), height 2.15 m (mast top). The Nav2 footprint uses these dimensions.

## Validation
```bash
source scripts/env.sh && colcon build --symlink-install --packages-select forklift_description
source install/setup.bash

# 1. Generate the URDF, then check the syntax and the tree
xacro src/forklift_description/urdf/forklift.urdf.xacro > /tmp/forklift.urdf
check_urdf /tmp/forklift.urdf

# 2. URDF→SDF conversion of Gazebo
gz sdf -p /tmp/forklift.urdf > /dev/null && echo "SDF OK"

# 3. Automated tests (structure, inertia, limits, interfaces, frames, stability, lint,
#    live mock ros2_control + TF launch test)
colcon test --packages-select forklift_description && colcon test-result --verbose

# 4. Visual inspection with lift and wheel sliders
ros2 launch forklift_description display.launch.py
```

The tests (`test/test_urdf_model.py`, 40 cases, and `test/test_mock_control_interfaces.py`)
check these items:
* Generation for each backend; backends that are not valid are refused; `check_urdf`; the
  `gz sdf` conversion keeps the movable joints.
* The exact set of links and joints, one tree with the root `base_footprint`, only collision
  primitives.
* Each physical link has visual, collision and inertial data. Each inertia tensor is positive
  definite and satisfies the triangle inequality.
* Joint axes and limits. The ros2_control command limits agree with the URDF limits.
* The robot stays on its wheels and casters. Fork height = 0.05 + *q*. Fork reach and spacing.
* Clearances: the carriage does not touch the mast. The LiDAR plane does not touch the tines,
  the carriage or the mast. The camera optical frame obeys REP-103 and sees the fork tips and a
  QR code at 1.0–2.5 m. The lifted carriage stays out of the top of the image.
* Center of mass and tip-over checks. YAML overrides go to the geometry and to the ros2_control
  limits.
* Live: `controller_manager` loads the mock hardware, the component is `active`, all 3 command
  interfaces and 7 state interfaces are present and not claimed, and `robot_state_publisher`
  publishes all 16 child frames.

## Robot types

The fleet has two different robots. The standard robot is the original differential forklift
and stays unchanged. Each type is the same xacro with an overlay file that replaces sections of
`forklift_params.yaml` (xacro argument `params_overlay`). `robot.launch.py` selects the type with
`robot_type:=standard|high_lift|mecanum`. `multi_sim.launch.py` reads the type of each robot from
`robot_type.<id>` in `forklift_fleet/config/fleet.yaml` (use `robot_types:=standard` there for two
standard robots).

| | `standard` | `high_lift` (robot 1) | `mecanum` (robot 2) |
|---|---|---|---|
| Overlay | none | `forklift_high_lift.yaml` | `forklift_mecanum.yaml` |
| Drive | differential, 2 casters | differential, 2 casters | 4 Mecanum wheels |
| Navigation speed (Nav2) | 0.4 m/s | 0.4 m/s | 1.0 m/s (hard limit 1.5 m/s) |
| Lift travel / max fork top | 1.00 m / 1.05 m | **2.00 m / 2.05 m** | 0.45 m / 0.50 m |
| Mast height | 2.15 m | **2.85 m** | **1.20 m** |
| Lift speed (hard / profile) | 0.20 / 0.15 m/s | 0.60 / 0.50 m/s | 0.50 / 0.40 m/s |
| Rated load | 500 kg | 500 kg | 150 kg |
| Lift actuator effort | 10000 N | 10000 N | 4000 N |
| Counterweight | 400 kg | 600 kg | 400 kg |
| Body color | orange | orange | blue |

The three robot types have the same sensors, forks, chassis and camera. The high-lift robot has a
heavier counterweight and mast, so that the forward tipping margin at 2 m with the rated load
(11.6 cm, with 0.5 m/s² braking) is better than that of the standard robot (6.3 cm).
`forklift_description/test/test_robot_types.py` checks the lift range, the capacity order, the
mast height, the stability and that each type can do the existing mission (the platforms are at
0.25 m). The overlay `navigation` and `lift_controller` sections change the Nav2 parameters and the
lift profile of one robot (`robot.launch.py`). Capacity and lift range are **not yet used by the
fleet allocator**: it can assign a load to a robot that cannot lift it.

## Mecanum variant (`drive_type: mecanum`)

The fast robot has 4 driven wheels. It uses the same xacro as the differential robot. The
overlay `config/forklift_mecanum.yaml` replaces the `drive_wheel` section and sets
`drive_type: mecanum` (xacro argument `params_overlay`). `robot.launch.py` selects it with
`drive_type:=mecanum`.

| Item | Differential (default) | Mecanum |
|---|---|---|
| Wheels | 2 driven front wheels + 2 rear casters | 4 driven wheels of the same size (r = 0.18 m), at x = 0 and x = −1.00 m, y = ±0.40 m |
| Joints | `left/right_drive_wheel_joint` | `front/rear_left/right_drive_wheel_joint` (continuous, axis +y) |
| Base controller | `diff_drive_controller` (`controllers.yaml`) | `mecanum_drive_controller` (`controllers_mecanum.yaml`) |
| Command topic | `forklift_base_controller/cmd_vel` | `forklift_base_controller/reference` (`TwistStamped`: `linear.x`, `linear.y`, `angular.z`) |
| Odometry topic | `forklift_base_controller/odom` | `forklift_base_controller/odometry` (the launch file changes `odom0` of the EKF) |

**Dynamics.** The rollers are not modelled. Each wheel is a sphere. In Gazebo, the contact has
an anisotropic friction: `mu` (high) along the friction direction `fdir1` and `mu2` (about
zero) along the roller axis. `fdir1` is at 45° to the wheel plane: (1, −1) for front-left and
rear-right, (1, 1) for front-right and rear-left. The direction is expressed in
`base_footprint`, not in the wheel frame, because the wheel link rotates
(`forklift_simulation/urdf/forklift_gazebo.xacro`). This is the method of the Gazebo
mecanum demo (`gz-sim` `worlds/mecanum_drive.sdf`). The friction extension must use the nested
`<collision><surface>` form: the plain `<fdir1>` URDF tag loses the frame in the SDF.

**Pivot.** The wheel rectangle has its center 0.5 m behind `base_footprint` (which is above the
front axle). `kinematics.base_frame_offset.x = wheelbase / 2` makes the controller turn the
robot about `base_footprint`, as the differential robot does. The odometry has the same frame.

**Measured in Gazebo (ground truth, no load, 0.3 to 0.4 m/s, no GUI; repeat it with
`scripts/drive_test.py`):** forward, backward and lateral distance agree with the
command within 1 cm. A turn of 2.0 rad has 0.00 m of position drift. The wheel odometry agrees
with the ground truth. The EKF fuses `vx`, `vy` and `vyaw` of the odometry.

### Limitations of the Mecanum variant
* The friction model is ideal: there is no roller slip, no roller compliance and no
  load-dependent traction. Real Mecanum wheels slip more, mainly in lateral motion. The result
  is better than a real vehicle. Tune `mecanum.mu` and `mecanum.mu2` if a more realistic
  behaviour is necessary.
* The rear LiDAR protects only reverse motion (`linear.x < 0`). There is no protection for
  lateral and diagonal motion.
* Nav2 uses the regulated pure pursuit controller. It does not command lateral motion. The
  robot drives like a differential robot (faster, with the Mecanum footprint
  `footprints_mecanum.yaml`). The lateral motion works with the keyboard and with direct
  commands. A Nav2 controller that uses it (for example MPPI) is not set up yet.
* If `vx`, `vy` and `wz` are all at their limits at the same time, the wheel speed limit is
  exceeded. Navigation must use smaller combined limits.
* The Mecanum robot did the existing mission in the fleet together with the high-lift robot
  (`docs/PROJECT_PLAN.md`, section 4). The pickup/place skills still use the same slow approach
  and insertion speeds.

## Known limitations
* The caster behavior is an approximation (spheres with no friction).
* There is no mast tilt and no fork side-shift.
* The box-shaped inertia approximations for the guard, the mast and the carriage give some
  moments that are a little too large. This has no effect on a slow vehicle, but dynamic
  studies need a better model.
* By design, this package does not contain Gazebo sensor and plugin tags. The `extensions`
  argument adds them.
