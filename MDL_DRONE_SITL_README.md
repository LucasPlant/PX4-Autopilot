# Context: Custom `gz_mdl_drone` SITL & 1-DOF Gimbal Architecture

**Document Purpose:** This document provides the environmental and architectural truth for the custom `gz_mdl_drone` simulation target. The agent MUST reference this to understand how the simulation physics, the ROS 2 bridges, and the URDF have been modified.

## 1. The Core Architectural Shift: Bypassing PX4

* **Old Architecture:** The ROS 2 HAL node published quaternions via `px4_msgs/GimbalManagerSetAttitude` to the PX4 firmware, which then mixed the commands and sent them to Gazebo.
* **New Architecture:** We are explicitly **bypassing PX4** for gimbal control to perfectly mirror our physical SBC-to-Servo hardware setup. The ROS 2 HAL node now publishes a standard `std_msgs/Float64` directly to a Gazebo `JointPositionController` plugin via `ros_gz_bridge`.
* **Rule:** Do not import, use, or attempt to configure any `px4_msgs` related to the gimbal or mount manager. PX4 handles flight; Gazebo directly handles the camera servo.

## 2. The Custom SITL Target (`gz_mdl_drone`)

We have created a custom simulation target to sandbox our physical changes without breaking the default PX4 models.

* **Launch Command:** `make px4_sitl gz_mdl_drone`
* **Airframe ID:** `4022_gz_mdl_drone`. This file configures the PX4 firmware to ignore the gimbal and points the simulator to spawn our custom SDF model. It also disables QGC/RC failsafes (`NAV_DLL_ACT=0`, `NAV_RCL_ACT=0`) so the simulation runs headlessly, and disables DDS time sync (`UXRCE_DDS_SYNCT=0`) so ROS 2 receives pure Gazebo sim time.
* **Model Name:** Gazebo spawns this instance under the model name `mdl_drone` (resulting in the instance name `mdl_drone_0`).
* **Do not use `4001_gz_x500` directly** for any headless/ROS instance — it defaults to `NAV_DLL_ACT=2` (RTL on datalink loss) which immediately triggers a failsafe with no QGroundControl present.

## 3. Gazebo Physics Modification (The Welded SDF)

To perfectly emulate the rigid STS3215 servo, the simulated gimbal physics have been permanently altered in the `model.sdf` files:

* **Roll and Yaw Joints:** Welded shut. Their `<type>` has been changed to `fixed`, and their controller plugins have been deleted.
* **Pitch Joint:** `cgo3_camera_joint` remains `revolute`.
* **The Plugin:** The pitch joint is driven by `gz::sim::systems::JointPositionController`.
* **Command Topic (Gazebo side):** `/model/mdl_drone_0/gimbal/pitch_position_cmd`. This is the topic the JPC plugin actually subscribes to. Gazebo's JPC prepends the model instance namespace (`/model/mdl_drone_0/`) to the plugin's `sub_topic` value (`gimbal/pitch_position_cmd`). The `ros_gz_bridge` remaps this to the clean ROS 2 topic `/gimbal/pitch_position_cmd` so the HAL node can publish to that short name.
* **Actuation Limits:** `cmd_min` is `-2.4` rad, `cmd_max` is `0.8` rad to match physical hardware constraints.



## 4. Launch File & Topic Bridges (`vo_bringup.launch.py`)

Because we bypass PX4, the `ros_gz_bridge` is solely responsible for bridging the camera and the servo commands. The launch file establishes:

1. **Command Bridge:** Gazebo topic `/model/mdl_drone_0/gimbal/pitch_position_cmd` (`gz.msgs.Double` ← `std_msgs/Float64`), remapped to ROS 2 topic `/gimbal/pitch_position_cmd`. The HAL node publishes to the short ROS name; the bridge forwards it to the Gazebo JPC topic.
2. **Image Bridge:** `/world/default/model/mdl_drone_0/link/camera_link/sensor/camera/image` → `/camera/image_raw`.
3. **Camera Info Bridge:** `/world/default/model/mdl_drone_0/link/camera_link/sensor/camera/camera_info` → `/camera/camera_info`.
4. **Joint State Bridge (NOT YET IN LAUNCH FILE):** For the TF tree (`robot_state_publisher`) to reflect the actual camera pitch angle, `/joint_states` must be published with the live `gimbal_pitch_joint` value. Without it the joint is frozen at 0 rad in TF. This requires either a `gz_joint_state_publisher` node or a `ros_gz_bridge` entry for the Gazebo model's joint state topic.

## 5. URDF Kinematics & Physical Offsets

The `gimbal_drone_sim.urdf` has been mathematically updated to match the raw physical dimensions of the Gazebo SDFs. The agent must NOT "auto-correct" these values to standard defaults.

* **The Z-Drop (`T_pivot_pitch`):** The actual pitch axis is located $16.2\text{ cm}$ below the drone mounting plate. An intermediate link (`gimbal_pitch_axis_link`) accounts for this `xyz="-0.0412 0.0 -0.162"` offset to prevent severe triangulation errors during AprilTag PnP solving.
* **The Optical Frame Flip (`T_pitch_C`):** The camera sensor inside the SDF is physically modeled facing backward relative to the gimbal arm's local X-axis (due to a 180° yaw applied at the mount).
* To convert this to the standard ROS Optical Frame (Z-forward, X-right, Y-down), the URDF explicitly applies `rpy="-1.5708 0 1.5708"` (a positive $90^\circ$ yaw instead of the standard negative $90^\circ$ yaw).



## 6. Airframe Parameter Policy (Headless/ROS Instances)

Both PX4 instances in our simulation run headlessly — no QGroundControl, no RC transmitter, and ROS 2 must receive pure Gazebo sim time. The base `4001_gz_x500` airframe is unsuitable for this because it defaults to `NAV_DLL_ACT=2` (RTL on datalink loss). All custom airframes we create **must** override:

| Param | Value | Reason |
|-------|-------|--------|
| `UXRCE_DDS_SYNCT` | `0` | ROS 2 must use pure Gazebo sim time |
| `NAV_DLL_ACT` | `0` | No QGroundControl; disable datalink-loss failsafe |
| `NAV_RCL_ACT` | `0` | No RC transmitter; disable RC-loss failsafe |

**Our custom airframes:**
- `4022_gz_mdl_drone` — main MDL drone (x500 + pitch-only gimbal)
- `4023_gz_platform_ekf` — platform GPS telemetry instance (attaches to `platform_ekf` in baylands)

**Platform EKF launch:**
```bash
PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4023 PX4_GZ_MODEL_NAME=platform_ekf \
  ./build/px4_sitl_default/bin/px4 -i 2
```
Use `PX4_SYS_AUTOSTART=4023`, **not** `4001`, to ensure the failsafe and time-sync params are correct.

## 7. Agent Directives

1. **Assume the physics engine is correct.** Do not attempt to add software roll/yaw locks in ROS 2. The Gazebo joints are physically fixed.
2. **Maintain strictly 1-DOF math.** The `gimbal_hal_sim.py` only needs to integrate the pitch velocity into a Float64 position and push it to the bridge.
3. **Trust the TF Tree.** The URDF matches the true physical offsets of the SDF. Use standard `tf2_ros` lookups for your state estimation; do not hardcode camera offsets into your Python nodes.

---

# Platform System Model (`platform_system`)

The moving platform, AprilTags, and their joints have been extracted from `baylands.sdf` into a self-contained composite model at `Tools/simulation/gz/models/platform_system/`.

## Contents

- **`moving_platform` sub-model** — 5×5×0.1 m box, `libMovingPlatformController.so` plugin. Reads `PX4_GZ_PLATFORM_VEL` and `PX4_GZ_PLATFORM_HEADING_DEG`. `platform_link` center at z=2.0 in model frame; top surface at z=2.05.
- **`apriltag_0_link`** — AprilTag 36h11 ID 0 at (+1.0, +1.0, 2.06). Mesh: `model://April Tag 0/meshes/AprilTags0.dae`.
- **`apriltag_1_link`** — AprilTag 36h11 ID 1 at (−1.0, −1.0, 2.06). Mesh: `model://April Tag 1/meshes/AprilTags1.dae`.

Tags are inlined as **non-static dynamic links** (not `<include>` sub-models) and welded via fixed joints to `moving_platform::platform_link`. A static nested model in Gazebo Harmonic is fixed to the world frame — it would not move with the platform.

## Why `platform_ekf` Is NOT Inside `platform_system`

PX4's gz_bridge constructs sensor topic paths as `/world/<world>/model/<name>/link/...`. If the x500 were nested inside `platform_system`, topics would route to `/world/<world>/model/platform_system/model/platform_ekf/link/...` and the second PX4 instance would fail to find sensors. The x500 must remain a **world-level model** with its joint referencing `platform_system::moving_platform::platform_link`.

## `platform_test` World

`Tools/simulation/gz/worlds/platform_test.sdf` is a minimal world (flat ground + sun) with the full `platform_system` + `platform_ekf` setup. Use it instead of baylands when you want moving platform tests without the overhead of downloading online Fuel models.

```bash
PX4_GZ_WORLD=platform_test PX4_GZ_PLATFORM_VEL=0.5 PX4_GZ_PLATFORM_HEADING_DEG=120 make px4_sitl gz_mdl_drone
```

Or use the infra script with `--platform-test` to bring up the full stack in one command:

```bash
bash ../ws_px4_ros/src/MDL/scripts/start_sim_infra.sh --platform-test
```

## Using in a New World

Copy these three blocks verbatim from `baylands.sdf`:

```xml
<include>
  <uri>model://platform_system</uri>
  <name>platform_system</name>
  <pose>0 0 0 0 0 0</pose>
</include>
<include>
  <uri>model://x500</uri>
  <name>platform_ekf</name>
  <pose>2.0 2.0 2.2 0 0 0</pose>
</include>
<joint name="platform_to_ekf_fixed" type="fixed">
  <parent>platform_system::moving_platform::platform_link</parent>
  <child>platform_ekf::base_link</child>
</joint>
```

---

# Local AprilTag Models

Three Ogre2-compatible AprilTag models live under `Tools/simulation/gz/models/`:

| Directory | Tag ID | Texture source |
|-----------|--------|----------------|
| `April Tag 0/` | 0 | Original (Carl Cort) |
| `April Tag 1/` | 1 | `../gazebo_apriltag` repo (Kenji Koide) |
| `April Tag 2/` | 2 | `../gazebo_apriltag` repo (Kenji Koide) |

Each model uses a COLLADA DAE file (2×2 m flat plane, scaled 0.5× → **1 m** physical tag) with an embedded Lambert texture — Ogre2-compatible. The `gazebo_apriltag` repo itself uses Ogre1 `.material` scripts which do not render under Gazebo Harmonic's default Ogre2 renderer; our format avoids that issue.

To add more tag IDs: copy an existing `April Tag N/` directory, replace the PNG with the corresponding file from `../gazebo_apriltag/models/Apriltag36_11_000NN/materials/textures/`, and update the `<init_from>` line inside the DAE.

---

# Platform EKF Second PX4 Instance (`4023_gz_platform_ekf`)

The `platform_ekf` x500 is rigidly fixed to the moving platform at corner position **(2.0, 2.0)** in world frame — near the +X/+Y corner of the 5×5 m platform. This better represents the physical GPS antenna mount on the real platform (off-center, near an edge) while keeping the drone body safely on the platform surface (0.5 m from the 2.5 m edge).

**Spawn and URDF geometry:**
- `platform_ekf` spawns at world (2.0, 2.0, 2.2). x500_base model offset = +0.24 m → `base_link` at world z = 2.44.
- `platform_base_link` ≡ `platform_ekf::base_link` at world (2.0, 2.0, 2.44).
- In `platform_base_link` frame (encoded in `platform_sim.urdf`):
  - `landing_pad_center` (platform top surface center): (−2.0, −2.0, −0.39)
  - `platform_tag_0` (AprilTag 0 at world (1.0, 1.0, 2.06)): (−1.0, −1.0, −0.38)

**Airframe:** `4023_gz_platform_ekf` sources `4001_gz_x500` and overrides:

| Param | Value | Reason |
|-------|-------|--------|
| `UXRCE_DDS_SYNCT` | `0` | ROS 2 must use pure Gazebo sim time |
| `NAV_DLL_ACT` | `0` | No QGroundControl; disable datalink-loss failsafe |
| `NAV_RCL_ACT` | `0` | No RC transmitter; disable RC-loss failsafe |

**Launch command:**
```bash
PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4023 PX4_GZ_MODEL_NAME=platform_ekf \
  ./build/px4_sitl_default/bin/px4 -i 2
```
Use `4023`, **not** `4001` — the base airframe defaults to RTL on datalink loss.

