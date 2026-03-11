# Context: Custom `gz_mdl_drone` SITL & 1-DOF Gimbal Architecture

**Document Purpose:** This document provides the environmental and architectural truth for the custom `gz_mdl_drone` simulation target. The agent MUST reference this to understand how the simulation physics, the ROS 2 bridges, and the URDF have been modified.

## 1. The Core Architectural Shift: Bypassing PX4

* **Old Architecture:** The ROS 2 HAL node published quaternions via `px4_msgs/GimbalManagerSetAttitude` to the PX4 firmware, which then mixed the commands and sent them to Gazebo.
* **New Architecture:** We are explicitly **bypassing PX4** for gimbal control to perfectly mirror our physical SBC-to-Servo hardware setup. The ROS 2 HAL node now publishes a standard `std_msgs/Float64` directly to a Gazebo `JointPositionController` plugin via `ros_gz_bridge`.
* **Rule:** Do not import, use, or attempt to configure any `px4_msgs` related to the gimbal or mount manager. PX4 handles flight; Gazebo directly handles the camera servo.

## 2. The Custom SITL Target (`gz_mdl_drone`)

We have created a custom simulation target to sandbox our physical changes without breaking the default PX4 models.

* **Launch Command:** `make px4_sitl gz_mdl_drone`
* **Airframe ID:** `4022_gz_mdl_drone`. This file configures the PX4 firmware to ignore the gimbal and points the simulator to spawn our custom SDF model.
* **Model Name:** Gazebo spawns this instance under the model name `mdl_drone` (typically resulting in the ROS bridge namespace `mdl_drone_0`).

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
2. **Image Bridge:** `.../sensor/camera/image` → `/camera/image_raw`.
3. **Camera Info Bridge:** `.../sensor/camera/camera_info` → `/camera/camera_info`.

## 5. URDF Kinematics & Physical Offsets

The `gimbal_drone_sim.urdf` has been mathematically updated to match the raw physical dimensions of the Gazebo SDFs. The agent must NOT "auto-correct" these values to standard defaults.

* **The Z-Drop (`T_pivot_pitch`):** The actual pitch axis is located $16.2\text{ cm}$ below the drone mounting plate. An intermediate link (`gimbal_pitch_axis_link`) accounts for this `xyz="-0.0412 0.0 -0.162"` offset to prevent severe triangulation errors during AprilTag PnP solving.
* **The Optical Frame Flip (`T_pitch_C`):** The camera sensor inside the SDF is physically modeled facing backward relative to the gimbal arm's local X-axis (due to a 180° yaw applied at the mount).
* To convert this to the standard ROS Optical Frame (Z-forward, X-right, Y-down), the URDF explicitly applies `rpy="-1.5708 0 1.5708"` (a positive $90^\circ$ yaw instead of the standard negative $90^\circ$ yaw).



## 6. Agent Directives

1. **Assume the physics engine is correct.** Do not attempt to add software roll/yaw locks in ROS 2. The Gazebo joints are physically fixed.
2. **Maintain strictly 1-DOF math.** The `gimbal_hal_sim.py` only needs to integrate the pitch velocity into a Float64 position and push it to the bridge.
3. **Trust the TF Tree.** The URDF matches the true physical offsets of the SDF. Use standard `tf2_ros` lookups for your state estimation; do not hardcode camera offsets into your Python nodes.

