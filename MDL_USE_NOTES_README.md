# Resource Locations and potential uses

## Airframe specifications
The airframe specification for the basic quadcopter with gimbal is at:
`ROMFS/px4fmu_common/init.d-posix/airframes/4019_gz_x500_gimbal`

I made a new airframe for our sim at
`ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_mdl_drone`
 - almost identical to `4019_gz_x500_gimbal` but disable the RC control of the gimbal (setting `MNT_RC_IN_MODE` to `0`) so that it accepts MavLink. Regardless, wanted to leave default untouched and make this in case we want to make future changes
 - To define more airframes, simply make a new file here (or for real-world maybe elsewhere) and make sure it has a unique identifier number at the front and then also add to the CMakeLists.txt in the airframes folder (`ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`)

In general, the airframes for simulation are defined under `ROMFS/px4fmu_common/init.d-posix/airframes`
 - These are SITL-specific (real airframes live under `ROMFS/px4fmu_common/init.d/`)
 - Modifications to these files allow for parameter setting applied on startup
   - e.g. for the gimbal quadcopter (`4019_gz_x500_gimbal`), MNT_* params are set here

## Gazebo Worlds and Models
The baylands world definition (sdf) is at:
`Tools/simulation/gz/worlds/baylands.sdf`
 - this is in a submodule (gz)

Additionally, I made a simple custom world at `Tools/simulation/gz/worlds/april_test.sdf` that can be activated by setting `PX4_GZ_WORLD=april_test` and ensuring that the vo bringup (in MDL package) is run with sim and `gz_world="april_test"`. This custom april_test world is for use with gimbal drone and initial static AprilTag viewing
 - AprilTag is placed in view of the camera without the drone needing to move but also not at horizontal pitch line
 - allows for testing basic AprilTag perception and/or visual servoing without drone controls being up
 - ```PX4_GZ_WORLD=april_test make px4_sitl gz_mdl_drone```
 - ```ros2 launch MDL vo_bringup.launch.py use_sim:=true gz_world:="april_test"```

Other worlds are at
`Tools/simulation/gz/worlds/`
and models etc are at
`Tools/simulation/gz/models`
e.g.
 - moving platform is at `Tools/simulation/gz/models/moving_platform`,
 - gimbal quadcopter is at `Tools/simulation/gz/models/x500_gimbal` (just the normal quadcopter plus the gimbal)
 - regular basic quadcopter is at `Tools/simulation/gz/models/x500`
 - to include things from this directory, can simply specify using a uri tag and the "model://" prefix which routes to the `Tools/simulation/gz/models` directory
   - e.g. the x500 drone is referenced via `<uri>model://x500</uri>` in our modified `baylands.sdf` file

### Adding custom worlds/models
It is very easy to add a model, just add a full directory with the name you want to reference the model by to the `Tools/simulation/gz/models` directory then follow notes above
 - e.g. look at `Tools/simulation/gz/models/April Tag 0` and `Tools/simulation/gz/models/mdl_drone`

Adding a custom world is similarly easy but has a few key requirements.
 - Only need to add a new .sdf to the `Tools/simulation/gz/worlds/`
 - world name in the sdf must match its name in the file (without `.sdf` suffix)
 - use per usual by setting: `PX4_GZ_WORLD=<MY_WORLD>` env var when running `PX4 make sitl ...`
 - e.g. look at `Tools/simulation/gz/worlds/april_test.sdf`, which is our custom AprilTag and perception without flight control testing world

---

# How to Add a Custom Drone SITL

Adding a new drone for Gazebo SITL requires touching **three areas**: the Gazebo model, the PX4 airframe, and the build system registration. Here is the complete checklist.

## 1. Define the Gazebo Model

Create a directory `Tools/simulation/gz/models/<model_name>/` containing:

- **`model.config`** — Gazebo model metadata. Must reference the correct SDF filename:
  ```xml
  <?xml version="1.0"?>
  <model>
    <name>my_model</name>
    <version>1.0</version>
    <sdf version="1.9">model.sdf</sdf>   <!-- MUST be named model.sdf -->
    ...
  </model>
  ```

- **`model.sdf`** — The Gazebo SDF description of the vehicle. **The file MUST be named `model.sdf`** — `px4-rc.gzsim` hard-codes the spawn path as `${PX4_GZ_MODELS}/<model_name>/model.sdf`.
- look at an existing one for reference


  Common pattern (merging an existing base + a payload):
  ```xml
  <sdf version='1.9'>
    <model name='my_model'>
      <include merge='true'>
        <uri>model://x500</uri>           <!-- base quad: motors/props/physics -->
      </include>
      <include merge='true'>
        <uri>model://my_payload</uri>
        <pose>0 0 0.26 0 0 3.14</pose>
      </include>
      <joint name="PayloadAttachJoint" type="fixed">
        <parent>base_link</parent>        <!-- from x500 -->
        <child>payload_mount_link</child> <!-- from my_payload -->
      </joint>
    </model>
  </sdf>
  ```
  The `x500` model only has motors. Sensors (IMU, GPS, barometer, magnetometer) come from `x500_base`, which `x500` already merges in. There is no need to include `x500_base` directly.

- **`meshes/`** (optional) — STL/DAE visual meshes if the model has custom visuals.


## 2. Define the PX4 Airframe

Create `ROMFS/px4fmu_common/init.d-posix/airframes/<NNNN>_gz_<model_name>` where `NNNN` is a unique 4-digit number not already used in that directory.

Key fields:
```sh
#!/bin/sh
# @name My Custom Drone
# @type Quadrotor

# IMPORTANT: fallback model name must match the model directory name exactly
PX4_SIM_MODEL=${PX4_SIM_MODEL:=<model_name>}

# Source a base airframe for all the standard quadrotor params
. ${R}etc/init.d-posix/airframes/4001_gz_x500

# Add any custom param defaults here
param set-default SOME_PARAM value
```

- The filename suffix after `_gz_` must exactly match the model directory name (e.g. `4022_gz_mdl_drone` → model dir `mdl_drone`)
- The cmake target name is derived by stripping everything up to `_gz_`: `4022_gz_mdl_drone` → `gz_mdl_drone`

## 3. Register the Airframe in the Build System

Add the airframe filename to `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt` inside the `px4_add_romfs_files(...)` block (keep the list sorted by number).

Once registered, cmake will **automatically generate** the following make targets:
- `gz_<model_name>` — launches with the default world
- `gz_<model_name>_<world_name>` — one target per world SDF in `Tools/simulation/gz/worlds/`

## 4. Rebuild

After adding new files or editing the CMakeLists.txt, a full rebuild is required:
```bash
make px4_sitl gz_<model_name>
```
This reconfigures cmake (picks up the new airframe), builds the PX4 binary, and copies init scripts to `build/px4_sitl_default/etc/init.d-posix/`.

## 5. Gimbal / Payload Plugins

If your payload has Gazebo plugins (e.g. a `JointPositionController` for a servo), the topics it subscribes to are **independent of PX4's gz_bridge**. gz_bridge publishes gimbal commands to `/<world>/<model>/command/gimbal_*` (for PX4 MAVLink-controlled gimbals). If your payload is controlled directly by ROS or another node, just make the plugin subscribe to whatever topic your ROS node will publish to — there is no conflict.

The MNT_* parameters in the airframe control PX4's gimbal module (which GZBridge reads to publish gimbal commands). They can be left as-is or omitted if the gimbal is externally controlled.

---

# MDL Drone SITL — Status Tracker

**Goal**: `make px4_sitl gz_mdl_drone` launches the MDL drone (x500 frame + pitch-only gimbal) in the default Gazebo world.

## Files Created/Modified

| File | Status | Notes |
|------|--------|-------|
| `Tools/simulation/gz/models/mdl_drone/model.sdf` | ✅ Done | Main model SDF — merges x500 + mdl_pitch_gimbal, adds GimbalAttachJoint |
| `Tools/simulation/gz/models/mdl_drone/model.config` | ✅ Done | Metadata referencing model.sdf |
| `Tools/simulation/gz/models/mdl_pitch_gimbal/model.sdf` | ✅ Done | Pitch-only gimbal SDF; JPC plugin subscribes to `/model/mdl_drone_0/gimbal/pitch_position_cmd` |
| `Tools/simulation/gz/models/mdl_pitch_gimbal/model.config` | ✅ Done | |
| `Tools/simulation/gz/models/mdl_pitch_gimbal/meshes/` | ✅ Done | Mesh files (also available from `model://gimbal/meshes/`) |
| `ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_mdl_drone` | ✅ Done | Airframe: disables QGC/RC failsafes and DDS time sync; sources 4001_gz_x500 |
| `ROMFS/px4fmu_common/init.d-posix/airframes/4023_gz_platform_ekf` | ✅ Done | Airframe: platform GPS telemetry instance; disables QGC/RC failsafes and DDS time sync; sources 4001_gz_x500 |
| `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt` | ✅ Done | `4022_gz_mdl_drone` and `4023_gz_platform_ekf` registered |

## Bugs Fixed (were blocking `make px4_sitl gz_mdl_drone`)

1. **`mdl_drone_model.sdf` → `model.sdf`**: `px4-rc.gzsim` spawns models via `sdf_filename: "${PX4_GZ_MODELS}/<name>/model.sdf"` — the file must be named `model.sdf`
2. **`mdl_pitch_gimbal/model.sdf` line 108**: Stray `=` after `</joint>` caused an SDF parse error; removed
3. **`4022_gz_mdl_drone`**: Wrong fallback `PX4_SIM_MODEL:=x500_gimbal` → corrected to `PX4_SIM_MODEL:=mdl_drone`
4. **`mdl_pitch_gimbal/model.sdf`**: Duplicate `<format>R8G8B8</format>` in camera sensor; removed
5. **`mdl_pitch_gimbal/model.sdf` JPC `sub_topic`**: Had a leading `/` (`/gimbal/pitch_position_cmd`). Gazebo strips the leading `/` and concatenates it onto the model namespace without a separator, producing the malformed topic `/model/mdl_drone_0gimbal/pitch_position_cmd` instead of `/model/mdl_drone_0/gimbal/pitch_position_cmd`. Fixed by removing the leading `/`. The model verification loop in `px4-rc.gzsim` checks for `^/model/<model_instance>/` — without this fix, the JPC topic didn't match and the check always timed out.

## ✅ `make px4_sitl gz_mdl_drone` — WORKING

Verified working as of 2026-03-11. Key startup output:
```
INFO  [init] found model autostart file as SYS_AUTOSTART=4022
INFO  [init] Model mdl_drone_0 confirmed in world
INFO  [gz_bridge] world: default, model: mdl_drone_0
INFO  [px4] Startup script returned successfully
```

## What Still Needs to Be Done

- [ ] **ROS gimbal control**: Publish `gz.msgs.Double` to `/model/mdl_drone_0/gimbal/pitch_position_cmd` to control camera pitch from ROS. Set up a gz↔ROS bridge for this topic.
- [ ] **Camera image topic**: Camera publishes to `/world/default/model/mdl_drone_0/link/camera_link/sensor/camera/image` — set up a gz↔ROS bridge to expose it in ROS.
- [ ] **Camera IMU topic**: GZBridge subscribes to the camera IMU (`/world/default/model/mdl_drone_0/link/camera_link/sensor/camera_imu/imu`) for gimbal stabilization feedback. Since the gimbal is externally controlled, this feedback is unused by PX4 — benign but worth noting.

## Design Notes

- The gimbal (`mdl_pitch_gimbal`) is **not controlled by PX4** — its `JointPositionController` listens to `/model/mdl_drone_0/gimbal/pitch_position_cmd`, not the PX4 gz_bridge gimbal topics. This matches the real hardware where the gimbal servo is driven directly by the SBC.
- MNT_* parameters are omitted from the airframe (the user removed them) since gimbal is externally controlled.
- Meshes: `mdl_pitch_gimbal/model.sdf` references `model://gimbal/meshes/...` for visuals (reuses the existing CGO3 gimbal geometry). The `mdl_pitch_gimbal/meshes/` directory contains local copies of those same meshes.
- The JPC `sub_topic` field uses model-relative naming (no leading `/`). Gazebo prepends `/model/<instance_name>/` automatically, so `gimbal/pitch_position_cmd` → `/model/mdl_drone_0/gimbal/pitch_position_cmd`.


---

# Platform EKF Second PX4 Instance (`4023_gz_platform_ekf`)

The `platform_ekf` model in `baylands.sdf` is an x500 rigidly fixed to the moving platform. A second PX4 instance attaches to it to publish GPS telemetry for the platform. Previously this used `PX4_SYS_AUTOSTART=4001` (the base `4001_gz_x500` airframe) directly.

**Problem with using `4001_gz_x500` directly:**
- `NAV_DLL_ACT=2` (RTL on datalink loss) — this instance has no QGroundControl and would immediately trigger a failsafe
- `UXRCE_DDS_SYNCT` defaults to enabled — must be disabled so ROS 2 gets pure Gazebo sim time

**Why not edit `4001_gz_x500`:**
`4001_gz_x500` is sourced by every x500 variant (`4019_gz_x500_gimbal`, `4022_gz_mdl_drone`, etc.). Editing it would affect all x500 SITL targets.

**Solution:** `4023_gz_platform_ekf` sources `4001_gz_x500` and overrides:

| Param | Value | Reason |
|-------|-------|--------|
| `UXRCE_DDS_SYNCT` | `0` | ROS 2 must use pure Gazebo sim time, not wall-clock sync |
| `NAV_DLL_ACT` | `0` | No QGroundControl connected; disable datalink-loss failsafe |
| `NAV_RCL_ACT` | `0` | No RC transmitter; disable RC-loss failsafe |

**Launch command (second terminal, after main drone is up):**
```bash
PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4023 PX4_GZ_MODEL_NAME=platform_ekf \
  ./build/px4_sitl_default/bin/px4 -i 2
```

---

# MISC For Future
