# PX4 Autopilot — Project Guide for Claude

This is a development fork of [PX4 Autopilot](https://px4.io/), used primarily for **Gazebo (gz-harmonic) simulation**. The main use case is a quadrotor (x500) with a gimbal camera flying over a moving platform, where a second PX4 instance provides GPS telemetry for the platform.

## Key Simulation Entry Points

```bash
# Gimbal drone, default world
make px4_sitl gz_x500_gimbal

# Gimbal drone, baylands world (pretty scenery)
make px4_sitl gz_x500_gimbal_baylands

# Lightweight moving platform test (no baylands overhead) + platform EKF drone
PX4_GZ_WORLD=platform_test PX4_GZ_PLATFORM_VEL=0.5 PX4_GZ_PLATFORM_HEADING_DEG=120 make px4_sitl gz_mdl_drone

# Baylands world + moving platform + platform EKF drone (no gimbal)
PX4_GZ_WORLD=baylands PX4_GZ_PLATFORM_VEL=0.5 PX4_GZ_PLATFORM_HEADING_DEG=120 make px4_sitl gz_x500

# Baylands world + moving platform + gimbal drone (full setup)
PX4_GZ_WORLD=baylands PX4_GZ_PLATFORM_VEL=0.5 PX4_GZ_PLATFORM_HEADING_DEG=120 make px4_sitl gz_x500_gimbal

# Second PX4 instance — attaches to the platform_ekf model for GPS telemetry
# Use 4023 (not 4001) — disables QGC/RC failsafes and DDS time sync for headless ROS use
PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4023 PX4_GZ_MODEL_NAME=platform_ekf ./build/px4_sitl_default/bin/px4 -i 2
```

**Simulation env vars:**
- `PX4_GZ_WORLD` — world SDF to load (default: `default`)
- `PX4_GZ_PLATFORM_VEL` — moving platform speed in m/s
- `PX4_GZ_PLATFORM_HEADING_DEG` — platform heading in degrees
- `PX4_GZ_MODEL_POSE` — spawn pose `x,y,z` for the main drone
- `PX4_GZ_MODEL_NAME` — attach to an existing Gazebo model instead of spawning
- `PX4_GZ_SIM_RENDER_ENGINE` — override Gazebo render engine (e.g., `ogre`)
- `HEADLESS=1` — skip launching Gazebo GUI

## Repo Structure

```
PX4-Autopilot/
├── ROMFS/px4fmu_common/
│   └── init.d-posix/
│       ├── rcS                    # Main SITL startup script
│       ├── px4-rc.simulator       # Dispatches to gz/jmavsim/etc.
│       ├── px4-rc.gzsim           # Gazebo startup, model spawn, gz_bridge start
│       └── airframes/             # One file per vehicle config (numbered)
│           ├── 4001_gz_x500       # Base x500 quad — all x500 variants source this
│           ├── 4019_gz_x500_gimbal # x500 + gimbal; sets MNT_* params
│           └── ...
│
├── Tools/simulation/gz/
│   ├── worlds/                    # Gazebo world SDF files
│   │   ├── default.sdf            # Flat ground plane
│   │   ├── baylands.sdf           # Real-world scenery + moving_platform + platform_ekf
│   │   └── ...
│   └── models/                    # Gazebo model SDF files
│       ├── x500/                  # Base quad (motor plugins only)
│       ├── x500_base/             # Geometry + sensors (IMU, GPS, baro, mag)
│       ├── x500_gimbal/           # x500 + gimbal (merge includes)
│       ├── gimbal/                # 3-axis gimbal with camera + JPC plugins
│       ├── moving_platform/       # Moving platform with libMovingPlatformController.so
│       └── ...
│
├── src/modules/simulation/
│   ├── gz_bridge/                 # C++ PX4↔Gazebo bridge (GZBridge, GZGimbal, etc.)
│   │   ├── GZBridge.cpp/hpp       # Main bridge: subscribes to all sensor topics
│   │   ├── GZGimbal.cpp/hpp       # Gimbal control: advertises command topics, reads MNT_* params
│   │   ├── GZMixingInterface*.cpp # ESC/servo/wheel output mixing
│   │   └── gz_env.sh.in           # Template for environment setup script
│   └── gz_plugins/                # Custom Gazebo system plugins (compiled to .so)
│       ├── moving_platform_controller/  # MovingPlatformController: reads PX4_GZ_PLATFORM_VEL/HEADING
│       └── optical_flow/
│
└── build/px4_sitl_default/
    ├── rootfs/                    # SITL working directory (px4 runs here)
    │   ├── gz_env.sh              # Sets GZ_SIM_RESOURCE_PATH, PX4_GZ_MODELS, etc.
    │   └── etc/init.d-posix/      # Built copies of ROMFS init scripts
    └── src/modules/simulation/
        └── gz_plugins/            # Compiled plugin .so files (libMovingPlatformController.so, etc.)
```

## How Simulation Startup Works

1. `make px4_sitl gz_<model>` runs the cmake target which invokes `px4` binary with `PX4_SIM_MODEL=gz_<model>`
2. `rcS` finds the matching airframe file by number (e.g., `4019_gz_x500_gimbal`) and sources it
3. `px4-rc.gzsim` is sourced via `px4-rc.simulator`:
   - Sources `gz_env.sh` to set resource paths
   - Starts `gz sim` server with the world SDF (background)
   - Waits for world to be ready (`gz service -i /world/.../scene/info`)
   - Spawns the drone model via `gz service /world/.../create`
   - Verifies the model appeared (polls `gz topic -l`)
   - Starts `gz_bridge` which subscribes to all sensor topics and bridges them to uORB

## Model Naming Convention

The cmake targets for Gazebo follow: `gz_<model>` or `gz_<model>_<world>`.

At runtime, the model instance name is: `<model>_<instance>` (e.g., `x500_gimbal_0` for instance 0).

The `MovingPlatformController` reads `PX4_SIM_MODEL` to determine the vehicle name it should wait for before starting to move the platform.

## baylands.sdf — Key Structure

The custom baylands world includes:
- Online Fuel models: baylands park, Coast Water
- `platform_system` — composite model (see `Tools/simulation/gz/models/platform_system/`) containing:
  - `moving_platform` sub-model — flat 5×5m platform with `MovingPlatformController` plugin
  - `apriltag_0_link` / `apriltag_1_link` — AprilTag 36h11 ID 0 and ID 1 markers, welded flush to the platform surface (1 cm clearance for z-fighting avoidance). Use DAE mesh format (Ogre2-compatible). Sourced from local `model://April Tag 0` and `model://April Tag 1`.
- `platform_ekf` — an `x500` model kept as a **world-level** model (not nested inside `platform_system`) so PX4's gz_bridge can find its sensor topics at the standard `/world/<world>/model/platform_ekf/...` path. Welded to `platform_system::moving_platform::platform_link` via a world joint. Spawned at (2.0, 2.0) — near the +X/+Y corner of the platform to better represent the real-world GPS antenna mount position.

The main flying drone is **dynamically spawned** (not in the SDF), controlled by the first PX4 instance. The `platform_ekf` drone is controlled by a second PX4 instance (`-i 2`) that attaches via `PX4_GZ_MODEL_NAME=platform_ekf`.

## Local AprilTag Models

Three AprilTag 36h11 models live under `Tools/simulation/gz/models/`:

| Directory | Tag ID | Mesh |
|-----------|--------|------|
| `April Tag 0/` | ID 0 | `meshes/AprilTags0.dae` + `tag36h11-0.png` |
| `April Tag 1/` | ID 1 | `meshes/AprilTags1.dae` + `tag36h11-1.png` |
| `April Tag 2/` | ID 2 | `meshes/AprilTags2.dae` + `tag36h11-2.png` |

All use a 2×2 m flat plane mesh scaled 0.5× → **1 m** physical tag size. Textures for IDs 1 and 2 were sourced from the [gazebo_apriltag](https://github.com/koide3/gazebo_apriltag) repo (Kenji Koide). The DAE/mesh format is **Ogre2-compatible** (Gazebo Harmonic default renderer), unlike the classic Ogre1 `.material` script format used by that repo.

To add more tag IDs: copy the `April Tag N` directory, swap the PNG from `../gazebo_apriltag/models/Apriltag36_11_000NN/materials/textures/`, and update the `<init_from>` line in the DAE.

## Sim Infrastructure Script

`../ws_px4_ros/src/MDL/scripts/start_sim_infra.sh` launches the full sim stack (XRCE agent, PX4 SITL, platform EKF) each in its own tmux window.

```bash
# baylands (default)
bash scripts/start_sim_infra.sh

# lightweight platform_test world (no Fuel downloads)
bash scripts/start_sim_infra.sh --platform-test

# headless + platform_test
bash scripts/start_sim_infra.sh platform_test 0.5 120 1

# with QGroundControl
bash scripts/start_sim_infra.sh --qgc
```

Flags: `--platform-test` overrides world to `platform_test`; `--qgc` launches QGroundControl; positional args are `[GZ_WORLD] [PLATFORM_VEL] [PLATFORM_HEADING_DEG] [HEADLESS]`.

## Known Issues / Notes

- **Spawn timeout**: Complex models (x500_gimbal) in busy worlds (baylands) previously timed out at 5000ms. Fixed by increasing to 15000ms + adding a model verification loop in `px4-rc.gzsim`.
- **libEGL warnings**: These come from the Gazebo server initializing rendering for camera sensors. They are warnings only and do not indicate a spawn failure.
- **Build copies**: `build/px4_sitl_default/etc/init.d-posix/` contains copies of the ROMFS init scripts. After editing source files in `ROMFS/`, either rebuild or manually copy to keep in sync.
- **MNT_* params**: Registered at compile time (in `src/modules/gimbal/gimbal_params.c`), always available regardless of module startup order.
