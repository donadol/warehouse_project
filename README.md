# Warehouse Robot Navigation Project

This project implements a complete autonomous navigation system for a warehouse robot using ROS2 Nav2 stack, including SLAM mapping, AMCL localization, and path planning capabilities.

## Project Structure

```plaintext
warehouse_project/
├── cartographer_slam/      # SLAM mapping with Cartographer
├── localization_server/    # AMCL localization
├── map_server/             # Map storage and keepout filters
├── path_planner_server/    # Nav2 path planning and navigation
├── attach_shelf/           # Shelf attachment service (from checkpoint 9)
└── nav2_apps/              # Simple Commander API applications
```

## Prerequisites

- ROS2 Humble
- Navigation2 (Nav2) stack
- Cartographer ROS
- Robot description package from `~/sim_ws` workspace

## Setup

### 1. Build the packages

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 2. Source the simulation workspace (required for robot meshes)

```bash
source ~/sim_ws/install/setup.bash
```

## Usage

### Task 1: Mapping with Cartographer SLAM

Create a map of the warehouse environment using Cartographer SLAM:

**Simulation:**

```bash
# Source both workspaces
source ~/sim_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash

# Launch Cartographer SLAM
ros2 launch cartographer_slam cartographer.launch.py use_sim_time:=True
```

**Real Robot:**

```bash
source ~/sim_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch cartographer_slam cartographer.launch.py use_sim_time:=False
```

**Features:**

- Real-time SLAM mapping
- RViz visualization with top-down view
- LaserScan and TF display
- Saves map as `warehouse_map_sim.yaml` and `warehouse_map_sim.pgm`
- **Automatic config selection**: Dynamically selects `cartographer_sim.lua` or `cartographer_real.lua` based on `use_sim_time` parameter

**Controls:**

- Drive the robot around to build the map
- Save the map when complete using map_saver

### Task 2: Localization with AMCL

Localize the robot on a pre-built map using Adaptive Monte Carlo Localization:

**Simulation:**

```bash
source ~/sim_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch localization_server localization.launch.py map_file:=warehouse_map_sim.yaml
```

**Real Robot:**

```bash
source ~/sim_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch localization_server localization.launch.py map_file:=warehouse_map_real.yaml
```

**Nodes Launched:**

- `map_server` - Serves the map for localization and navigation
- `amcl` - Adaptive Monte Carlo Localization
- `filter_mask_server` - Serves costmap filter masks (keepout zones, etc.)
- `costmap_filter_info_server` - Provides filter metadata
- `lifecycle_manager_localization` - Manages lifecycle of all above nodes
- `rviz2` - Visualization

**Features:**

- AMCL particle filter localization
- Costmap filter infrastructure (keepout zones, speed limits)
- RViz visualization with:
  - Map display
  - Particle cloud visualization
  - LaserScan overlay
  - Robot pose estimation
- Set initial pose using "2D Pose Estimate" tool in RViz
- **Intelligent config selection**: Automatically detects map file and sets `use_sim_time` accordingly
  - Maps containing `'real'` in filename → uses `amcl_config_real.yaml` and `filters_real.yaml` with `use_sim_time: False`
  - Other maps → uses `amcl_config_sim.yaml` and `filters_sim.yaml` with `use_sim_time: True`
  - Detection logic: `PythonExpression(["'real' not in '", map_file_arg, "'"])` for flexible filename matching

### Task 3: Path Planning and Navigation

Navigate autonomously to goal locations using Nav2 stack:

**Simulation:**

```bash
source ~/sim_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch path_planner_server pathplanner.launch.py use_sim_time:=True
```

**Real Robot:**

```bash
source ~/sim_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch path_planner_server pathplanner.launch.py use_sim_time:=False
```

**Nodes Launched:**

- `planner_server` - Nav2 global path planner (NavFn)
- `controller_server` - Nav2 local trajectory controller (DWB)
- `bt_navigator` - Behavior tree navigation executor
- `recoveries_server` - Recovery behaviors (spin, backup, wait)
- `lifecycle_manager_pathplanner` - Manages lifecycle of above Nav2 nodes
- `rviz2` - Visualization
- `approach_service_server` - Shelf attachment service with adaptive leg detection

**Features:**

- Full Nav2 navigation stack
- RViz displays:
  - Global and local costmaps
  - Global and local paths
  - Robot footprint visualization
  - Map and robot model
- Set navigation goals using "2D Nav Goal" tool in RViz
- Autonomous navigation with replanning and recovery behaviors
- **Dynamic configuration**: Automatically selects sim or real configs based on `use_sim_time` parameter
- **Conditional remapping**: `/cmd_vel` remapping only applies in simulation
- **Integrated shelf attachment service**:
  - Adaptive intensity thresholding (75% sim, 80% real of max intensity)
  - Multi-frame cart reference system (align, entry, center frames)
  - Proper orientation calculation using perpendicular vectors

## Configuration Files

### Cartographer SLAM

- `cartographer_slam/config/cartographer_sim.lua` - SLAM parameters for simulation (odom frame)
- `cartographer_slam/config/cartographer_real.lua` - SLAM parameters for real robot (robot_odom frame)
- `cartographer_slam/rviz/mapping.rviz` - RViz configuration

### Localization

- `localization_server/config/amcl_config_sim.yaml` - AMCL parameters for simulation
- `localization_server/config/amcl_config_real.yaml` - AMCL parameters for real robot
- `localization_server/config/warehouse_map_sim.yaml` - Simulation map
- `localization_server/config/warehouse_map_real.yaml` - Real robot map
- `localization_server/config/filters_sim.yaml` - Costmap filters for simulation
- `localization_server/config/filters_real.yaml` - Costmap filters for real robot
- `localization_server/rviz/localization.rviz` - RViz configuration

### Path Planning

**Simulation Configs:**

- `path_planner_server/config/planner_sim.yaml` - Global planner and costmap
- `path_planner_server/config/controller_sim.yaml` - Local planner and costmap (10 Hz, odom frame)
- `path_planner_server/config/bt_navigator_sim.yaml` - Behavior tree navigator (10ms loop)
- `path_planner_server/config/recoveries_sim.yaml` - Recovery behaviors

**Real Robot Configs:**

- `path_planner_server/config/planner_real.yaml` - Global planner and costmap
- `path_planner_server/config/controller_real.yaml` - Local planner and costmap (5 Hz, robot_odom frame, 3m x 2m)
- `path_planner_server/config/bt_navigator_real.yaml` - Behavior tree navigator (20ms loop)
- `path_planner_server/config/recoveries_real.yaml` - Recovery behaviors

**Shared:**

- `path_planner_server/config/navigate_w_replanning_and_recovery.xml` - Behavior tree definition
- `path_planner_server/rviz/pathplanning.rviz` - RViz configuration

### Shelf Attachment Service

- `attach_shelf/config/approach_params_sim.yaml` - Simulation configuration
  - Topics: `/scan`, `/diffbot_base_controller/odom`, `/diffbot_base_controller/cmd_vel_unstamped`
  - Frames: `map`, `odom`, `robot_base_link`
  - Adaptive threshold: 75% of max intensity
  - Distances: align_ahead=0.30m, cart_center=0.48m
- `attach_shelf/config/approach_params_real.yaml` - Real robot configuration
  - Topics: `/scan`, `/odom`, `/cmd_vel`
  - Frames: `map`, `robot_odom`, `base_link`
  - Adaptive threshold: 80% of max intensity
  - Distances: align_ahead=0.35m, cart_center=0.34m

## Key Parameters

### Robot Configuration

- **Base Frame:** `robot_base_footprint`
- **Robot Radius:** 0.25m (circular footprint)
- **Inflation Radius:** 0.35m
- **Max Linear Velocity:** 0.26 m/s
- **Max Angular Velocity:** 1.0 rad/s

### Simulation vs Real Robot Differences

| Parameter | Simulation | Real Robot |
|-----------|-----------|------------|
| **use_sim_time** | True | False |
| **Controller Frequency** | 10 Hz | **5 Hz** |
| **Local Costmap Frame** | `odom` | **`robot_odom`** |
| **Local Costmap Size** | 1m x 1m | **3m x 2m** |
| **Local Costmap Update Freq** | 5 Hz | 5 Hz |
| **BT Loop Duration** | 10ms | **20ms** |
| **cmd_vel Remapping** | `/diffbot_base_controller/cmd_vel_unstamped` | `/cmd_vel` (no remapping) |
| **Approach Service Topics** | See `approach_params_sim.yaml` | See `approach_params_real.yaml` |

### Navigation Stack Details

- **Global Planner:** NavFn (Dijkstra's algorithm)
- **Local Planner:** DWB (Dynamic Window Approach)
- **Global Costmap:** Static layer + obstacle layer + inflation
- **Local Costmap:** Voxel layer + inflation (rolling window)
- **Goal Tolerance:** XY: 0.25m, Yaw: 0.15 rad
- **Planner Frequency:** 10 Hz
- **Behavior Tree:** Replanning at 1 Hz with 6 recovery retries
- **Recovery Sequence:** Clear costmaps → Spin → Wait → Backup

## RViz Tools

### Mapping (Cartographer)

- View the map being built in real-time
- TF tree visualization
- LaserScan overlay

### Localization (AMCL)

- **2D Pose Estimate:** Set initial robot pose (required on startup)
- View particle cloud convergence
- Monitor localization accuracy

### Navigation (Nav2)

- **2D Pose Estimate:** Set initial pose (if needed)
- **2D Nav Goal:** Send navigation goals
- View global/local paths
- Monitor costmaps
- Observe recovery behaviors

## Troubleshooting

### Robot mesh errors

Make sure to source the sim_ws before launching:

```bash
source ~/sim_ws/install/setup.bash
```

### Navigation not starting

1. Ensure localization is running first
2. Set initial pose using "2D Pose Estimate" tool
3. Check that map is loaded correctly
4. Verify TF tree is complete (map → odom → robot_base_footprint)

### Robot not moving

1. Check topic remapping: `/diffbot_base_controller/cmd_vel_unstamped`
2. Verify lifecycle nodes are active
3. Check for costmap obstacles blocking path
4. Review recovery behavior execution

### Costmap issues

1. Verify laser scan topic: `/scan`
2. Check robot_radius parameter matches actual robot size
3. Ensure use_sim_time is set correctly for your environment

## Architecture

### TF Tree

``` Plain text
map
 └─ odom
     └─ robot_base_footprint
         └─ robot_base_link
             ├─ robot_front_laser_base_link
             ├─ robot_omni_backwheel_link
             ├─ robot_omni_front_leftwheel_link
             └─ robot_omni_front_rightwheel_link
```

## Task 4: Autonomous Warehouse Robot (Simple Commander API)

Complete autonomous workflow for shelf pickup and delivery using Nav2 Simple Commander API.

### Waypoints

**Simulation:**

```python
init_position:      (-0.199, 0.200)      # Starting/home position (0° facing forward)
loading_position:   (5.613, -0.448)      # Shelf pickup location (-90° facing left)
shipping_position:  (2.618, 1.395)       # Shelf delivery location (+90° facing right)
```

**Real Robot:**

```python
init_position:      (3.060, 0.038)       # Starting/home position
loading_position:   (1.173, -4.416)      # Shelf pickup location
shipping_position:  (3.872, -2.204)      # Shelf delivery location
```

### Robot Footprint

The robot uses octagon-shaped footprints that dynamically change when carrying a shelf:

- **Normal footprint**: 0.30m radius octagon (without shelf)
- **Shelf footprint**: 0.55m radius octagon (with shelf attached)

Footprint updates are done via Nav2 parameter services and include:

- Robot footprint polygon
- Voxel layer obstacle_min_range (0.0 normal, 0.55 with shelf)
- Voxel layer raytrace_min_range (0.0 normal, 0.55 with shelf)

### Keepout Filter (Cones Avoidance)

**Files Created:**

- `map_server/config/warehouse_map_keepout_sim.pgm` - Binary mask defining forbidden zones
- `map_server/config/warehouse_map_keepout_sim.yaml` - Keepout mask configuration
- `path_planner_server/config/filters.yaml` - Costmap filter configuration

**Files Modified:**

- `path_planner_server/config/planner_sim.yaml` - Added keepout filter to global costmap
- `path_planner_server/config/controller_sim.yaml` - Added keepout filter to local costmap

The keepout filter marks the cones area as lethal obstacles, forcing the planner to route around it automatically.

### Running the Complete Warehouse Task

#### Simulation Workflow

##### Terminal 1: Start Localization (Sim)

```bash
source ~/sim_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch localization_server localization.launch.py map_file:=warehouse_map_keepout_sim.yaml
```

##### Terminal 2: Start Navigation (Sim)

```bash
source ~/sim_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch path_planner_server pathplanner.launch.py use_sim_time:=True
```

**Note:**

- The `localization.launch.py` automatically launches `filter_mask_server` and `costmap_filter_info_server` with the correct configuration.
- The `pathplanner.launch.py` automatically launches the `approach_service_server_node` with the correct configuration based on `use_sim_time`.
- No need to launch filter servers or attach_shelf separately.

##### Terminal 3: Launch Warehouse Task (Sim)

```bash
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/warehouse_project/nav2_apps/scripts/move_shelf_to_ship.py
```

#### Real Robot Workflow

##### Terminal 1: Start Localization (Real)

```bash
source ~/sim_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch localization_server localization.launch.py map_file:=warehouse_map_keepout_real.yaml
```

##### Terminal 2: Start Navigation (Real)

```bash
source ~/sim_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch path_planner_server pathplanner.launch.py use_sim_time:=False
```

**Note:** The `pathplanner.launch.py` automatically launches the `approach_service_server_node` with real robot topics configured in `approach_params_real.yaml`.

##### Terminal 3: Launch Warehouse Task (Real)

```bash
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/warehouse_project/nav2_apps/scripts/move_shelf_to_ship_real.py
```

### Key Differences Between Simulation and Real Robot

- **Scripts**: `move_shelf_to_ship.py` (sim) vs `move_shelf_to_ship_real.py` (real)
- **Topics**: Automatically configured via `approach_params_sim.yaml` vs `approach_params_real.yaml`
- **Positions**: Different waypoints for sim vs real robot environments
- **Elevator Control**: Real robot publishes multiple times with delays for reliability

### Workflow Steps

1. **[Step 1/8]** Set initial pose for localization
2. **[Step 2/8]** Navigate to loading position
3. **[Step 3/8]** Attach to shelf (via `/approach_shelf` service)
   - Move under shelf using laser intensity detection
   - Raise elevator
   - Update robot footprint to 0.55m octagon (larger with shelf)
4. **[Step 4/8]** Move backward from shelf with curved motion
   - Execute curved backward motion (1.75m at 0.2 m/s with 0.10 rad/s curvature)
   - Perform in-place rotation (-45 degrees)
   - Clears shelf area without collision despite enlarged footprint
   - Eliminates need for loading_2 waypoint navigation
5. **[Step 5/8]** Navigate to shipping position
   - Keepout filter avoids cones area automatically
   - Robot carries shelf safely with enlarged footprint
6. **[Step 6/8]** Detach from shelf
   - Lower elevator
   - Execute straight backward motion (1.5m at 0.2 m/s, no curvature)
   - Restore original robot footprint to 0.30m octagon
7. **[Step 7/8]** Return to init position
8. **[Step 8/8]** Task complete
   - Robot ready for next task

### Topics Used

**Publishers:**

- `/elevator_up` (std_msgs/String) - Raise elevator
- `/elevator_down` (std_msgs/String) - Lower elevator
- `/diffbot_base_controller/cmd_vel_unstamped` (geometry_msgs/Twist) - Manual control during backward motions (sim)
- `/cmd_vel` (geometry_msgs/Twist) - Manual control during backward motions (real robot)

**Service Clients:**

- `/approach_shelf` (attach_shelf/GoToLoading) - Shelf attachment service

**Filter Topics:**

- `/costmap_filter_info` - Filter metadata
- `/keepout_filter_mask` - Keepout mask data

### Manual Velocity Control

The scripts use manual velocity control (Twist messages) for backward motions:

**After Shelf Pickup (Step 4):**

- Curved backward motion: `linear.x = -0.2 m/s`, `angular.z = 0.10 rad/s` for 1.75m
- In-place rotation: `angular.z = -0.7854 rad/s` (≈-45°) for 3 seconds
- Allows robot to clear shelf area without collision

**After Shelf Dropoff (Step 6):**

- Straight backward motion: `linear.x = -0.2 m/s`, `angular.z = 0.0` for 1.5m
- Clean detachment from shelf without curvature

### Troubleshooting Warehouse Task

**Robot not avoiding cones:**

- Verify filter servers are running: `ros2 node list | grep filter`
- Check filter nodes are active: `ros2 lifecycle get /costmap_filter_info_server`
- Verify filter topics exist: `ros2 topic list | grep filter`
- Ensure keepout mask aligns with warehouse map (same resolution/origin)
- Note: Filter servers are automatically launched by `localization.launch.py`

**Shelf attachment fails:**

- Verify `/approach_shelf` service: `ros2 service list | grep approach`
- Check that `approach_service_server_node` is running (automatically launched by pathplanner.launch.py)
- Ensure robot reached correct loading position
- Verify correct config file is loaded (approach_params_sim.yaml or approach_params_real.yaml)

**Elevator not responding:**

- Check elevator topics are published: `ros2 topic echo /elevator_up`
- Verify simulation/robot subscribes to elevator topics
- Real robot: Ensure multiple publish attempts with delays are being executed

**Robot collision after shelf pickup:**

- Verify curved backward motion is executing (Step 4)
- Check footprint was updated to 0.55m before backward motion
- Monitor cmd_vel topic for backward motion commands
- Ensure sufficient clearance in loading area

**Wrong topics being used:**

- Check which config file is being loaded: sim vs real
- Verify `use_sim_time` parameter matches your environment
- For simulation: Topics should use `/diffbot_base_controller/` prefix
- For real robot: Topics should be `/cmd_vel`, `/odom` (no prefix)

## Notes

- Always source `~/sim_ws/install/setup.bash` before launching to access robot meshes
- All packages support both simulation and real robot with automatic configuration selection
- Launch files use PythonExpression for dynamic config selection based on parameters
- Detection logic uses `'real' in filename` for flexible sim/real switching
- See **Simulation vs Real Robot Differences** table for detailed parameter variations
- The `approach_service_server_node` is automatically launched by pathplanner.launch.py with correct configuration
- Curved backward motion after shelf pickup eliminates need for additional waypoint navigation
- Log levels: Major milestones at INFO level, detailed progress at DEBUG level
