# Warehouse Robot Navigation Project

This project implements a complete autonomous navigation system for a warehouse robot using ROS2 Nav2 stack, including SLAM mapping, AMCL localization, and path planning capabilities.

## Project Structure

```plaintext
warehouse_project/
├── cartographer_slam/      # Task 1: SLAM mapping with Cartographer
├── localization_server/    # Task 2: AMCL localization
└── path_planner_server/    # Task 3: Nav2 path planning and navigation
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
colcon build --packages-select cartographer_slam localization_server path_planner_server
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

**Features:**

- AMCL particle filter localization
- RViz visualization with:
  - Map display
  - Particle cloud visualization
  - LaserScan overlay
  - Robot pose estimation
- Set initial pose using "2D Pose Estimate" tool in RViz
- **Intelligent config selection**: Automatically detects map file and sets `use_sim_time` accordingly
  - `warehouse_map_real.yaml` → uses `amcl_config_real.yaml` with `use_sim_time: False`
  - Other maps → uses `amcl_config_sim.yaml` with `use_sim_time: True`

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

**Features:**

- Full Nav2 navigation stack with 6 nodes:
  - Planner Server (NavFn global planner)
  - Controller Server (DWB local planner)
  - BT Navigator (behavior tree execution)
  - Recovery Behaviors (spin, backup, wait)
  - Lifecycle Manager (node lifecycle management)
  - RViz2 (visualization)
- RViz displays:
  - Global and local costmaps
  - Global and local paths
  - Robot footprint visualization
  - Map and robot model
- Set navigation goals using "2D Nav Goal" tool in RViz
- Autonomous navigation with replanning and recovery behaviors
- **Dynamic configuration**: Automatically selects sim or real configs based on `use_sim_time` parameter
- **Conditional remapping**: `/cmd_vel` remapping only applies in simulation

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

```
map
 └─ odom
     └─ robot_base_footprint
         └─ robot_base_link
             ├─ robot_front_laser_base_link
             ├─ robot_omni_backwheel_link
             ├─ robot_omni_front_leftwheel_link
             └─ robot_omni_front_rightwheel_link
```

### Topic Remappings

**Simulation:**

- `/cmd_vel` → `/diffbot_base_controller/cmd_vel_unstamped`
- Odometry: `/odom`
- LaserScan: `/scan`
- Map: `/map`

**Real Robot:**

- `/cmd_vel` (no remapping)
- Odometry: `/odom`
- LaserScan: `/scan`
- Map: `/map`

## Notes

- Always source `~/sim_ws/install/setup.bash` before launching to access robot meshes
- All packages support both simulation and real robot with automatic configuration selection
- Launch files use OpaqueFunction with inline conditionals for clean, maintainable code
- See **Simulation vs Real Robot Differences** table for detailed parameter variations
