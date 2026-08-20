# Task 2.1 — Multi-Robot ROS 2 Simulation & Mission Planning

## Overview

The first package developed is (`mission_planner_pkg`) implements a centralized mission
planner for a multi-robot Husky fleet (`r0`, `r1`, `r2`) in a Gazebo
simulation. The node assigns waypoints to robots and tracks
their state (`IDLE` / `BUSY`) as goals are received and completed.

The second package, `mission_bridge_pkg`, connects this stack to the
semantic mission planning pipeline built in Task 2.2: it subscribes to
a topic carrying a grounded task plan and forwards it to Nav2 as a
`NavigateToPose` goal. 
See `src/sm_mission_planner/README.md` for how
that plan is produced.

## Prerequisites

- ROS 2 Humble
- Gazebo Ignition Fortress
- Clearpath ROS 2 packages (`clearpath_gz`, `nav2_bringup`)

## Build

From the workspace root:
```bash
cd ~/dev/phd_assignment_ws
colcon build --symlink-install
source install/setup.bash
```

## Running the stack

Each step below runs in its own terminal, from the workspace root
(`ltu-rai-assignment-3568-2026`), or how the folder is named in this order. 
The commands shown bring up a single robot (`r0`) end to end.

**1. Gazebo simulation**
```bash
ros2 launch clearpath_gz gz_sim.launch.py world:=warehouse
```

**2. Spawn the robot**
```bash
ros2 launch clearpath_gz robot_spawn.launch.py \
  setup_path:=/home/user/dev/phd_assignment_ws/clearpath/r0 \
  world:=warehouse x:=0.0 y:=0.0 z:=0.1 yaw:=0.0
```

```bash
ros2 launch clearpath_gz robot_spawn.launch.py \
  setup_path:=/home/user/dev/phd_assignment_ws/clearpath/r1 \
  world:=warehouse x:=2.0 y:=0.0 z:=0.1 yaw:=0.0
```

```bash
ros2 launch clearpath_gz robot_spawn.launch.py \
  setup_path:=/home/user/dev/phd_assignment_ws/clearpath/r1 \
  world:=warehouse x:=4.0 y:=0.0 z:=0.1 yaw:=0.0
```

**3. Nav2 bringup**
```bash
ros2 launch nav2_bringup bringup_launch.py \
  namespace:=r0 use_namespace:=true use_sim_time:=true \
  map:=/home/user/dev/phd_assignment_ws/map/warehouse_demo.yaml \
  params_file:=nav2_config/nav2_params_r0.yaml \
  autostart:=true use_composition:=False
```

Wait for `Managed nodes are active` in the log before proceeding. Then,
in RViz2 (see step 4) or via `/r0/initialpose`, set the robot's initial
pose — Nav2 will not plan without it.

```bash
ros2 topic pub -1 /r0/initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{header: {frame_id: "map"}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}'

ros2 topic pub -1 /r1/initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{header: {frame_id: "map"}, pose: {pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}'

ros2 topic pub -1 /r2/initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{header: {frame_id: "map"}, pose: {pose: {position: {x: 4.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}'


```

**4. RViz2** (optional, for visualization)
```bash
rviz2 -d rviz2/nav2_r0.rviz
```

**5. Mission planner node**
```bash
ros2 run mission_planner_pkg mission_planner_node
```

**6. Mission bridge node** (integration point with Task 2.2)
```bash
ros2 run mission_bridge_pkg mission_bridge_node --ros-args -p robot_namespace:=r0
```
This subscribes to `/semantic_task_plan` and forwards executable plans
to Nav2 as navigation goals. See `src/sm_mission_planner/README.md` for
how to generate and publish a plan.

## Known limitations

- No watchdog timeout for stalled goals: if a robot never
  reaches a goal or reports failure, the mission planner does not
  currently detect this.
- Waypoint assignment uses Euclidean distance, which is blind to
  obstacles; `ComputePathToPose` would give a more accurate cost.
- `r0` and `r1` were configured with a reduced sensor set relative to
their default Clearpath `a200` loadout, to improve simulation
performance, `r2` retains the full default sensor loadout .

- `mission_bridge_node` sends the target object's exact center position
  as the navigation goal. Since this point can coincide with the
  object's own footprint in the occupancy grid, `GridBased` planning
  may fail to find a valid path within the default tolerance. An
  approach-offset point near the object, rather than its exact center,
  would resolve this not implemented in this deliverable.

