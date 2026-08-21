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
See the "Task 2.2" section below for how
that plan is produced.

## Prerequisites

- ROS 2 Humble
- Gazebo Ignition Fortress
- Clearpath ROS 2 packages (`clearpath_gz`, `nav2_bringup`)

**Note (ARM64 / Apple Silicon):** `ros_gz`, `gz_ros2_control`, and the Clearpath
stack are not published as ARM64 binaries for Humble and were built from
source into a separate `deps_ws`, sourced ahead of this workspace:

```bash
mkdir -p ~/dev/deps_ws/src && cd ~/dev/deps_ws/src

git clone -b humble https://github.com/clearpathrobotics/clearpath_common.git
git clone -b humble https://github.com/clearpathrobotics/clearpath_config.git
git clone -b humble https://github.com/clearpathrobotics/clearpath_msgs.git
git clone -b humble https://github.com/clearpathrobotics/clearpath_simulator.git
git clone -b humble https://github.com/ros-controls/gz_ros2_control.git
git clone -b humble https://github.com/gazebosim/ros_gz.git

cd ~/dev/deps_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

Source both workspaces in this order before running anything else in this repo:
```bash
source /opt/ros/humble/setup.bash
source ~/dev/deps_ws/install/setup.bash
source ~/dev/phd_assignment_ws/install/setup.bash
export ROS_DOMAIN_ID=0
```

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
  setup_path:=/home/user/dev/phd_assignment_ws/clearpath/r2 \
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

## Known limitations

- No watchdog timeout for stalled goals: if a robot never
  reaches a goal or reports failure, the mission planner does not
  currently detect this.

- Waypoint assignment uses Euclidean distance, which is blind to
  obstacles; `ComputePathToPose` would give a more accurate cost.

- `r0` and `r1` were configured with a reduced sensor set relative to
their default Clearpath `a200` loadout, to improve simulation
performance, `r2` retains the full default sensor loadout .



# Task 2.2 — Semantic Mission Planning and Grounding

## Overview

This directory implements a  pipeline
that grounds a natural-language instruction against a scene graph of
the Task 2.1 warehouse environment, verifies the result
 and produces a task plan. That plan can then be consumed by
`mission_bridge_pkg` (Task 2.1 side) to drive robot navigation 

## Directory structure
sm_mission_planner/
├── JSON/
│ └── scene_graph.json # regions + objects extracted from warehouse.sdf
├── maps/
│ └── overlay.png # scene_graph regions/objects drawn over the .pgm map,
│ # used to visually verify region boundaries
├── ground_stage_region.py # Stage 1: instruction -> region_id
├── ground_stage_object.py # Stage 2: instruction + region_id -> object_id
├── validation.py # safety/consistency checks on the grounded result
└── pipeline.py # orchestrates the full flow end to end

### `JSON/scene_graph.json`
Two dictionaries: `regions` (each with a `bbox` and a `label`) and
`objects` (each with `type`, `position`, and `parent_region`). Derived
from `warehouse.sdf` and verified against the `.pgm` map file (see
`maps/overlay.png`).

### `ground_stage_region.py`
Resolves which region of the warehouse an instruction refers to.
Uses Qwen3:8B via Ollama with the `format` parameter (JSON Schema,
`enum = region_ids`) to constrain the output.
The model cannot return a region id that doesn't exist in the scene graph.

### `ground_stage_object.py`
Given the region resolved in Stage 1, resolves which specific object
within it the instruction refers to. Candidates are scoped to objects
whose `parent_region` matches the resolved region. Returns
`NO_OBJECT` when the instruction refers only to the area, with no
specific object mentioned (e.g. "go to the break area")

### `validation.py`
Checks whether the grounded result can be safely executed. When a
specific object is targeted: existence, region/object hierarchy
consistency, confidence gate, and proximity-based safety against any
`human` in the scene graph. 

### `pipeline.py`
Runs Stage 1 → Stage 2 → validation and prints the resulting task
plan as JSON, along with a formatted robot instruction string.

## Prerequisites

Ollama running with `qwen3:8b` pulled — on the host machine (tested on
macOS, Apple Silicon), since the VM does not have the compute for local
LLM inference:
```bash
ollama pull qwen3:8b
```

Python dependencies:
```bash
pip install -r requirements.txt
```

## Running

**Full pipeline** (Runs Stage 1 (regions), Stage 2(Objects), and validation
in sequence):
```bash
python3 pipeline.py
```
Prompts for an instruction on terminal and prints the resulting task plan.

**Individual stages** (for isolated testing):
```bash
python3 ground_stage_region.py    # prompts for an instruction, prints the obtained region
python3 ground_stage_object.py    # requires importing resolve_region first, see __main__
python3 validation.py             # runs three built-in demo cases against the scene graph
```

## Testing end-to-end with Task 2.1

`pipeline.py`  prints the resulted plan to the terminal; it
does not yet publish directly to a ROS 2 topic (see Known Limitations).

**1.** Run `pipeline.py` on the host machine and copy the printed JSON plan.

**2.** With the Task 2.1 running 

`mission_bridge_node` running inside the VM:

```bash
ros2 run mission_bridge_pkg mission_bridge_node --ros-args -p robot_namespace:=r0
```

**3.** Publish the plan to the topic it listens on:

```bash
ros2 topic pub /semantic_task_plan std_msgs/String   '{data: "{\"executable\": true, \"action\": \"navigate_to\", \"object_id\": \"chair_0\", \"region_id\": \"south_corridor\", \"target_position\": {\"x\": 14.3, \"y\": -5.5}}"}' --once
```

If the plan is executable, `mission_bridge_node` sends the resulting
goal to Nav2.
(navigation goals sent to an object exact center may fail to plan if that point
falls on the object's footprint in the occupancy grid) Adjustments to get the closest
point need to be used.

## Known limitations

- The pipeline runs standalone and does not publish directly
  to `/semantic_task_plan`; connecting the two is a manual copy/paste
  step for now.
- Event re-planning (target occluded, anomaly detected, safety
  violation) was observed during design but not implemented.
  
- Two directions of confidence miscalibration: overconfidence when
 a region is  ambiguous (e.g.
  "go to the area and find shelf", where shelves exist in 4 of 5
  regions), and  low confidence on `NO_OBJECT` responses
  even when correct.
- `closest_point_in_bbox` (used by `mission_bridge_node` for area-only
  plans) clamps to the region's bounding box without checking
  occupancy; 

- `mission_bridge_node` sends the target object's exact center position
  as the navigation goal. Since this point can coincide with the
  object's own footprint in the occupancy grid, `GridBased` planning
  may fail to find a valid path within the default tolerance. An
  approach-offset point near the object, rather than its exact center,
  would resolve this not implemented in this deliverable.



