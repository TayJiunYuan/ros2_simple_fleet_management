## ROS2 Simple Fleet Management (Humble)

[![Watch the video](https://img.youtube.com/vi/xutFPUfduec/maxresdefault.jpg)](https://youtu.be/xutFPUfduec)


### Intro

This is a simple fleet management system built with ROS 2 Humble. Code quality is a bit messy as it was assembled quickly in an afternoon. The world is a 6x6 grid with a rectangular obstacle in the middle. Two robots operate on the grid and are assigned goals to reach. New goals can be published at runtime.

Key properties:

- **Grid size**: 6x6
- **Obstacles**: rectangular block at cells `(2,2), (2,3), (3,2), (3,3)`
- **Robots**: 2 robots (`robot1`, `robot2`)
- **Tick**: Fleet manager plans every 3s; robots simulate 1s step latency per move

### System Requirements

- **OS**: Ubuntu 22.04 (Jammy)
- **ROS**: ROS 2 Humble
- **Python deps**: `matplotlib` for the visualizer (TkAgg backend)
  - Install: `python3 -m pip install matplotlib`

### Workspace Setup and Build

```bash
# Create a ROS 2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone this repo
git clone <your-repo-url> ros2_simple_fleet_management

# Build
cd ~/ros2_ws
colcon build

# Source the overlay
source install/setup.bash
```

### How to Run

```bash
# From your workspace root after building
source install/setup.bash
ros2 launch fleet_management_pkg fleet_system.launch.py
```

This launches four nodes:

- `robot1_node`
- `robot2_node`
- `fleet_manager_node`
- `visualizer_node`


### Nodes Overview

#### 1) Fleet Manager Node

- Name: `fleet_manager_node`
- Responsibilities:
  - Service client to query each robot’s current and final positions
  - Publishes the next step (grid cell) to each robot
  - Subscribes for new goals to add to the pool
  - Publishes the full grid state for visualization
- Every 3 seconds, the fleet manager updates the state and sends next-step instructions
- Algorithm summary:
  - Computes current global state (robots, obstacles, goals)
  - Ranks goals per robot by Manhattan distance
    - If both robots are idle and their closest goal ties, compare their second-closest goals. The robot with the closer second goal gets that second goal; the other robot gets the closest tied goal
  - Runs A\* to compute the next single step for the assigned goal
  - Considers the other robot’s current and final positions as temporary obstacles to avoid collisions and target conflicts
  - Publishes next step; if the step equals the goal, removes that goal

Topics:

- Subscribes: `/fleet/new_goal` (`geometry_msgs/Point`)
- Publishes: `/robot1/instruction`, `/robot2/instruction` (`geometry_msgs/Point`)
- Publishes: `/fleet/state` (`std_msgs/String`, JSON-encoded state)

Services (clients):

- `/robot1/get_state` (`fleet_management_pkg/srv/GetRobotState`)
- `/robot2/get_state` (`fleet_management_pkg/srv/GetRobotState`)

Parameters/constants (hardcoded):

- `GRID_SIZE = 6`
- `OBSTACLES = {(2,2), (2,3), (3,2), (3,3)}`
- `TICK_SEC = 3`

#### 2) Robot Nodes

- Names: `robot1_node`, `robot2_node`
- Responsibilities:
  - Service server that returns current and final positions
  - Subscriber for next-step instructions from fleet manager
  - Simulates movement: after receiving a new target cell, waits 1 second and then updates current position to that cell

Topics:

- Subscribes (robot1): `/robot1/instruction` (`geometry_msgs/Point`)
- Subscribes (robot2): `/robot2/instruction` (`geometry_msgs/Point`)

Services (servers):

- `/robot1/get_state` (`fleet_management_pkg/srv/GetRobotState`)
- `/robot2/get_state` (`fleet_management_pkg/srv/GetRobotState`)

Timing:

- `MOVE_DELAY_SEC = 1` per step simulation

#### 3) Visualizer Node

- Name: `visualizer_node`
- Responsibilities:
  - Subscribes to the fleet state and plots the grid, obstacles, goals, and robots
  - Runs the Matplotlib UI loop on a background thread using the TkAgg backend

Topics:

- Subscribes: `/fleet/state` (`std_msgs/String`, JSON with grid, obstacles, robots, goals)

Plot details:

- Blue circles: robot current positions
- Orange small circles: robot final/target positions
- Green circles: goals
- Black squares: obstacles

### Adding Goals at Runtime

Publish a goal as a point (floats are accepted and cast to ints; z is ignored):

```bash
ros2 topic pub --once /fleet/new_goal geometry_msgs/msg/Point "{x: 1.0, y: 5.0, z: 0.0}"
```

### Launch File

The launch file `fleet_system.launch.py` starts the two robots, the fleet manager, and the visualizer.

### Message and Service Definitions

- `fleet_management_pkg/srv/GetRobotState`:
  - Request: empty
  - Response: `int32 current_x`, `int32 current_y`, `int32 final_x`, `int32 final_y`

### Implementation Notes

- The fleet manager publishes the entire serialized state (as JSON) on `/fleet/state` for the visualizer to render.
- The A\* computation is used only to determine the next immediate step, not the entire path.
- Other robots’ current and final positions are treated as blocked cells to reduce collision risk.

### Known Limitations and Improvements

- Hardcoded to exactly 2 robots; goal assignment logic assumes two robots. Make this dynamic by discovering robots or using parameters and generalizing the assignment algorithm.
- Inefficient to run A\* every tick just for the next step. Consider delegating local path planning to robots (e.g., compute full path once, track progress, replan only on conflict or obstacle change).
