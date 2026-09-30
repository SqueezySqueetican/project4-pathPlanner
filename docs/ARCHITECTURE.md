## ARCHITECTURE.md

```markdown
# Architecture & Design

## System Overview

## Core Components

### 1. Grid (`Grid.hpp/cpp`)

**Responsibility:** Spatial representation of the environment.

**Key features:**
- Creates a 2D grid of points with uniform spacing
- Supports marking obstacles
- Finds 4-directional neighbors of any point

**Data flow:**
- Input: Grid dimensions, spacing, obstacle positions
- Output: Valid points, neighbor lists

### 2. PID Controller (`PID.hpp/cpp`)

**Responsibility:** Feedback control for velocity.

**Algorithm:** u(t) = Kp * e(t) + Ki * ∫e(t)dt + Kd * de(t)/dt

**Output:** velocity command

### 3. Controller (`Controller.hpp/cpp`)

**Responsibility:** High-level control using two PID controllers.

**Inputs:**
- Robot pose (x, y, theta)
- Target position
- Navigation state

**Outputs:** Twist command (linear + angular velocity)

**Logic:**
- MOVING state: Both PIDs active
- ROTATING state: Only angular PID (linear = 0)
- STOPPING/FINISHED: Both velocities = 0

### 4. Simulator (`Simulator.hpp/cpp`)

**Responsibility:** Realistic robot dynamics simulation.

**Kinematics:** Differential drive (unicycle model)
x += v * cos(theta) * dt
y += v * sin(theta) * dt
theta += omega * dt

**Constraints applied:**
- Velocity limits (max linear/angular)
- Acceleration limits (max accel/decel)

### 5. Navigator (`Navigator.hpp/cpp`)

**Responsibility:** Waypoint management and state transitions.

**States:**
| State    | Description |
|----------|-------------|
| MOVING   | Moving toward current waypoint |
| ROTATING | Rotating in place before next segment |
| STOPPING | Stopped (waiting/deciding) |
| FINISHED | Reached end of path |

**Logic:**
1. Check distance to current waypoint
2. If within tolerance → advance to next waypoint
3. If direction change needed → enter ROTATING state
4. When rotation complete → return to MOVING state

## Data Flow Diagram
[User] → main.cpp
│
├─→ Grid (environment)
│
├─→ Path (waypoints)
│
├─→ Navigator
│ ├─→ update(position) → state + target
│ └─→ getTarget() → current waypoint
│
├─→ Controller
│ ├─→ computeCommand(pose, target, state)
│ └─→ → Twist (cmd)
│
└─→ Simulator
├─→ setVelocityCommand(cmd)
├─→ update(dt)
└─→ getPose() → new position
│
└─→ back to Navigator (loop)