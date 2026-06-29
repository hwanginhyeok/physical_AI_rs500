# Autonomous Driving RS500 — Project Rules

> ROS2-based orchard autonomous-driving tracked vehicle (RS500) software project.
> Perception (camera) → Decision (row following) → Control (CAN/motor) pipeline.

## Tasks

- [CURRENT_TASK.md](CURRENT_TASK.md) | [PREPARED_TASK.md](PREPARED_TASK.md) | [FINISHED_TASK.md](FINISHED_TASK.md)
- [DIFFICULTY.md](DIFFICULTY.md) — Difficult problems & know-how

---

## Session Start Protocol

1. **Check Tasks** — Review `CURRENT_TASK.md` (in progress, blocked, blockers)
2. **Meeting** — Brief the user on the current situation
3. **Direction Discussion** — Decide today's work priorities and direction together

---

## Project Structure

```
physical_AI_rs500/
├── src/
│   ├── ad_bringup/       # Launch files, parameter YAML
│   ├── ad_can_bridge/    # CAN communication (MD2K motor controller)
│   ├── ad_control/       # Motor control, safety stop
│   ├── ad_core/          # Common utils, constants
│   ├── ad_interfaces/    # ROS2 custom messages/services
│   ├── ad_perception/    # Camera perception (crop_row detection)
│   └── ad_planning/      # Path decision, row following
├── TASK.md               # Task index
├── CURRENT_TASK.md       # In progress
├── PREPARED_TASK.md      # Planned + TODO
├── FINISHED_TASK.md      # Completed (current month)
├── TASK_ARCHIVE/         # Monthly completion archive
├── DIFFICULTY.md         # Difficult problems & know-how
├── docs/프로젝트/
│   └── task/             # Task detail logs
└── .claude/
    ├── rules/            # Session rules (auto-loaded)
    └── skills/           # Per-task skills (loaded when needed)
```

## Commands

```bash
# Build
colcon build

# Run all tests
colcon test && colcon test-result --verbose

# Test a specific package
colcon test --packages-select ad_control
PYTHONPATH="src/ad_control:$PYTHONPATH" python3 -m pytest src/ad_control/test/ -v

# Launch (main)
ros2 launch ad_bringup full_system_launch.py      # Full system
ros2 launch ad_bringup simulation_launch.py       # Gazebo simulation
ros2 launch ad_bringup synthetic_test_launch.py   # Synthetic image test
ros2 launch ad_bringup crop_row_test_launch.py    # crop_row standalone test
```

## Tech Stack

- **ROS2** (Humble) — Inter-node communication, launch, parameters
- **Python 3** — All node implementations
- **CAN** — MD2K motor controller control
- **GitHub Actions** — CI pipeline (colcon build + test)

---

## Core Principles

- **Critical thinking partner** — For new features/architecture/technology choices, ask questions and discuss first. For bug fixes/agreed-upon implementations, execute immediately. Details: `.claude/rules/critical-thinking.md`
- **Real-time TASK management** — `CURRENT_TASK.md` / `PREPARED_TASK.md` / `FINISHED_TASK.md` 3-file system. Update immediately on start/completion/discovery

---

## Rules (auto-loaded)

| File | Content |
|------|------|
| `critical-thinking.md` | Execute-immediately vs discuss criteria, question patterns |

## Skills (referenced during work)

| File | Trigger |
|------|--------|
| `simulation-report.md` | When writing a simulation test report |
