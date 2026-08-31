# Building the ORIO workspace

This is a **catkin (ROS 1 Noetic)** workspace. It is built with **`catkin_make`**
using **system Python 3.8** (`/usr/bin/python3`) — *not* the anaconda Python that
may shadow `python3` on the host PATH, and *not* `catkin build` (catkin_tools).

> Note: `build/CMakeCache.txt` pins `PYTHON_EXECUTABLE=/usr/bin/python3`.
> Switching to `catkin build` needs a clean `build/ devel/` and would disrupt the
> demo. Leftover `build_isolated/` / `devel_isolated/` can be deleted.

## Team packages (`src/devel_packages/`)

| Package | Type | Notes |
|---|---|---|
| `orio_core` | pure python lib | Honest, unit-tested core (kinematics, geometry, planning). No ROS. Importable via `from orio_core import ...` after build. |
| `orio` | python nodes | The demo brain: `state_machine.py`, pneumatic control, diagnostics. Depends on `orio_core`, `custom_msgs`. Currently run from source (`python3 state_machine.py`). |
| `custom_msgs` | messages | `AddLabeledItem.srv`. |
| `manipulation` | python nodes + launch | Cameras, TF publishers, MoveIt. |
| `prm` | (not a package) | Exploratory PRM code, marked `CATKIN_IGNORE` (excluded from build). |

Third-party ROS packages live in `src/git_packages/` (git submodules).

## Build

```bash
# Use system python 3.8, not anaconda:
export PATH=/usr/bin:$PATH
source /opt/ros/noetic/setup.bash
cd ~/16662_RobotAutonomy
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
```

(The demo itself builds/runs inside the docker container; these host commands are
for local package validation.)

## Run the pure-core unit tests (no ROS needed)

```bash
cd src/devel_packages/orio_core
python3 -m pytest -q
```

## Verify packaging

```bash
rospack find orio_core        # -> .../src/devel_packages/orio_core
python3 -c "from orio_core import robot_util, planning, perception_geometry"
```
