# AgileX Tracer in Genesis Sim

AgileX Tracer simulation demo using [Genesis](https://genesis-embodied-ai.github.io/) simulator.

Tracer's URDF model was derived from the [official repo](https://github.com/agilexrobotics/tracer_ros/tree/master/tracer_description). Gazebo- and ROS-related parts were removed, and complete URDF file was generated from decomposed XACRO files.

## Installation

The [uv](https://docs.astral.sh/uv/) package manager is used for dependencies management. Install uv, clone the repo, create venv and sync dependencies:

```shell
git clone https://github.com/TimeEscaper/genesis-tracer-demo.git
cd genesis-tracer-demo
uv venv
source .venv/bin/activate
uv sync
```

## Running the code

Source venv and run the [main script](main.py):

```shell
source .venv/bin/activate
python3 main.py
```

## Known issues

For some reasons, the model is shaking a lot without reasons. Seems that the root of the issue somewhere in castor wheels. We are trying to identify and fix the issue now.

**Update:**  Genesis Team proposed solutions, see the [issue](https://github.com/Genesis-Embodied-AI/Genesis/issues/1610). They are added into the current branch of this repo.