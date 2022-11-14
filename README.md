# ompl-evaluation

## Overview

The [Open Motion Planning Library (OMPL)](https://ompl.kavrakilab.org/) is a major facilitator to research and development in the field of robotic manipulation and motion planning. In the interests of broadening Viam's motion planning capabilities, eventual utilization of the OMPL from within the `rdk` has been set as a long term objective of the Motion Team. In the shorter term, comparing the performance of supported planners within OMPL to our own motion planning services will be an important element of assessing `rdk` performance. To those ends, this repository is intended to serve as a testing environment for planners, a benchmarking environment for Viam's motion planning, an interface development sandbox, and, over time, help us craft the blueprints to interface `rdk` with the OMPL.

For now, the majority of code in this repository will probably be written in C++ or Python, but over time these files will be amended with or replaced by code written in Golang or Rust.

## Setting Up OMPL

These instructions assume the user will NOT be using OMPL binaries and libraries included with any release version of the Robot Operating System (ROS). Instead, this section will walk through the steps required to build a standalone install of OMPL and include its capabilities as part of other software, such as this repository. Several dependencies are required by the OMPL, including CMake, Boost, and Eigen; depending on the chosen method of installation, additional manual steps may need to be taken before using OMPL as part of your exploration.

### From Source

#### Ubuntu 20.04+

1. Clone the ompl repository onto your local machine by executing `git clone git@github.com:ompl/ompl.git`
    * Cloning to a code workspace or something in the home directory is recommended
2. Within the newly cloned repository, execute `mkdir -p build/Release`
3. Change your directory to the `build/Release` folder you just created and execute `cmake -DCMAKE_INSTALL_PREFIX=/opt/ompl ../..`
    * You can change `/opt/ompl` to another path if you wish to install OMPL somewhere else
4. (OPTIONAL) Execute  `make update_bindings` to generate Python bindings
5. Execute `sudo make install`

Alternatively, the [install-ompl-ubuntu.sh](https://ompl.kavrakilab.org/install-ompl-ubuntu.sh) script may be used with other arguments, but executing this script will immediately install OMPL to `/usr/local`, which may complicate later cleanup.

You will now be able to include libraries and headers from OMPL in your software. If using **CMake**, the following snippets should quickly enable OMPL usage within a project:

```
...
find_package(ompl REQUIRED)

include_directories(
  include
  ${OMPL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_<sample> src/<sample>)
target_link_libraries(${PROJECT_NAME}_<sample>
  ${OMPL_LIBRARIES}
)
...
```

#### Debian Bullseye (e.g. Raspberry Pi OS Lite 64-bit)

WIP

### From Homebrew

WIP

## Usage

Code within this repository can be easily prepared for use by executing the following helper script:

```shell
cd ompl-evaluation
./compile.sh
```

...which will generate Golang bindings and compile the C++ source necessary to generate the `planning_evaluator` binary.

### Running against scenes with the planning_evaluator

Once the `planning_evaluator` is ready, you may pass a number of command line arguments in order to have finer control
over the operation of the underlying `ArmPlanningEvalInterface`.

```shell
planning_evaluator <scene> <time> <planner> <title>
```

#### Options

* **scene** (string, default: scene1)

  Selects the scene (world construction, arm choice, start and goal definitions) to use when performing motion planning

* **time** (double, default: 5)

  Amount of time, in seconds, to plan solutions for the set scene

* **planner** (int, default: 0)

  Enumerated value which represents which planner should be used when planning for a given scene. Choices are 0 (RRT*),
  1 (Informed RRT*), 2 (BIT*), or 3 (Advanced BIT*). Other passed values will default the evaluator to using RRT*.

* **title** (string, optional)

  If given, this will set a prefix for the result and path files generated when the `planning_evaluator` has completed
  its work. Otherwise, the title prefix will match the given scene name.

#### Batch Executions

A script is provided to execute a large number of parallel instances of the `planning_evaluator` with the same scenes,
times, and planners, but with different generated file titles. This is very helpful for providing a lot of data for
particular scenes quickly without overwriting path or result details. This script can be executed by calling...

```shell
./scripts/execute_ompl_tests.sh
```

...which, by default, will execute 100 tests for 20 seconds against all available scenes in batches of 10 jobs. The
results and path files will each have unique names. Customization requires a user to modify constants within the shell
script.

### Running against scenes with RDK

WIP

### Interfacing with RDK

WIP

## Development Notes

WIP
