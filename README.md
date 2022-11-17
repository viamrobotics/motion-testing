# motion-testing

## Overview

At present, development of motion planning within Viam benefits from significant virtualized testing. Checking algorithmic performance, behavior of motion planners, forward and inverse kinematic solver performance, and tracking collision checking performance are all facilitated by the use of **scenes**. Scenes are combinations of specific robotic arms, world configurations, start and goal states, and other input parameters that comprise a repeatable test. These scenes allow the Motion Planning team to evaluate our capabilities in a more integrated fashion.

Below this section, a comprehensive overview of the scenes (including their objectives, what features they demonstrate, and how they tie into user stories) is included.

In addition, the Motion Planning team also benefits from comparisons to state-of-the-art implementations provided by other motion planning libraries, namely the **[Open Motion Planning Library (OMPL)](https://ompl.kavrakilab.org/)**. In the shorter term, comparing the performance of supported planners within OMPL to our own motion planning services will be an important element of assessing the competitiveness of motion services provided by `rdk`. To those ends, this repository also serves as a testing environment for OMPL's included planners, a benchmarking environment for Viam's motion planning against planning provided by the OMPL, an interface development sandbox, and, over time, perhaps facilitate deeper collaboration between `rdk` and the OMPL.

## Scenes

WIP

## The Open Motion Planning Library

The [Open Motion Planning Library (OMPL)](https://ompl.kavrakilab.org/) is a major facilitator to research and development in the field of robotic manipulation and motion planning. This software is not typically included in any Mac OS or Linux OS installation, so instructions follow for setting up, compiling, and including software from the OMPL in a local environment.

For now, the majority of OMPL-leveraging code in this repository will probably be written in C++ or Python, but over time these files will be amended with or replaced by code written in Golang or Rust.

### Setting Up OMPL

These instructions assume the user will NOT be using OMPL binaries and libraries included with any release version of the Robot Operating System (ROS). Instead, this section will walk through the steps required to build a standalone install of OMPL and include its libraries as part of other software, such as the `planning_evaluation` scripts within this repository. Several dependencies are required by the OMPL, including **CMake**, **Boost**, and **Eigen**; depending on the chosen method of installation, additional manual steps may need to be taken before using the OMPL as part of your development.

#### From Source

##### Ubuntu 20.04+

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

##### Debian Bullseye (e.g. Raspberry Pi OS Lite 64-bit)

WIP

#### From Homebrew

WIP

## Usage

Code within this repository can be easily prepared for use by executing the following helper script:

```shell
cd motion-testing
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
