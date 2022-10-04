# ompl-evaluation

## Overview

The Open Motion Planning Library (OMPL) is a major facilitator to research and development in the field of robotic manipulation and motion planning. In the interests of broadening Viam's motion planning capabilities, eventual utilization of the OMPL from within the `rdk` has been set as a long term objective of the Motion Team. In the shorter term, comparing the performance of supported planners within OMPL to our own motion planning services will be an important element of assessing `rdk` performance. To those ends, this repository is intended to serve as a testing environment for planners, a benchmarking environment for Viam's motion planning, an interface development sandbox, and, over time, help us craft the blueprints to interface `rdk` with the OMPL.

For now, the majority of code in this repository will probably be written in C++ or Python, but over time these files will be amended with or replaced by code written in Golang or Rust.

## Setting Up OMPL

These instructions assume the user will NOT be using OMPL binaries and libraries included with any release version of the Robot Operating System (ROS). Instead, this section will walk through the steps required to build a standalone install of OMPL and include its capabilities as part of other software, such as this repository. Several dependencies are required by the OMPL, including CMake, Boost, and Eigen; depending on the chosen method of installation, additional manual steps may need to be taken before using OMPL as part of your exploration. At the end of this section, you should be able to 

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

### From Homebrew

WIP

## Usage

Code within this repository can be easily compiled by executing the following commands:

```shell
mkdir build && cd build
cmake ..
make
```

Which should create new executable code within the build directory, depending on how CMakeLists is structured.

### Testing

WIP

### Benchmarking

WIP

### Interfacing with RDK

WIP

## Development Notes

WIP
