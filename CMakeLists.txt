cmake_minimum_required(VERSION 3.16)

project(ompl-evaluation)

add_compile_options(-std=c++11)

find_package(ompl REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)

include_directories(
  include
  omplbindings
  ${OMPL_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_interfaces
  src/interfaces/ArmPlanningEvalInterface.cpp
)

target_link_libraries(${PROJECT_NAME}_interfaces
  ${CMAKE_SOURCE_DIR}/omplbindings/bindings
  ${OMPL_LIBRARIES}
)

### Demo code targets ###
### Can likely be removed at some point ###
add_executable(${PROJECT_NAME}_simplesetup src/archive/simplesetup_demo.cpp)
target_link_libraries(${PROJECT_NAME}_simplesetup
  ${OMPL_LIBRARIES}
)
set_target_properties(${PROJECT_NAME}_simplesetup
  PROPERTIES OUTPUT_NAME simplesetup_demo
  PREFIX ""
)

### Evaluation targets ###
add_executable(${PROJECT_NAME}_planning_evaluator src/planning_evaluator.cpp)
target_link_libraries(${PROJECT_NAME}_planning_evaluator
  ${OMPL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PROJECT_NAME}_interfaces
)
set_target_properties(${PROJECT_NAME}_planning_evaluator
  PROPERTIES OUTPUT_NAME planning_evaluator
  PREFIX ""
)
