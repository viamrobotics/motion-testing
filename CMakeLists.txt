cmake_minimum_required(VERSION 3.16)

project(ompl-evaluation)

find_package(ompl REQUIRED)

include_directories(
  include
  ${OMPL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_simplesetup src/simplesetup_demo.cpp)
target_link_libraries(${PROJECT_NAME}_simplesetup
  ${OMPL_LIBRARIES}
)
set_target_properties(${PROJECT_NAME}_simplesetup
  PROPERTIES OUTPUT_NAME simplesetup_demo
  PREFIX ""
)
