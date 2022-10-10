cmake_minimum_required(VERSION 3.16)

project(ompl-evaluation)

add_compile_options(-std=c++11)

find_package(ompl REQUIRED)

include_directories(
  include
  omplbindings
  ${OMPL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_interfaces
  src/interfaces/ArmPlanningInterface.cpp
)

target_link_libraries(${PROJECT_NAME}_interfaces
  ${CMAKE_SOURCE_DIR}/omplbindings/bindings.a
  ${OMPL_LIBRARIES}
)

### Demo code targets ###
### Can likely be removed at some point ###
add_executable(${PROJECT_NAME}_simplesetup src/simplesetup_demo.cpp)
target_link_libraries(${PROJECT_NAME}_simplesetup
  ${OMPL_LIBRARIES}
)
set_target_properties(${PROJECT_NAME}_simplesetup
  PROPERTIES OUTPUT_NAME simplesetup_demo
  PREFIX ""
)

### Evaluation code targets ###
add_executable(${PROJECT_NAME}_eval_emptyworld src/eval_emptyworld.cpp)
target_link_libraries(${PROJECT_NAME}_eval_emptyworld
  ${OMPL_LIBRARIES}
  ${PROJECT_NAME}_interfaces
)
set_target_properties(${PROJECT_NAME}_eval_emptyworld
  PROPERTIES OUTPUT_NAME eval_emptyworld
  PREFIX ""
)
