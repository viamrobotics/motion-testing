
#include <ompl-evaluation/interfaces/ArmPlanningEvalInterface.hpp>

#include "bindings.h"

#include <cstdint>
#include <string>
#include <vector>

int main(int argc, char* argv[])
{
  // Initialize a scene based on input arguments, calls Init function from Golang bindings
  std::string scene_name = "scene1";

  if (argc == 1)
  {
    std::cout << "No arguments provided, using scene 1 as default and planning with RRT*..." << std::endl;
  }
  else if (argc == 2)
  {
    scene_name = std::string(argv[1]);

    std::cout << "Evaluating planner performance with [ " << scene_name << " ] scene selected." << std::endl;
    std::cout << "No planner argument provided, planning with RRT* as default..." << std::endl;
  }
  GoString rdk_scene = {scene_name.c_str(), ptrdiff_t(scene_name.length())};
  Init(rdk_scene);

  // Getting scene details after initialization
  uintptr_t resPtr = StartPos();
  double* start_pos = (double*) resPtr;

  // TODO(wspies)
  ompl_evaluation::interfaces::PlanEvaluationParams eval_params;
  eval_params.scene_name = scene_name;
  for (int j = 0; j < 6; ++j)
  {
    eval_params.start.push_back(start_pos[j]);
  }
  eval_params.goal  = {-M_PI_2, -M_PI, M_PI_2, M_PI, -M_PI_2, M_PI};
  eval_params.arm_dof = std::uint8_t(eval_params.start.size());
  eval_params.goal_threshold = 0.01;
  eval_params.planner = ompl_evaluation::interfaces::PlannerChoices::RRTstar;
  eval_params.planner_time = 1.0;

  // TODO(wspies)
  ompl_evaluation::interfaces::ArmPlanningEvalInterface eval_arm_planner(eval_params);
  eval_arm_planner.configure();
  
  ompl::geometric::PathGeometric* path = eval_arm_planner.solve();
  if (path != NULL) {
    std::cout << "Found solution:" << std::endl;
    path->printAsMatrix(std::cout);
    eval_arm_planner.visualize(path);
  } else {
    std::cout << "No solution found." << std::endl;
  }
}
