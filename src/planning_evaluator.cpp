
#include <ompl-evaluation/interfaces/ArmPlanningEvalInterface.hpp>

#include <cstdint>
#include <string>
#include <vector>

int main(int argc, char* argv[])
{
  // Initialize a scene based on input arguments, calls Init function from Golang bindings
  std::string scene_name = "scene1";
  const int scene_dof = 6;

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
  double* start_pos = StartPos();

  struct pose* goal_pose = GoalPose();
  double* goal_pos = ComputePose(goal_pose);

  struct limits* joint_limits = Limits();
  // TODO(wspies): Any assertions on sizes needed here?

  // Setting up evaluation parameters
  ompl_evaluation::interfaces::PlanEvaluationParams eval_params;
  eval_params.scene_name = scene_name;
  for (int j = 0; j < scene_dof; ++j)
  {
    eval_params.start.push_back(start_pos[j]);
    eval_params.goal.push_back(goal_pos[j]);
  }
  eval_params.arm_dof = std::uint8_t(scene_dof);
  for (int k = 0; k < scene_dof; ++k)
  {
    eval_params.arm_limits.push_back(joint_limits[k]);
  }
  eval_params.goal_threshold = 1e-6;
  eval_params.planner = ompl_evaluation::interfaces::PlannerChoices::RRTstar;
  eval_params.planner_time = 1.0;
  eval_params.check_time = 0.001;

  // TODO(wspies)
  ompl_evaluation::interfaces::ArmPlanningEvalInterface eval_arm_planner(eval_params);
  eval_arm_planner.configure();

  ompl::geometric::PathGeometric* path = eval_arm_planner.solve();
  if (path)
  {
    std::cout << "Found solution. Generating evaluation data..." << std::endl;

    eval_arm_planner.exportPathAsCSV(path, scene_name);
    eval_arm_planner.visualizePath(path);
  }
  else
  {
    std::cout << "No solution found. Generating evaluation data..." << std::endl;
  }
  eval_arm_planner.printResults();
  eval_arm_planner.exportResultsAsCSV(scene_name);
}
