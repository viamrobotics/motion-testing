
#include <ompl-evaluation/interfaces/ArmPlanningEvalInterface.hpp>

#include "bindings.h"

#include <cstdint>
#include <string>
#include <vector>

int main(int argc, char* argv[])
{
  // TODO(wspies)
  ompl_evaluation::interfaces::PlanEvaluationParams eval_params;
  eval_params.arm_name = "arm";
  eval_params.arm_kinematics_file = "/home/wspies/workspace/rdk/components/arm/xarm/xarm7_kinematics.json";
  eval_params.arm_dof = 7;
  eval_params.start = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00};
  eval_params.goal  = {-M_PI_2, -M_PI, M_PI_2, M_PI, -M_PI_2, M_PI, M_PI_2};
  eval_params.goal_threshold = 0.01;
  eval_params.planner = ompl_evaluation::interfaces::PlannerChoices::RRTstar;
  eval_params.planner_time = 1.0;

  // TODO(wspies)
  ompl_evaluation::interfaces::ArmPlanningEvalInterface eval_arm_planner(eval_params);
  eval_arm_planner.configure();
  if (eval_arm_planner.solve())
  {
    // eval_arm_planner.exportPlan();
    ;
  }

  return 0;
}
