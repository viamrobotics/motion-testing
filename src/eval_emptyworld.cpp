
#include <ompl-evaluation/interfaces/ArmPlanningInterface.hpp>

#include <string>
#include <vector>

int main(int argc, char* argv[])
{
  const std::vector<double> start = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00};
  const std::vector<double> goal  = {-M_PI_2, -M_PI, M_PI_2, M_PI, -M_PI_2, M_PI, M_PI_2};

  ompl_evaluation::interfaces::ArmPlanningInterface xarm7("xarm", 7, start, goal, 0.01);

  xarm7.solve(1.0);

  return 0;
}
