
#include <ompl-evaluation/interfaces/ArmPlanningInterface.hpp>

#include <string>
#include <vector>

int main(int argc, char* argv[])
{
  const std::vector<double> start = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00};
  const std::vector<double> goal  = {-M_PI_2, -M_PI, M_PI_2, M_PI, -M_PI_2, M_PI};

  ompl_evaluation::interfaces::ArmPlanningInterface xarm6("xarm", 6, start, goal, 0.01);

  xarm6.solve(1.0);

  return 0;
}