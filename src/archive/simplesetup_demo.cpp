// This file's sole purpose is to make sure we have the OMPL set up correctly and we can use software from the OMPL
// within our software.

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/tools/benchmark/MachineSpecs.h>

#include <string>
#include <memory>
#include <vector>

bool isStateValid(const ompl::base::State *state)
{
  return true;
}

int main(int argc, char* argv[])
{
  // Setup state space
  auto space(std::make_shared<ompl::base::SE3StateSpace>());

  // Set bounds on R3
  ompl::base::RealVectorBounds bounds(3);
  bounds.setLow(-1);
  bounds.setHigh(1);
  space->setBounds(bounds);

  // Leverage SimpleSetup
  ompl::geometric::SimpleSetup setup(space);

  setup.setStateValidityChecker([](const ompl::base::State *state) { 
    return isStateValid(state);
  });

  // Set up random start state
  ompl::base::ScopedState<ompl::base::SE3StateSpace> start(space);
  start.random();

  // Set up random goal state
  ompl::base::ScopedState<ompl::base::SE3StateSpace> goal(setup.getSpaceInformation());
  goal.random();

  setup.setStartAndGoalStates(start, goal);

  // Start solving
  ompl::base::PlannerStatus solved = setup.solve(1.0);

  if (solved)
  {
    std::cout << "Found solution:" << std::endl;

    // Print the solution to screen
    setup.simplifySolution();
    setup.getSolutionPath().print(std::cout);
  }

  return 0;
}
