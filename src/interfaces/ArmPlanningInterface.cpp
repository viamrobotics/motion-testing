#include <ompl-evaluation/interfaces/ArmPlanningInterface.hpp>

#include "bindings.h"

#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/Path.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

using namespace ompl_evaluation::interfaces;


bool isStateValid(const ompl::base::SpaceInformation* si, const ompl::base::State* joints_state)
{
  // TODO(viam): Need some Go bindings here I think. At a minimum, ComputePosition -> CheckCollisions. If no collisions,
  //             state is valid.

  return si->satisfiesBounds(joints_state);
}

bool isStateValidWithGo(const ompl::base::SpaceInformation* si, const ompl::base::State* joints_state)
{
  GoString kinFile = {"/home/wspies/workspace/rdk/components/arm/xarm/xarm7_kinematics.json", 68};
  Init(kinFile);

  double *joint_angles = joints_state->as<ompl::base::RealVectorStateSpace::StateType>()->values;

  std::cout << "\n>>> Joint angles at current state:\n";
  std::cout << "J1: " << joint_angles[0] << " , " << "J2: " << joint_angles[1] << " , "
            << "J3: " << joint_angles[2] << " , " << "J4: " << joint_angles[3] << " , "
            << "J5: " << joint_angles[4] << " , " << "J6: " << joint_angles[5] << " , "
            << "J7: " << joint_angles[6] << std::endl;

  GoFloat64 jointData[] = {joint_angles[0], joint_angles[1], joint_angles[2], joint_angles[3], joint_angles[4],
                           joint_angles[5], joint_angles[6]};
  GoSlice joints = {jointData, 7, 7};
  GoString armName = {"arm", 3};

  struct pose* p;
  p = ComputePositions(joints);

  std::cout << ">>> FK results for arm at current joint angles:\n";
  std::cout << "X: "     << p->X     << " , "  << "Y: "    << p->Y    << " , " << "Z: "   << p->Z   << " , "
            << "Pitch: " << p->Pitch << " , "  << "Roll: " << p->Roll << " , " << "Yaw: " << p->Yaw << std::endl;

  return si->satisfiesBounds(joints_state);
}

ArmPlanningInterface::ArmPlanningInterface(const std::string name, const std::uint8_t dof,
                                           const std::vector<double>& start_pos, const std::vector<double>& goal_pos,
                                           const double goal_threshold)
  : arm_name_(name)
  , arm_dof_(dof)
  , arm_ss_(nullptr)
  , arm_si_(nullptr)
  , arm_pdef_(nullptr)
  , arm_planner_(nullptr)
{
  // TODO(wspies): Maybe add something that can take an arm component name here and call the relevant methods to marshal
  // JSON necessary to fully understand the kinematic model of the given arm component?
  configure(start_pos, goal_pos, goal_threshold);
}

void ArmPlanningInterface::setStartState(const std::vector<double>& joint_pos)
{
  ompl::base::State* start = arm_si_->getStateSpace()->allocState();
  double *start_joints = start->as<ompl::base::RealVectorStateSpace::StateType>()->values;

  // The allocated state should be of type RealVectorStateSpace with bounds set to length of the joint_pos vector
  // As a result, assignment here should be a simple 1-to-1 mapping.
  for (size_t k = 0; k < joint_pos.size(); ++k)
  {
    start_joints[k] = joint_pos[k];
  }
  arm_pdef_->addStartState(start);

  arm_si_->freeState(start);
}

void ArmPlanningInterface::setGoalState(const std::vector<double>& joint_pos, const double threshold = 1e-7)
{
  ompl::base::State* goal = arm_si_->getStateSpace()->allocState();
  double *goal_joints = goal->as<ompl::base::RealVectorStateSpace::StateType>()->values;

  // The allocated state should be of type RealVectorStateSpace with bounds set to length of the joint_pos vector
  // As a result, assignment here should be a simple 1-to-1 mapping.
  for (size_t k = 0; k < joint_pos.size(); ++k)
  {
    goal_joints[k] = joint_pos[k];
  }
  arm_pdef_->setGoalState(goal, threshold);

  arm_si_->freeState(goal);
}

bool ArmPlanningInterface::initSpace()
{
  // TODO(viam): Need some Go bindings here I think

  // Set up real vector bounds equal to number of arm joints
  // TODO(wspies): Set this with real arm joint limits after we parse ModelJSON?
  // TODO(wspies): Continuous joints need to have a different representation of their bounds or else you get
  //               a joint wrapping issue similar to what Peter and Ray have already seen.
  ompl::base::RealVectorBounds arm_bounds(arm_dof_);
  for (size_t k = 0; k < arm_dof_; ++k)
  {
    arm_bounds.setLow(k, (-3 * M_PI));
    arm_bounds.setHigh(k, (3 * M_PI));
  }

  // Set up real vector state space based on the previously established arm bounds
  arm_ss_ = std::make_shared<ompl::base::RealVectorStateSpace>(arm_dof_);
  arm_ss_->as<ompl::base::RealVectorStateSpace>()->setBounds(arm_bounds);

  // Finally, construct an instance of SpaceInformation from the arm state space
  arm_si_ = std::make_shared<ompl::base::SpaceInformation>(arm_ss_);
  arm_si_->setStateValidityChecker(
    [this](const ompl::base::State* joints_state) { return isStateValidWithGo(this->arm_si_.get(), joints_state); }
  );

  // Call SpaceInformation setup method
  arm_si_->setup();

  return arm_si_->isSetup();
}

bool ArmPlanningInterface::initPlanning(const std::vector<double>& start, const std::vector<double>& goal,
                                        const double threshold = 0.01)
{
  // Build up the problem definition
  arm_pdef_ = std::make_shared<ompl::base::ProblemDefinition>(arm_si_);

  // Specify start and goal states, at least initially. These will likely be updated later.
  setStartState(start);
  setGoalState(goal, threshold);

  // Set the planner the arm is going to use for planning
  // TODO(wspies): Make this configurable later
  arm_planner_ = std::make_shared<ompl::geometric::RRTConnect>(arm_si_);
  arm_planner_->setProblemDefinition(arm_pdef_);

  // Call Planner setup method
  arm_planner_->setup();

  return arm_planner_->isSetup();
}

bool ArmPlanningInterface::configure(const std::vector<double>& start_pos, const std::vector<double>& goal_pos,
                                     const double goal_threshold)
{
  bool arm_configured = true;

  arm_configured &= initSpace();
  arm_configured &= initPlanning(start_pos, goal_pos, goal_threshold);

  return arm_configured;
}

bool ArmPlanningInterface::solve(const double solver_time = 1.0)
{
  ompl::base::PlannerStatus status = arm_planner_->ompl::base::Planner::solve(solver_time);

  if (status == ompl::base::PlannerStatus::EXACT_SOLUTION || status == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION)
  {
    ompl::base::PathPtr arm_path = arm_pdef_->getSolutionPath();
    ompl::geometric::PathGeometric& arm_path_geo = *static_cast<ompl::geometric::PathGeometric *>(arm_path.get());

    std::cout << "Found solution:" << std::endl;
    arm_path_geo.printAsMatrix(std::cout);
    return true;
  }
  else
  {
    std::cout << "No solution found." << std::endl;
    return false;
  }
}
