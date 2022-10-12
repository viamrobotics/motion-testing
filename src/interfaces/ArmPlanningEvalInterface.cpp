#include <ompl-evaluation/interfaces/ArmPlanningEvalInterface.hpp>

#include "bindings.h"

#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/Path.h>
#include <ompl/geometric/PathGeometric.h>

#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

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

ArmPlanningEvalInterface::ArmPlanningEvalInterface(const PlanEvaluationParams params)
  : eval_params_(params)
  , arm_ss_(nullptr)
  , arm_si_(nullptr)
  , arm_pdef_(nullptr)
  , arm_planner_(nullptr)
{
}

void ArmPlanningEvalInterface::setStartState(const std::vector<double>& joint_pos)
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

void ArmPlanningEvalInterface::setGoalState(const std::vector<double>& joint_pos, const double threshold = 1e-7)
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

bool ArmPlanningEvalInterface::initSpace(const PlanEvaluationParams& params)
{
  // TODO(viam): Eventually need some Go bindings here I think

  // Set up real vector bounds equal to number of arm joints
  // TODO(wspies): Set this with real arm joint limits after we parse ModelJSON?
  // TODO(wspies): Continuous joints need to have a different representation of their bounds or else you get
  //               a joint wrapping issue similar to what Peter and Ray have already seen.
  ompl::base::RealVectorBounds arm_bounds(params.arm_dof);
  for (size_t k = 0; k < params.arm_dof; ++k)
  {
    arm_bounds.setLow(k, (-3 * M_PI));
    arm_bounds.setHigh(k, (3 * M_PI));
  }

  // Set up real vector state space based on the previously established arm bounds
  arm_ss_ = std::make_shared<ompl::base::RealVectorStateSpace>(params.arm_dof);
  arm_ss_->as<ompl::base::RealVectorStateSpace>()->setBounds(arm_bounds);

  // Finally, construct an instance of SpaceInformation from the arm state space
  arm_si_ = std::make_shared<ompl::base::SpaceInformation>(arm_ss_);
  arm_si_->setStateValidityChecker(
    [this](const ompl::base::State* joints_state) { return isStateValid(this->arm_si_.get(), joints_state); }
  );

  // Call SpaceInformation setup method
  arm_si_->setup();

  return arm_si_->isSetup();
}

bool ArmPlanningEvalInterface::initPlanning(const PlanEvaluationParams& params)
{
  // Build up the problem definition
  arm_pdef_ = std::make_shared<ompl::base::ProblemDefinition>(arm_si_);

  // Specify start and goal states, at least initially. These will likely be updated later.
  setStartState(params.start);
  setGoalState(params.goal, params.goal_threshold);

  // Set the planner the arm is going to use for planning
  // TODO(wspies): Future options can be added here as additional cases, if we want to do that
  switch (params.planner)
  {
    case (PlannerChoices::BITstar):
    {
      arm_planner_= std::make_shared<ompl::geometric::BITstar>(arm_si_);
      break;
    }
    case (PlannerChoices::FMT):
    {
      arm_planner_= std::make_shared<ompl::geometric::FMT>(arm_si_);
      break;
    }
    case (PlannerChoices::InformedRRTstar):
    {
      arm_planner_= std::make_shared<ompl::geometric::InformedRRTstar>(arm_si_);
      break;
    }
    case (PlannerChoices::RRTstar):
    default:
    {
      arm_planner_= std::make_shared<ompl::geometric::RRTstar>(arm_si_);
      break;
    }
  }
  arm_planner_->setProblemDefinition(arm_pdef_);

  // Call Planner setup method
  arm_planner_->setup();

  return arm_planner_->isSetup();
}

bool ArmPlanningEvalInterface::configure()
{
  bool arm_configured = true;

  arm_configured &= initSpace(eval_params_);
  arm_configured &= initPlanning(eval_params_);

  return arm_configured;
}

ompl::geometric::PathGeometric* ArmPlanningEvalInterface::solve()
{
  ompl::base::PlannerStatus status = arm_planner_->ompl::base::Planner::solve(eval_params_.planner_time);

  if (status == ompl::base::PlannerStatus::EXACT_SOLUTION || status == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION)
  {
    ompl::base::PathPtr arm_path = arm_pdef_->getSolutionPath();
    return static_cast<ompl::geometric::PathGeometric *>(arm_path.get());
  }
  return NULL;
}
