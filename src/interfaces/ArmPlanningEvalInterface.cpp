#include <ompl-evaluation/interfaces/ArmPlanningEvalInterface.hpp>

#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/Path.h>
#include <ompl/base/terminationconditions/CostConvergenceTerminationCondition.h>
#include <ompl/geometric/PathGeometric.h>

#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <cstdint>
#include <functional>
#include <iostream>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

using namespace ompl_evaluation::interfaces;

bool isStateValid(const ompl::base::SpaceInformation* si, const ompl::base::State* joints_state)
{
  // This is using RDK to determine whether or not a state is valid. Sampled joint positions are passed to RDK functions
  // that perform FK, collision checking, and other state validity checks.

  bool is_satisfied = true;

  // Check joint positions against the RDK scene
  double *joint_angles = joints_state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  GoFloat64 jointData[] = {joint_angles[0], joint_angles[1], joint_angles[2], joint_angles[3], joint_angles[4],
                           joint_angles[5]};
  GoSlice joints = {jointData, 6, 6};
  const int valid = ValidState(joints);
  is_satisfied &= (bool) valid;

  // Check joint positions against bounds we set when setting up the OMPL StateSpace
  is_satisfied &= si->satisfiesBounds(joints_state);

  return is_satisfied;
}

ArmPlanningEvalInterface::ArmPlanningEvalInterface(const PlanEvaluationParams params)
  : eval_params_(params)
  , eval_results_()
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
  // Set up real vector bounds equal to number of arm joints
  // TODO(wspies): Continuous joints need to have a different representation of their bounds or else you get
  //               a joint wrapping issue similar to what Peter and Ray have already seen.
  ompl::base::RealVectorBounds arm_bounds(params.arm_dof);
  for (size_t k = 0; k < params.arm_dof; ++k)
  {
    arm_bounds.setLow(k, params.arm_limits[k].Min);
    arm_bounds.setHigh(k, params.arm_limits[k].Max);
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
  // NOTE(wspies): Disabled for now, need tuning on cost convergence or exact solution use as a sentinel
  // Set up planner termination conditions to reflect the desired behavior for one run of the evaluation interface
  //TerminalCondition terminate_on_time = ompl::base::timedPlannerTerminationCondition(eval_params_.planner_time);
  //TerminalCondition terminate_on_cost = ompl::base::CostConvergenceTerminationCondition(arm_pdef_, 1, 0.1);
  //TerminalCondition conditions = ompl::base::plannerOrTerminationCondition(terminate_on_time, terminate_on_cost);

  // When calling to solve, capture some data about the evaluation, incl. time to solve (in milliseconds)
  auto start = std::chrono::high_resolution_clock::now();

  // NOTE(wspies): Former statement leverages commented TerminalConditions above
  //ompl::base::PlannerStatus status = arm_planner_->ompl::base::Planner::solve(conditions, eval_params_.check_time);
  ompl::base::PlannerStatus status = arm_planner_->ompl::base::Planner::solve(eval_params_.planner_time);

  auto stop = std::chrono::high_resolution_clock::now();
  eval_results_.actual_time = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start);

  std::stringstream ss;
  ss << std::setprecision(3) << std::fixed << "Time spent on planning : "  << eval_results_.actual_time.count() * 1e-6
     << " milliseconds" << std::endl;
  std::cout << ss.str();

  if (status == ompl::base::PlannerStatus::EXACT_SOLUTION || status == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION)
  {
    eval_results_.available = true;

    ompl::base::PathPtr arm_path = arm_pdef_->getSolutionPath();
    return static_cast<ompl::geometric::PathGeometric *>(arm_path.get());
  }
  else
  {
    eval_results_.available = false;
  }
  
  return nullptr;
}

void ArmPlanningEvalInterface::visualizePath(ompl::geometric::PathGeometric* path)
{
  // Get states that constitute the path
  const ompl::base::StateSpace *space(arm_si_->getStateSpace().get());
  std::vector<ompl::base::State*> states = path->getStates();

  // Convert to a 2D slice
  std::vector<GoSlice> slices;
  std::vector<std::vector<double>> reals(states.size());
  for (int i = 0; i < states.size(); ++i)
  {
    space->copyToReals(reals[i], states[i]);
    GoSlice input = {static_cast<GoFloat64*>(reals[i].data()), static_cast<GoInt>(reals[i].size()),
                     static_cast<GoInt>(reals[i].size())};
    slices.push_back(input);
  }
  GoSlice inputs = {static_cast<GoSlice*>(slices.data()), static_cast<GoInt>(slices.size()),
                    static_cast<GoInt>(slices.size())};

  VisualizeOMPL(inputs);
}

void ArmPlanningEvalInterface::exportPathAsCSV(ompl::geometric::PathGeometric* path, const std::string& filename)
{
  const std::string filename_csv = filename + ".csv";

  // Get states that constitute the path
  const std::vector<ompl::base::State*> states = path->getStates();

  // Open an filestream for the designated filename
  std::fstream plan_file;
  plan_file.open(filename_csv, std::ios::out);

  // Write joint states values to the designated file
  for (int j = 0; j < states.size(); ++j)
  {
    double *state_joint_values = states[j]->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    for (int k = 0; k < eval_params_.arm_dof; ++k)
    {
      plan_file << state_joint_values[k];
      plan_file << (k + 1 == eval_params_.arm_dof ? "\n" : ",");
    }
  }

  plan_file.close();
  std::cout << "Planned path file written to [ " << filename_csv << " ]" << std::endl;
}

void ArmPlanningEvalInterface::printResults()
{
  std::cout << std::endl;
  std::stringstream results_ss;
  results_ss << std::setprecision(3) << std::fixed;
  results_ss << "Evaluation Results for [ "     << eval_params_.scene_name << " ]:\n";
  results_ss << "Plan Availability\t: "         << std::boolalpha << eval_results_.available << "\n";
  results_ss << "Plan Quality\t\t: "            << eval_results_.quality << "\n";
  results_ss << "Planner Performance\t: "       << eval_results_.performance << "\n";
  results_ss << "Actual Planning Time (ns)\t: " << eval_results_.actual_time.count() << "\n";
  std::cout << results_ss.str() << std::endl;
}

void ArmPlanningEvalInterface::exportResultsAsCSV(const std::string& filename)
{
  const std::string filename_csv = filename + "_results.csv";

  // Open an filestream for the designated filename
  std::fstream results_file;
  results_file.open(filename_csv, std::ios::out);

  // Write results data to the designated file
  results_file << eval_results_.available << "," << eval_results_.actual_time.count() << ","
               << eval_results_.quality   << "," << eval_results_.performance << "\n";

  results_file.close();
  std::cout << "Evaluation results file written to [ " << filename_csv << " ]" << std::endl;
}
