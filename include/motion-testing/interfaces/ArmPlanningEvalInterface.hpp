#ifndef MOTION_TESTING_INTERFACES_ARM_PLANNING_EVAL_INTERFACE_H
#define MOTION_TESTING_INTERFACES_ARM_PLANNING_EVAL_INTERFACE_H

#include "bindings.h"

#include <ompl/base/Planner.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>
#include <ompl/geometric/PathGeometric.h>

#include <chrono>
#include <string>
#include <memory>
#include <vector>

namespace motion_testing
{
namespace interfaces
{
//! PlannerChoices is an enumeration that allows for more developer-friendly identification of planner choices when
//! deciding what OMPL planner to use
enum PlannerChoices : int
{
  RRTstar = 0,
  InformedRRTstar = 1,
  BITstar = 2,
  AdvancedBITstar = 3
};

//! Alias for a vector of C-struct joint limits, which are defined in the @p omplbindings::bindings library
using LimitsVec = std::vector<limits>;

//! This struct captupres a specific scene and planning parameters used when a ArmPlanningEvalInterface is instantiated
struct PlanEvaluationParams
{
  // Name of the scene used to establish the world when performing evaluations
  std::string scene_name;

  //! Vector of doubles (of length arm_dof) that define the arm's starting joint states, in radians
  std::vector<double> start;

  //! Vector of doubles (of length arm_dof) that define the target joint states the arm should move to, in radians
  std::vector<double> goal;

  //! Degrees of freedom of the selected robotic arm (this assumes the arm is an N-count revolute joint chain)
  int arm_dof;

  //! Joint limits for all joints on the selected robotic arm
  LimitsVec arm_limits;

  //! Threshold to consider when checking if the goal has been reached, in radians
  double goal_threshold;

  //! Which planner should we use when evaluating planner performance?
  PlannerChoices planner;

  //! How much time is the planner allowed to spend planning for this evaluation, in seconds?
  double planner_time;

  //! What is the time interval for checking the designated conditions for planning to terminate, in seconds?
  double check_time;
};

//! This struct contains our calculated Motion Planning Effectiveness metrics
struct PlanEvaluationResults
{
  //! Is the planner able to return a valid plan in the allowed planner time?
  //! Per scope doc... For availability, we’ll capture if the planner was able to create a path in RT (0 or 1).
  bool available;

  //! In nanoseconds, how long did it take for the planner @p solve() method to return a valid plan?
  std::chrono::nanoseconds actual_time;
};

//! @brief The ArmPlanningEvalInterface is an OMPL interface and data collection class that works on some small operator
//! input to set up a motion problem and generate solutions to that problem. A limited set of planning algorithms can be
//! selected in order to solve such problems, each of which are hand-selected to ensure that sampling, problem
//! definitions, optimality conditions, etc. are all compatible by default with the given problem.
class ArmPlanningEvalInterface
{
public:
  //! Alias for @p ompl::base::PlannerTerminationCondition function return signature
  using TerminalCondition = ompl::base::PlannerTerminationCondition;

  //! @brief Delete default constructor, must construct with arguments
  ArmPlanningEvalInterface() = delete;

  //! @brief Constructor which takes arguments to configure a planning interface to the OMPL, particulars on the
  //! evaluation criteria, and configures an interface to the RDK to get an arm kinematic model, environment
  //! construction, and other details
  //! @param[in] params Struct detailing all of the evaluation parameters nessesary to perform a plan evaluation
  ArmPlanningEvalInterface(const PlanEvaluationParams params);

  //! @brief Default empty destructor
  ~ArmPlanningEvalInterface() = default;

  //! @brief Configures all elements of the OMPL interface
  //! @return True if user should immediately be able to plan arm motion using this object; False otherwise
  bool configure();

  //! @brief Tell the planner to start generating a plan to reach the goal
  //! @return The path found by the planner, or NULL if none was found
  ompl::geometric::PathGeometric* solve();

  //! @brief Display a matplotlib rendering of the specified path
  //! @param[in] path Planned path to visualize
  void visualizePath(ompl::geometric::PathGeometric* path);

  //! @brief Export the specified path to a CSV file
  //! @param[in] path     Planned path to export
  //! @param[in] filename Name of the CSV file to generate, will have '.csv' appended at the end
  //! @note One line per joint state (incl. start and goal state), with each joint value separated by a comma
  void exportPathAsCSV(ompl::geometric::PathGeometric* path, const std::string& filename);

  //! @brief Prints the calculated results in a visually appealing format
  void printResults();

  //! @brief Export the evaluation statistics from the last use of this evaluator to a TXT file
  //! @param[in] filename Name of the TXT file to generate, will have '_stats.txt' appended at the end
  //! @note One line per result, with each element in the result struct separated by a comma
  void exportStatsAsTXT(const std::string& filename);

private:
  //! @brief Sets the starting joint state of the robot arm
  //! @param[in] joint_pos Vector of doubles detailing the start position of each arm joint, in radians
  void setStartState(const std::vector<double>& joint_pos);

  //! @brief Sets the target joint state of the robot arm
  //! @param[in] joint_pos Vector of doubles detailing the goal position for each arm joint, in radians
  //! @param[in] threshold Threshold to consider when checking if the goal has been reached, in radians
  void setGoalState(const std::vector<double>& joint_pos, const double threshold);

  //! @brief Initialize the OMPL space information constructs
  //! @param[in] params Struct detailing all of the evaluation parameters nessesary to perform a plan evaluation
  //! @return True if the space has been properly configured
  bool initSpace(const PlanEvaluationParams& params);

  //! @brief Initialize the OMPL planning constructs, which requires information about what the arm is planning to do
  //! @param[in] params Struct detailing all of the evaluation parameters nessesary to perform a plan evaluation
  //! @return True if the planner has been properly configured
  bool initPlanning(const PlanEvaluationParams& params);

  //! Parameters detailing what and how this object will evaluate planning with the RDK and the OMPL
  PlanEvaluationParams eval_params_;

  //! Results of running a given planner and planning parameters against a given scene
  PlanEvaluationResults eval_results_;

  //! OMPL's StateSpace construct for the arm
  ompl::base::StateSpacePtr arm_ss_;

  //! OMPL's SpaceInformation construct detailing how we are planning for the arm
  ompl::base::SpaceInformationPtr arm_si_;

  //! OMPL's ProblemDefinition construct detailing problem information for arm planning purposes
  ompl::base::ProblemDefinitionPtr arm_pdef_;

  //! OMPL's Planner construct
  ompl::base::PlannerPtr arm_planner_;
};

}  // namespace interfaces
}  // namespace motion_testing

#endif  // MOTION_TESTING_INTERFACES_ARM_PLANNING_EVAL_INTERFACE_H
