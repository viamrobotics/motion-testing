#ifndef OMPL_EVALUATION_INTERFACES_ARM_PLANNING_EVAL_INTERFACE_H
#define OMPL_EVALUATION_INTERFACES_ARM_PLANNING_EVAL_INTERFACE_H

#include <ompl/base/Planner.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>
#include <ompl/geometric/PathGeometric.h>

#include <string>
#include <memory>
#include <vector>

namespace ompl_evaluation
{
namespace interfaces
{
//! TODO(wspies)
enum PlannerChoices : std::uint8_t
{
  RRTstar = 0,
  InformedRRTstar = 1,
  FMT = 2,
  BITstar = 3
};

//! TODO(wspies)
struct PlanEvaluationParams
{
  // Name of the scene used to establish the world when performing evaluations
  std::string scene_name;

  //! Vector of doubles (of length dof) that define the arm's starting joint states, in radians
  std::vector<double> start;

  //! Vector of doubles (of length dof) that define the target joint states the arm should move to, in radians
  std::vector<double> goal;

  //! Degrees of freedom of the selected robotic arm (this assumes the arm is an N-count revolute joint chain)
  std::uint8_t arm_dof;

  //! Threshold to consider when checking if the goal has been reached, in radians
  double goal_threshold;

  //! Which planner should we use when evaluating planner performance?
  PlannerChoices planner;

  //! How much time is the planner allowed to spend planning for this evaluation?
  double planner_time;
};

//! @brief TODO(wspies)
class ArmPlanningEvalInterface
{
public:
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
  //! @param[in] path Path to visualize
  void visualize(ompl::geometric::PathGeometric* path);

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
}  // namespace ompl_evaluation

#endif  // OMPL_EVALUATION_INTERFACES_ARM_PLANNING_EVAL_INTERFACE_H
