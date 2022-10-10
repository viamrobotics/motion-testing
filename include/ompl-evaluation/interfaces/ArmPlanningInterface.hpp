#ifndef OMPL_EVALUATION_INTERFACES_ARM_PLANNING_INTERFACE_H
#define OMPL_EVALUATION_INTERFACES_ARM_PLANNING_INTERFACE_H

#include <ompl/base/Planner.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>

#include <string>
#include <memory>
#include <vector>

namespace ompl_evaluation
{
namespace interfaces
{
//! @brief TODO(wspies)
class ArmPlanningInterface
{
public:
  //! @brief Delete default constructor, must construct with arguments
  ArmPlanningInterface() = delete;

  //! @brief Constructor which takes arguments to configure an arm planning interface
  ArmPlanningInterface(const std::string name, const std::uint8_t dof, const std::vector<double>& start_pos,
                       const std::vector<double>& goal_pos, const double goal_threshold);

  //! @brief Default empty destructor
  ~ArmPlanningInterface() = default;

  //! @brief Method to intialize all elements of the OMPL interface. If returning true, user should immediately be able
  //! to plan motion using this object.
  //! @param[in] start     Vector of doubles detailing the start position of each arm joint, in radians
  //! @param[in] goal      Vector of doubles detailing the goal position for each arm joint, in radians
  //! @param[in] threshold Threshold to consider when checking if the goal has been reached, in radians
  bool configure(const std::vector<double>& start_pos, const std::vector<double>& goal_pos, const double threshold);

  //! @brief Tell the planner to start generating a plan to reach the goal
  //! @param[in] solver_time Overall time that the plan solver is allowed to spend on generating a solution, in seconds.
  bool solve(const double solver_time);

private:
  //! @brief Sets the starting joint state of the robot arm
  //! @param[in] joint_pos Vector of doubles detailing the start position of each arm joint, in radians
  void setStartState(const std::vector<double>& joint_pos);

  //! @brief Sets the target joint state of the robot arm
  //! @param[in] joint_pos Vector of doubles detailing the goal position for each arm joint, in radians
  //! @param[in] threshold Threshold to consider when checking if the goal has been reached, in radians
  void setGoalState(const std::vector<double>& joint_pos, const double threshold);

  //! @brief Initialize the OMPL space information constructs
  bool initSpace();

  //! @brief Initialize the OMPL planning constructs, which requires information about what the arm is planning to do
  //! @param[in] start     Vector of doubles detailing the start position of each arm joint, in radians
  //! @param[in] goal      Vector of doubles detailing the goal position for each arm joint, in radians
  //! @param[in] threshold Threshold to consider when checking if the goal has been reached, in radians
  bool initPlanning(const std::vector<double>& start, const std::vector<double>& goal,
                    const double threshold);

  //! Name of the robotic arm
  std::string arm_name_;

  //! Degrees of freedom of the robotic arm (this assumes the arm is an N-count revolute joint chain)
  std::uint8_t arm_dof_;

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

#endif  // OMPL_EVALUATION_INTERFACES_ARM_PLANNING_INTERFACE_H
