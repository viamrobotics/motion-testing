#ifndef OMPL_EVALUATION_INTERFACE_ARM_PLANNING_INTERFACE_H
#define OMPL_EVALUATION_INTERFACE_ARM_PLANNING_INTERFACE_H

#include <ompl/base/Planner.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>

#include <string>
#include <memory>
#include <vector>

namespace ompl_evaluation
{
//! @brief TODO(wspies)
class ArmPlanningInterface
{
public:
  //! @brief Delete default constructor, must construct with arguments
  ArmPlanningInterface() = delete;

  //! @brief Constructor to just take a few up-front arguments about the robot arm so we can configure
  ArmPlanningInterface(std::string name, std::uint8_t dof, std::vector<double>& start_pos,
                       std::vector<double>& goal_pos, double goal_threshold);

  //! @brief Default empty destructor
  ~ArmPlanningInterface() = default;

  //! @brief TODO(wspies)
  bool configure(const std::vector<double>& start_pos, const std::vector<double>& goal_pos, const double threshold);

  //! @brief TODO(wspies)
  bool solve(const double solver_time);

private:
  //! @brief TODO(wspies)
  void setStartState(const std::vector<double>& joint_pos);

  //! @brief TODO(wspies)
  void setGoalState(const std::vector<double>& joint_pos, const double threshold);

  //! @brief TODO(wspies)
  bool initSpace();

  //! @brief TODO(wspies)
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

}  // namespace ompl_evaluation

#endif  // OMPL_EVALUATION_INTERFACE_ARM_PLANNING_INTERFACE_H
