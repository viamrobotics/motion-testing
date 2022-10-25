
#include <ompl-evaluation/interfaces/ArmPlanningEvalInterface.hpp>

#include <boost/format.hpp>
#include <boost/program_options.hpp>

#include <cstdint>
#include <string>
#include <vector>

//! TODO(wspies)
namespace bpo = boost::program_options;

//! Convenience alias to shortcut getting to the PlannerChoices namespace
using PlannerChoices = ompl_evaluation::interfaces::PlannerChoices;

//! TODO(wspies)
struct ArgStruct
{
  //! TODO(wspies)
  std::string scene;
  
  //! TODO(wspies)
  int planner;
  
  //! TODO(wspies)
  double time;
};


bool parseArgs(int argc, char* argv[], ArgStruct& settings)
{
  try
  {
    bpo::options_description desc("OMPL w/ RDK Planning Evaluator\n\n"
      "Invocation: <program> <scene> <time> <planner(int)>\n"
      "Invocation: <program> --scene <scene> --time <time> --planner <planner(int)>\n\n"
      "Arguments");

    desc.add_options()
      ("help,h", "Prints help for this application")
      ("scene", bpo::value<std::string>(&settings.scene)->default_value("scene1"),
        "Specifies which scene file should be used when setting up the world")
      ("time", bpo::value<double>(&settings.time)->default_value(5.0),
        "Specifies how long an OMPL planner should be allowed to plan, in seconds")
      ("planner", bpo::value<int>(&settings.planner)->default_value(0),
        "Specifies which OMPL planner should be used when planning");

    bpo::positional_options_description pos_desc;
    pos_desc.add("scene", 1);
    pos_desc.add("time", 1);
    pos_desc.add("planner", 1);

    bpo::variables_map var_map;
    bpo::store(bpo::command_line_parser(argc, argv).options(desc).positional(pos_desc).run(), var_map);

    if (var_map.count("help"))
    {
      std::cout << desc << std::endl;
      return false;
    }

    bpo::notify(var_map);
    return true;
  }
  catch (bpo::error& ex)
  {
    std::cout << "Error loading parameters: " << ex.what() << std::endl;
    return false;
  }
}

int main(int argc, char* argv[])
{
  ArgStruct user_settings;
  bool is_ok = parseArgs(argc, argv, user_settings);

  if (is_ok)
  {
    const std::string scene_name = user_settings.scene;

    PlannerChoices planner_choice;
    std::string planner_name;
    switch (user_settings.planner)
    {
      default:
      {
        std::cout << "!!! Unknown planner option specified, defaulting to RRT*" << std::endl;
      }
      case (PlannerChoices::RRTstar):
      {
        planner_choice = PlannerChoices::RRTstar;
        planner_name = "RRT*";
        break;
      }
      case (PlannerChoices::InformedRRTstar):
      {
        planner_choice = PlannerChoices::InformedRRTstar;
        planner_name = "Informed RRT*";
        break;
      }
    }

    // Setting up evaluation parameters
    const int scene_dof = 6;  // TODO(wspies): Still have to deal with this not being defaulted to 6    

    ompl_evaluation::interfaces::PlanEvaluationParams eval_params;
    eval_params.scene_name = scene_name;
    eval_params.arm_dof = std::uint8_t(scene_dof);
    eval_params.goal_threshold = 1e-6;
    eval_params.planner = planner_choice;
    eval_params.planner_time = user_settings.time;
    eval_params.check_time = 0.001;

    // Initialize a scene based on input arguments, calls Init function from Golang bindings
    std::cout << "\nEvaluating planner performance with [ " << scene_name << " ] scene selected." << std::endl;
    std::cout << "Planning with OMPL's [ " << planner_name << " ] planner..." << std::endl;
    std::cout << std::setprecision(2.0) << std::fixed << "Evaluating planner performance when given [ "
              << user_settings.time << " ] seconds to plan...\n" << std::endl;
  
    GoString rdk_scene = {scene_name.c_str(), ptrdiff_t(scene_name.length())};
    Init(rdk_scene);

    // Getting scene details after initialization
    double* start_pos = StartPos();

    struct pose* goal_pose = GoalPose();
    double* goal_pos = ComputePose(goal_pose);

    struct limits* joint_limits = Limits();
    // TODO(wspies): Any assertions on sizes needed here?

    for (int j = 0; j < scene_dof; ++j)
    {
      eval_params.start.push_back(start_pos[j]);
      eval_params.goal.push_back(goal_pos[j]);
    }

    for (int k = 0; k < scene_dof; ++k)
    {
      eval_params.arm_limits.push_back(joint_limits[k]);
    }

    // TODO(wspies)
    ompl_evaluation::interfaces::ArmPlanningEvalInterface eval_arm_planner(eval_params);
    eval_arm_planner.configure();

    ompl::geometric::PathGeometric* path = eval_arm_planner.solve();
    if (path)
    {
      std::cout << "Found solution. Generating evaluation data..." << std::endl;

      eval_arm_planner.visualizePath(path);
      eval_arm_planner.exportPathAsCSV(path, user_settings.scene);
    }
    else
    {
      std::cout << "No solution found. Generating evaluation data..." << std::endl;
    }

    eval_arm_planner.printResults();
    eval_arm_planner.exportResultsAsCSV(user_settings.scene);
    return EXIT_SUCCESS;
  }
  else
  {
    std::cout << "Exiting..." << std::endl;
    return EXIT_FAILURE;
  }
}
