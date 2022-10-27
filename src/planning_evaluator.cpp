
#include <ompl-evaluation/interfaces/ArmPlanningEvalInterface.hpp>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>

#include <cstdint>
#include <string>
#include <vector>

//! Cut down on those massive boost::program_options paths
namespace bpo = boost::program_options;

//! Convenience alias to shortcut getting to the PlannerChoices namespace
using PlannerChoices = ompl_evaluation::interfaces::PlannerChoices;

//! This will help handle passing user configuration details back and forth between @p main() and @p parseArgs()
struct ArgStruct
{
  //! User's choice of scene
  std::string scene;
  
  //! User's choice of motion planning algorithm, see ArmPlanningEvalInterface.hpp for valid options
  int planner;
  
  //! User's specification for allowed planning time
  double time;

  //! If set, string that will be used (instead of scene name) for generated files
  std::string title;
};

bool parseArgs(int argc, char* argv[], ArgStruct& settings)
{
  try
  {
    bpo::options_description desc("OMPL w/ RDK Planning Evaluator\n\n"
      "Invocation: <program> <scene> <time> <planner(int)> <title>\n"
      "Invocation: <program> --scene <scene> --time <time> --planner <planner(int)> --title <title>\n\n"
      "Arguments");

    desc.add_options()
      ("help,h", "Prints help for this application")
      ("scene", bpo::value<std::string>(&settings.scene)->default_value("scene1"),
        "Specifies which scene file should be used when setting up the world")
      ("time", bpo::value<double>(&settings.time)->default_value(5.0),
        "Specifies how long an OMPL planner should be allowed to plan, in seconds")
      ("planner", bpo::value<int>(&settings.planner)->default_value(0),
        "Specifies which OMPL planner should be used when planning")
      ("title", bpo::value<std::string>(&settings.title)->default_value(""),
        "If set, will override the title of any plan and statistics files generated during runtime");

    bpo::positional_options_description pos_desc;
    pos_desc.add("scene", 1);
    pos_desc.add("time", 1);
    pos_desc.add("planner", 1);
    pos_desc.add("title", 1);

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

std::string getResultsPathPrefix(const PlannerChoices& choice)
{
  // This script will need a little filesystem exploration to find the results directory, use . as default otherwise
  std::string path_prefix = ".";
  const auto invoc_path = boost::filesystem::current_path();
  if (invoc_path.stem().string() == "build")
  {
    path_prefix = invoc_path.parent_path().string();
  }
  else if (invoc_path.stem().string() == "ompl-evaluation")
  {
    path_prefix = invoc_path.string();
  }

  path_prefix += "/results/ompl";

  // Also use the planner choice to decide how to name the containment directory, no default (which should stick out)
  std::string planner_code = "";
  switch (choice)
  {
    case (PlannerChoices::RRTstar):
    {
      planner_code = "-RRTstar";
      break;
    }
    case (PlannerChoices::InformedRRTstar):
    {
      planner_code = "-InfRRTstar";
      break;
    }
  }
  path_prefix += planner_code + "/";

  // If we generate a path and find the folder does not exist, filestreaming will have trouble unless we make it
  if (!boost::filesystem::exists(path_prefix))
  {
    std::cout << "Results directory for given planner not found, creating new directory at [ " << path_prefix << " ]..."
              << std::endl;
    boost::filesystem::create_directory(path_prefix);
  }

  return path_prefix;
}

int main(int argc, char* argv[])
{
  ArgStruct user_settings;
  bool is_ok = parseArgs(argc, argv, user_settings);

  if (is_ok)
  {
    const std::string scene_name = user_settings.scene;
    const std::string title_override = user_settings.title;

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
    ompl_evaluation::interfaces::PlanEvaluationParams eval_params;
    eval_params.scene_name = scene_name;
    eval_params.goal_threshold = 1e-6;
    eval_params.planner = planner_choice;
    eval_params.planner_time = user_settings.time;
    eval_params.check_time = 0.001;

    // Initialize a scene based on input arguments, calls Init function from Golang bindings
    std::cout << "\nEvaluating planner performance with [ " << scene_name << " ] scene selected." << std::endl;
    std::cout << "Planning with OMPL's [ " << planner_name << " ] planner..." << std::endl;
    std::cout << std::setprecision(2.0) << std::fixed << "Evaluating planner performance when given [ "
              << user_settings.time << " ] seconds to plan...\n" << std::endl;
    if (!title_override.empty())
    {
      std::cout << "Title override specified, files generated for this evaluation will be prefixed with [ "
                << title_override << " ]...\n" << std::endl;
    }
  
    GoString rdk_scene = {scene_name.c_str(), ptrdiff_t(scene_name.length())};
    Init(rdk_scene);

    // Getting scene details after initialization
    eval_params.arm_dof = NumJoints();

    double* start_pos = StartPos();

    struct pose* goal_pose = GoalPose();
    double* goal_pos = ComputePose(goal_pose);

    struct limits* joint_limits = Limits();

    for (int j = 0; j < eval_params.arm_dof; ++j)
    {
      eval_params.start.push_back(start_pos[j]);
      eval_params.goal.push_back(goal_pos[j]);
    }

    for (int k = 0; k < eval_params.arm_dof; ++k)
    {
      eval_params.arm_limits.push_back(joint_limits[k]);
    }

    // Get the path prefix for the files we are going to generate, then decide which filename to use
    const std::string fs_prefix = getResultsPathPrefix(planner_choice);
    const std::string results_fs_path = fs_prefix + (title_override.empty() ? scene_name : title_override);

    // This creates, configures, and uses the ArmPlanningEvalInterface to generate motion plans
    ompl_evaluation::interfaces::ArmPlanningEvalInterface eval_arm_planner(eval_params);
    eval_arm_planner.configure();

    ompl::geometric::PathGeometric* path = eval_arm_planner.solve();
    if (path)
    {
      std::cout << "Found solution. Generating evaluation data..." << std::endl;

      eval_arm_planner.visualizePath(path);
      eval_arm_planner.exportPathAsCSV(path, results_fs_path);
    }
    else
    {
      std::cout << "No solution found. Generating evaluation data..." << std::endl;
    }

    eval_arm_planner.printResults();
    eval_arm_planner.exportStatsAsTXT(results_fs_path);
    return EXIT_SUCCESS;
  }
  else
  {
    std::cout << "Exiting..." << std::endl;
    return EXIT_FAILURE;
  }
}
