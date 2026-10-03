#include <rlc_planner/trajopt/trajopt_planner_manager.hpp>

#include <rlc_planner/exceptions.hpp>
#include <rlc_planner/trajopt/trajopt_interface.hpp>
#include <rlc_planner/trajopt/trajopt_planning_context.hpp>

#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/utils/logger.hpp>

#include <pluginlib/class_list_macros.hpp>

#include <tesseract_monitoring/constants.h>
#include <tesseract_monitoring/environment_monitor_interface.h>
#include <tesseract_environment/environment.h>

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <rclcpp/exceptions/exceptions.hpp>

#include <rlc_utils/tesseract_utils.hpp>

namespace rlc_planner
{

namespace
{
constexpr char TRAJOPT_PLANNER_ID[] = "rlc_trajopt";
constexpr char TRAJOPT_PLANNER_DESCRIPTION[] = "TrajOpt";

void setError(moveit_msgs::msg::MoveItErrorCodes& ec, int32_t error_val, std::string msg)
{
  ec.val = error_val;
  ec.message = std::move(msg);
  ec.source = TRAJOPT_PLANNER_ID;
}

rclcpp::Logger getLogger()
{
  return moveit::getLogger("rlc_planner.trajopt_planner_manager");
}

template <typename T>
T declareOrGetParameter(rclcpp::Node& node, const std::string& name,
                        const T& default_value)
{
  if (!node.has_parameter(name))
  {
    try
    {
      return node.declare_parameter<T>(name, default_value);
    }
    catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
    {
      // Fall through: parameter was declared by another component (e.g., automatic
      // declaration from overrides).
    }
  }

  T value = default_value;
  (void)node.get_parameter(name, value);
  return value;
}
}  // namespace

TrajOptPlannerManager::TrajOptPlannerManager() = default;

bool TrajOptPlannerManager::initialize(const moveit::core::RobotModelConstPtr& model,
                                       const rclcpp::Node::SharedPtr& node,
                                       const std::string& parameter_namespace)
{
  model_ = model;
  node_ = node;
  parameter_namespace_ = parameter_namespace;

  if (!model_)
  {
    RCLCPP_ERROR(getLogger(), "initialize() failed: RobotModel is null");
    return false;
  }
  if (!node_)
  {
    RCLCPP_ERROR(getLogger(), "initialize() failed: Node is null");
    return false;
  }

  const std::string param_prefix =
      parameter_namespace_.empty() ? std::string{} : (parameter_namespace_ + ".");

  options_ = declareAndLoadOptions(param_prefix);

  if (!initializeTesseractMonitor(node_, options_.monitor))
  {
    RCLCPP_ERROR(getLogger(), "initialize() failed: could not initialize Tesseract monitor");
    return false;
  }

  try
  {
    trajopt_interface_ = std::make_shared<TrajOptInterface>(options_.trajopt);
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(getLogger(),
                 "initialize() failed: could not create TrajOptInterface: %s", ex.what());
    return false;
  }

  return true;
}

TrajOptPlannerOptions TrajOptPlannerManager::declareAndLoadOptions(const std::string& pfx)
{
  TrajOptPlannerOptions opt{};

  // Tesseract monitor connectivity options
  const auto monitor_namespace_param = pfx + "monitor.monitor_namespace";
  const auto env_name_param = pfx + "monitor.env_name";
  const auto wait_timeout_ms_param = pfx + "monitor.wait_timeout_ms";

  opt.monitor.monitor_namespace = declareOrGetParameter<std::string>(
      *node_, monitor_namespace_param, opt.monitor.monitor_namespace);
  // Environment name must match the environment monitor's ID (usually the URDF robot
  // name). Default to the MoveIt robot model name for convenience.
  opt.monitor.env_name =
      declareOrGetParameter<std::string>(*node_, env_name_param, model_->getName());

  const int wait_timeout_ms = declareOrGetParameter<int>(
      *node_, wait_timeout_ms_param, static_cast<int>(opt.monitor.wait_timeout.count()));
  if (wait_timeout_ms >= 0)
  {
    opt.monitor.wait_timeout = std::chrono::milliseconds{ wait_timeout_ms };
  }
  else
  {
    RCLCPP_WARN(getLogger(),
                "monitor.wait_timeout_ms=%d is invalid; using default=%ld",
                wait_timeout_ms, static_cast<long>(opt.monitor.wait_timeout.count()));
  }

  // TrajOpt interface options
  const auto default_num_steps_param = pfx + "trajopt.default_num_steps";
  const auto use_moveit_group_as_tesseract_manipulator_param =
      pfx + "trajopt.use_moveit_group_as_tesseract_manipulator";
  const auto fixed_tesseract_manipulator_group_param =
      pfx + "trajopt.fixed_tesseract_manipulator_group";
  const auto enable_seed_cache_param = pfx + "trajopt.enable_seed_cache";
  const auto enable_last_solution_cache_param =
      pfx + "trajopt.enable_last_solution_cache";

  opt.trajopt.default_num_steps = declareOrGetParameter<int>(
      *node_, default_num_steps_param, opt.trajopt.default_num_steps);
  opt.trajopt.use_moveit_group_as_tesseract_manipulator =
      declareOrGetParameter<bool>(*node_, use_moveit_group_as_tesseract_manipulator_param,
                                  opt.trajopt.use_moveit_group_as_tesseract_manipulator);
  opt.trajopt.fixed_tesseract_manipulator_group =
      declareOrGetParameter<std::string>(*node_, fixed_tesseract_manipulator_group_param,
                                         opt.trajopt.fixed_tesseract_manipulator_group);
  opt.trajopt.enable_seed_cache = declareOrGetParameter<bool>(
      *node_, enable_seed_cache_param, opt.trajopt.enable_seed_cache);
  opt.trajopt.enable_last_solution_cache = declareOrGetParameter<bool>(
      *node_, enable_last_solution_cache_param, opt.trajopt.enable_last_solution_cache);

  return opt;
}

bool TrajOptPlannerManager::initializeTesseractMonitor(
    const rclcpp::Node::SharedPtr& node, const TrajOptPlannerOptions::MonitorOptions& opt)
{
  rclcpp::NodeOptions node_opt;
  node_opt.context(node->get_node_base_interface()->get_context());
  node_opt.start_parameter_services(false);
  node_opt.start_parameter_event_publisher(false);

  const std::string client_node_name =
      node->get_name() + std::string{ "_tesseract_monitor_client" };
  tesseract_monitor_client_node_ =
      std::make_shared<rclcpp::Node>(client_node_name, node->get_namespace(), node_opt);

  auto monitor_interface = rlc_utils::tesseract_utils::makeMonitorInterface(
      tesseract_monitor_client_node_, getLogger(), opt.monitor_namespace, opt.wait_timeout,
      opt.env_name);
  if (!monitor_interface)
  {
    RCLCPP_ERROR(getLogger(), "initialize() failed: could not create monitor interface");
    return false;
  }
  monitor_interface_ =
      std::make_shared<tesseract_monitoring::ROSEnvironmentMonitorInterface>(
          std::move(*monitor_interface));
  return true;
}

std::string TrajOptPlannerManager::getDescription() const
{
  return TRAJOPT_PLANNER_DESCRIPTION;
}

void TrajOptPlannerManager::getPlanningAlgorithms(std::vector<std::string>& algs) const
{
  algs.clear();
  algs.emplace_back(TRAJOPT_PLANNER_ID);
}

bool TrajOptPlannerManager::canServiceRequest(
    const planning_interface::MotionPlanRequest& req) const
{
  if (!model_)
  {
    RCLCPP_ERROR(getLogger(), "Robot model is not initialized");
    return false;
  }
  if (!model_->hasJointModelGroup(req.group_name))
  {
    RCLCPP_ERROR(getLogger(),
                 "Joint model group '%s' is not available in the robot model",
                 req.group_name.c_str());
    return false;
  }
  if (!monitor_interface_)
  {
    RCLCPP_ERROR(getLogger(), "Tesseract monitor interface is not initialized");
    return false;
  }
  if (!trajopt_interface_)
  {
    RCLCPP_ERROR(getLogger(), "TrajOpt interface is not initialized");
    return false;
  }

  const bool planner_id_ok =
      req.planner_id.empty() || (req.planner_id == TRAJOPT_PLANNER_ID);
  return planner_id_ok;
}

planning_interface::PlanningContextPtr TrajOptPlannerManager::getPlanningContext(
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const planning_interface::MotionPlanRequest& req,
    moveit_msgs::msg::MoveItErrorCodes& error_code) const
{
  if (!planning_scene)
  {
    setError(error_code, moveit_msgs::msg::MoveItErrorCodes::FAILURE,
             "PlanningScene is null");
    return nullptr;
  }

  TrajOptPlanningContext::EnvironmentPtr env_snapshot;
  if (!monitor_interface_)
  {
    setError(error_code, moveit_msgs::msg::MoveItErrorCodes::FAILURE,
             "Tesseract monitor interface is not initialized");
    return nullptr;
  }

  try
  {
    auto env_uptr = monitor_interface_->getEnvironment(options_.monitor.monitor_namespace);
    env_snapshot =
        std::shared_ptr<tesseract_environment::Environment>(std::move(env_uptr));
  }
  catch (const std::exception& e)
  {
    setError(error_code, moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED,
             std::string{ "Failed to snapshot Tesseract environment: " } + e.what());
    return nullptr;
  }

  if (!env_snapshot)
  {
    setError(error_code, moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED,
             "Failed to snapshot Tesseract environment");
    return nullptr;
  }

  const std::string context_name =
      req.planner_id.empty() ? std::string{ TRAJOPT_PLANNER_ID } : req.planner_id;

  auto context = std::make_shared<TrajOptPlanningContext>(
      context_name, req.group_name, trajopt_interface_, std::move(env_snapshot));
  context->setPlanningScene(planning_scene);
  context->setMotionPlanRequest(req);

  setError(error_code, moveit_msgs::msg::MoveItErrorCodes::SUCCESS, "");
  return context;
}

void TrajOptPlannerManager::validateRequest(
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const planning_interface::MotionPlanRequest& req) const
{
  if (!planning_scene)
  {
    throw PlanningError(moveit_msgs::msg::MoveItErrorCodes::FAILURE,
                        "PlanningScene is null");
  }
  if (!canServiceRequest(req))
  {
    throw PlanningError(moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED,
                        "Request not supported by planner_id='" + req.planner_id + "'");
  }
  if (!monitor_interface_)
  {
    throw PlanningError(moveit_msgs::msg::MoveItErrorCodes::FAILURE,
                        "Tesseract monitor interface is not initialized");
  }
  if (!trajopt_interface_)
  {
    throw PlanningError(moveit_msgs::msg::MoveItErrorCodes::FAILURE,
                        "TrajOpt interface is not initialized");
  }
}

}  // namespace rlc_planner

PLUGINLIB_EXPORT_CLASS(rlc_planner::TrajOptPlannerManager,
                       planning_interface::PlannerManager)
