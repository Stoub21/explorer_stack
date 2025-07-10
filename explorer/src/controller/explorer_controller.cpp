// Copyright 2023 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ros2_control_explorer/explorer_controller.hpp"

namespace ros2_control_explorer
{

controller_interface::InterfaceConfiguration RobotController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= 6; ++i) {
    config.names.push_back("joint_" + std::to_string(i) + "/position");
    config.names.push_back("joint_" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::InterfaceConfiguration RobotController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= 6; ++i) {
    config.names.push_back("joint_" + std::to_string(i) + "/position");
    config.names.push_back("joint_" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::CallbackReturn RobotController::on_init()
{
  logger_ = get_node()->get_logger(); 
  RCLCPP_INFO(logger_, "Initializing RobotController...");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController::on_configure(const rclcpp_lifecycle::State &)
{
  // Crée un nœud temporaire uniquement pour accéder aux paramètres
  auto param_node = std::make_shared<rclcpp::Node>(
    "temporary_param_client_node",
    rclcpp::NodeOptions().use_intra_process_comms(false)
  );

  // Crée un client de paramètres pointant vers /robot_state_publisher
  auto param_client = std::make_shared<rclcpp::SyncParametersClient>(
    param_node, "/robot_state_publisher"
  );

  // Attends que le service de paramètres soit disponible
  if (!param_client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(logger_, "Parameter service not available on '/robot_state_publisher'");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Essaye de lire le paramètre
  try {
    urdf_content_ = param_client->get_parameter<std::string>("robot_description");
    RCLCPP_INFO(logger_, "Successfully retrieved robot_description from /robot_state_publisher (%zu characters)",
                urdf_content_.size());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "Failed to get 'robot_description' from /robot_state_publisher: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController::on_activate(const rclcpp_lifecycle::State &)
{
  auto logger_ = get_node()->get_logger();
  model = Model::RobotModel<Model::RobotModelImplType::PINOCCHIO>::loadModelFromString(urdf_content_, "joint_1", "tool0");
    
  velocity_problem = std::make_shared<JointVelocityProblem>(model);  
  main_task = velocity_problem->task_set->add<Task::CartesianVelocity>("MainTask"); 
  regularisation_task = velocity_problem->task_set->add<Task::JointVelocity>("RegularisationTask",1e-5); 
  
  auto joint_configuration_constraint = velocity_problem->constraint_set->add<Constraint::JointConfiguration>("JointConfigurationConstraint");
  auto joint_velocity_constraint = velocity_problem->constraint_set->add<Constraint::JointVelocity>("JointVelocityConstraint");

  dof_ = model->getNrOfDegreesOfFreedom();
  qposition_ = Eigen::VectorXd::Zero(dof_);
  qvelocity_ = Eigen::VectorXd::Zero(dof_);

  robot_state.joint_position.resize(dof_);
  robot_state.joint_velocity.resize(dof_);


  updateJointStates();
  model->setRobotState(robot_state);
  goal_pose_ = pinocchio::SE3(model->getFramePose(model->getTipFrameName()).matrix());

  RCLCPP_INFO(logger_, "dof_ : %d",dof_);

  pinocchio::Model pinocchio_model = model->getModel();
  for (const auto& joint_name : pinocchio_model.names)
  {
    RCLCPP_INFO(logger_, "- %s", joint_name.c_str());
  }

  goal_pose_publisher_ = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>("/debug/goal_pose", 10);
  current_pose_publisher_ = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>("/debug/current_pose", 10);
  publisher_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("/explorer_controller/commands", 10);

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type RobotController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) 
{
  // publish_poses();
  Eigen::VectorXd velocity = updateController(period.seconds());

  std_msgs::msg::Float64MultiArray commands;
  for (int i=0;i<dof_;i++){
    commands.data.push_back(velocity[i]);
  }
  publisher_->publish(commands);
  
  for (auto i = 0; i < dof_; ++i) {
    command_interfaces_[2 * i + 1].set_value(velocity[i]);
  }
  return controller_interface::return_type::OK;
}

Eigen::VectorXd RobotController::updateController(double dt){
  updateJointStates();
  model->setRobotState(this->robot_state);

  // Calculate target input for qontrol
  pinocchio::SE3 current_pose(model->getFramePose(model->getTipFrameName()).matrix());
  const pinocchio::SE3 tipMdes = current_pose.actInv(goal_pose_);
  auto err = pinocchio::log6(tipMdes).toVector();
  Eigen::Matrix<double, 6, 1> p_gains_vect = p_gains_ * Eigen::Matrix<double, 6, 1>::Ones();
  Eigen::Matrix<double, 6, 1> xd_star = p_gains_vect.cwiseProduct(err);

  main_task->setTargetVelocity(xd_star);

  // use qontrole to get the velocity command to the ros2 controller
  try
  {
    velocity_problem->update(dt);
    if (velocity_problem->solutionFound())
    {
      Eigen::VectorXd qposition = robot_state.joint_position;
      Eigen::VectorXd velocity_solution = velocity_problem->getJointVelocityCommand();
      // Eigen::VectorXd position_command = qposition + velocity_solution * dt;
      return velocity_solution;
    }
  }
  catch(const std::exception& e)
  {
    RCLCPP_ERROR(logger_, "Qontrol solution search : %s",e.what());
  }
  Eigen::VectorXd null_cmd = Eigen::VectorXd::Zero(dof_);
  return null_cmd;

}

void RobotController::updateJointStates() {
  for (auto i = 0; i < dof_; ++i) {

    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");
    robot_state.joint_position[i] = position_interface.get_value();
    robot_state.joint_velocity[i] = velocity_interface.get_value();
  }
}

// Publishe Target and Current pose
void RobotController::publish_poses(){
  geometry_msgs::msg::PoseStamped goal_pose_message = geometry_msgs::msg::PoseStamped();
  goal_pose_message.header.stamp = get_node()->now();
  goal_pose_message.header.frame_id = "base_link";
  goal_pose_message.pose.position.x = goal_pose_.translation()[0];
  goal_pose_message.pose.position.y = goal_pose_.translation()[1];
  goal_pose_message.pose.position.z = goal_pose_.translation()[2];
  goal_pose_message.pose.orientation.x = goal_pose_.rotation()(0, 0);
  goal_pose_message.pose.orientation.y = goal_pose_.rotation()(1, 0);
  goal_pose_message.pose.orientation.z = goal_pose_.rotation()(2, 0);
  goal_pose_message.pose.orientation.w = goal_pose_.rotation()(0, 1);
  goal_pose_publisher_->publish(goal_pose_message);

  pinocchio::SE3 current_pose(model->getFramePose(model->getTipFrameName()).matrix());
  geometry_msgs::msg::PoseStamped current_pose_message = geometry_msgs::msg::PoseStamped();
  current_pose_message.header.stamp = get_node()->now();
  current_pose_message.header.frame_id = "base_link";
  current_pose_message.pose.position.x = current_pose.translation()[0];
  current_pose_message.pose.position.y = current_pose.translation()[1];
  current_pose_message.pose.position.z = current_pose.translation()[2];
  current_pose_message.pose.orientation.x = current_pose.rotation()(0, 0);
  current_pose_message.pose.orientation.y = current_pose.rotation()(1, 0);
  current_pose_message.pose.orientation.z = current_pose.rotation()(2, 0);
  current_pose_message.pose.orientation.w = current_pose.rotation()(0, 1);

  current_pose_publisher_->publish(current_pose_message);

}

}  // namespace ros2_control_explorer

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_explorer::RobotController, controller_interface::ControllerInterface)
