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

#ifndef ROS2_CONTROL_EXPLORER_CONTROLLER_HPP_
#define ROS2_CONTROL_EXPLORER_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <iostream>
#include <fstream>
#include <functional>

#include <controller_interface/controller_interface.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "Qontrol/Qontrol.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>


#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

using namespace Qontrol;

namespace ros2_control_explorer
{
class RobotController : public controller_interface::ControllerInterface
{
public:

  rclcpp::Logger logger_{rclcpp::get_logger("unnamed_controller")};

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

private:
  void updateJointStates();
  Eigen::VectorXd updateController(double dt);

  void publish_poses();
  
  std::string urdf_content_;
  Eigen::VectorXd qposition_;
  Eigen::VectorXd qvelocity_;
  pinocchio::SE3 goal_pose_;
  double p_gains_ = 10.0;
  Eigen::Matrix<double, 6, 1> p_gains_vect_;
  std::vector<int> passage_;
  int dof_ = 6;

  std::shared_ptr<Model::RobotModel<Model::RobotModelImplType::PINOCCHIO>> model;
  std::shared_ptr<Qontrol::JointVelocityProblem> velocity_problem;
  std::shared_ptr<Qontrol::Task::CartesianVelocity<Qontrol::ControlOutput::JointVelocity>> main_task;
  std::shared_ptr<Task::JointVelocity<ControlOutput::JointVelocity>> regularisation_task;
  Qontrol::RobotState robot_state;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_publisher_;

};

}  // namespace ros2_control_explorer

#endif  // ROS2_CONTROL_EXPLORER_CONTROLLER_HPP_
