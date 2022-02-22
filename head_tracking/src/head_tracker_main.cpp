// Copyright 2021 Intelligent Robotics Lab
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

#include <memory>

#include "head_tracking/HeadController.hpp"
#include "head_tracking_msgs/msg/pan_tilt_command.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node_head_controller = std::make_shared<head_tracking::HeadController>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_head_controller->get_node_base_interface());

  node_head_controller->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
