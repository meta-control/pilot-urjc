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

#ifndef PILOT_NAV2_BT__PATHPOINTAT_HPP_
#define PILOT_NAV2_BT__PATHPOINTAT_HPP_

#include <string>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "nav_msgs/msg/path.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "rclcpp/rclcpp.hpp"

namespace pilot_nav2_bt
{

class PathPointAt : public BT::ActionNodeBase
{
public:
  explicit PathPointAt(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt() override {};
  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::OutputPort<double>("angle"),
        BT::InputPort<double>("distance")
      });
  }

protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  nav_msgs::msg::Path::UniquePtr path_;

  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  void plan_callback(nav_msgs::msg::Path::UniquePtr msg);
  geometry_msgs::msg::PoseStamped get_point_at_distance(
    nav_msgs::msg::Path path, double distance);
  double get_angle_to_point(const geometry_msgs::msg::PoseStamped & pose);
};

}  // namespace pilot_nav2_bt

#endif  // PILOT_NAV2_BT__PATHPOINTAT_HPP_
