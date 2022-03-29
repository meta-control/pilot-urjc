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

#ifndef HEAD_TRACKING_BT__TRACKOBJECTS_HPP_
#define HEAD_TRACKING_BT__TRACKOBJECTS_HPP_

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "head_tracking_bt/ctrl_support/BTLifecycleCtrlNode.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "head_tracking_msgs/msg/pan_tilt_command.hpp"
#include "rclcpp/rclcpp.hpp"

namespace head_tracking_bt
{

class TrackObjects : public head_tracking_bt::BtLifecycleCtrlNode
{
public:
  explicit TrackObjects(
    const std::string & xml_tag_name,
    const std::string & node_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<double>("pan"),
        BT::InputPort<double>("tilt")
      });
  }

  BT::NodeStatus on_tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<head_tracking_msgs::msg::PanTiltCommand>::SharedPtr command_pub_;
};

}  // namespace head_tracking_bt

#endif  // HEAD_TRACKING_BT__TRACKOBJECTS_HPP_
