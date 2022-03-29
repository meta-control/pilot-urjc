// Copyright 2019 Intelligent Robotics Lab
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

#include <string>
#include <iostream>
#include <vector>
#include <memory>

#include "head_tracking_bt/TrackObjects.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "head_tracking_msgs/msg/pan_tilt_command.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace head_tracking_bt
{

TrackObjects::TrackObjects(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: head_tracking_bt::BtLifecycleCtrlNode(xml_tag_name, action_name, conf)
{
  config().blackboard->get("node", node_);
  command_pub_ = node_->create_publisher<head_tracking_msgs::msg::PanTiltCommand>("command", 100);
}

BT::NodeStatus
TrackObjects::on_tick()
{
  double pan = 0.0;
  double tilt = 0.0;

  getInput("pan", pan);
  getInput("tilt", tilt);

  if (getInput("pan", pan).has_value() || getInput("tilt", tilt).has_value()) {
    head_tracking_msgs::msg::PanTiltCommand msg;
    msg.pan = pan;
    msg.tilt = tilt;
    command_pub_->publish(msg);
  }

  return BT::NodeStatus::RUNNING;
}

}  // namespace head_tracking_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<head_tracking_bt::TrackObjects>(
        name, "/head_tracker", config);
    };

  factory.registerBuilder<head_tracking_bt::TrackObjects>(
    "TrackObjects", builder);
}
