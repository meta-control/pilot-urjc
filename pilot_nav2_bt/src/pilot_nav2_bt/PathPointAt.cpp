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


#include <tf2/transform_datatypes.h>
#include <tf2/time.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>
#include <iostream>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "nav_msgs/msg/path.hpp"
#include "pilot_nav2_bt/PathPointAt.hpp"

#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

using std::placeholders::_1;
using namespace std::chrono_literals;
namespace pilot_nav2_bt
{

PathPointAt::PathPointAt(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  config().blackboard->get("node", node_);
  path_sub_ = node_->create_subscription<nav_msgs::msg::Path>("/plan", 100,
    std::bind(&PathPointAt::plan_callback, this, _1));
}

BT::NodeStatus
PathPointAt::tick()
{
  if ((node_->now() - path_->header.stamp) < 5s && path_ != nullptr && !path_->poses.empty()) {

    double distance = 3.0;  // Default value, but it shouldn't be used
    if (getInput("distance", distance).has_value()) {
      geometry_msgs::msg::PoseStamped point_at_distance = get_point_at_distance(*path_, distance);
      double angle = get_angle_to_point(point_at_distance);

      setOutput("angle", angle);

      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Distance not set in PathPointAt");
      return BT::NodeStatus::FAILURE;
    }
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Path not valid");
    return BT::NodeStatus::FAILURE;
  }
}

void
PathPointAt::plan_callback(nav_msgs::msg::Path::UniquePtr msg)
{
  path_ = std::move(msg);
}

geometry_msgs::msg::PoseStamped
PathPointAt::get_point_at_distance(nav_msgs::msg::Path path, double distance)
{
  double distance_aux = 0.0;
  geometry_msgs::msg::PoseStamped current_point = path.poses[0];

  path.poses.erase(path.poses.begin());

  do {
    std::cerr << distance_aux << " => " << current_point.pose.position.x << std::endl;
    double & p1x = current_point.pose.position.x;
    double & p1y = current_point.pose.position.y;
    double & p2x = path.poses[0].pose.position.x;
    double & p2y = path.poses[0].pose.position.y;

    distance_aux += sqrt((p1x - p2x) * (p1x - p2x) + (p1y - p2y) * (p1y - p2y));

    current_point = path.poses[0];
    path.poses.erase(path.poses.begin());
  } while (distance_aux < distance && !path.poses.empty());

  return current_point;
}

double
PathPointAt::get_angle_to_point(const geometry_msgs::msg::PoseStamped & pose)
{
  // We have to transform from map coordinates to base_footrpint to obtaint the angle with 
  // respect the robot 

  geometry_msgs::msg::TransformStamped map2bf_msg;
  try {
    map2bf_msg = tf_buffer_.lookupTransform(
      "map", "base_footprint", tf2::timeFromSec(rclcpp::Time(pose.header.stamp).seconds()));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Impossible to transform from map to base_footprint: %s", ex.what());
    return 0.0;
  }

  tf2::Stamped<tf2::Transform> map2bf;
  tf2::fromMsg(map2bf_msg, map2bf);

  tf2::Vector3 p_map(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
  tf2::Vector3 p_bf = tf2::Transform(map2bf) * p_map;

  return atan2(p_bf.y(), p_bf.x());
}

}  // namespace pilot_nav2_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pilot_nav2_bt::PathPointAt>("PathPointAt");
}
