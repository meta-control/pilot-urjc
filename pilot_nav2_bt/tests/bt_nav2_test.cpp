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

#include <string>
#include <list>
#include <memory>
#include <vector>
#include <set>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "nav_msgs/msg/path.hpp"

#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"

#include "pilot_nav2_bt/PathPointAt.hpp"

#include "rclcpp/rclcpp.hpp"


#include "gtest/gtest.h"

using namespace std::placeholders;
using namespace std::chrono_literals;


class PathPointAtTest : public pilot_nav2_bt::PathPointAt
{
public:
  PathPointAtTest(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
  : PathPointAt(xml_tag_name, conf)
  {
  }

  void set_node(rclcpp::Node::SharedPtr node)
  {
    node_ = node;
  }

  geometry_msgs::msg::PoseStamped get_point_at_distance_test(
    nav_msgs::msg::Path path, double distance)
  {
    return get_point_at_distance(path, distance);
  }

  double get_angle_to_point_test(const geometry_msgs::msg::PoseStamped & pose)
  {
    return get_angle_to_point(pose);
  }
};

TEST(bt_action, pathpointat_funcs)
{
  BT::NodeConfiguration config;
  config.blackboard = BT::Blackboard::create();
  auto test_node = rclcpp::Node::make_shared("test_node");
  config.blackboard->set("node", test_node);

  tf2_ros::StaticTransformBroadcaster br(*test_node);

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {rclcpp::spin_some(test_node);}
    });

  PathPointAtTest bt_node("test", config);

  // Three paths
  auto path_straight = std::make_unique<nav_msgs::msg::Path>();
  path_straight->header.frame_id = "map";
  path_straight->header.stamp = test_node->now();
  for (int i = 0; i < 100; i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = test_node->now();
    pose.pose.position.x = i * 0.1;
    path_straight->poses.push_back(pose);
  }

  auto path_right = std::make_unique<nav_msgs::msg::Path>();
  path_right->header.frame_id = "map";
  path_right->header.stamp = test_node->now();
  for (int i = 0; i < 100; i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = test_node->now();
    pose.pose.position.x = i * 0.1;
    pose.pose.position.y = i * -0.1;
    path_right->poses.push_back(pose);
  }

  auto path_left = std::make_unique<nav_msgs::msg::Path>();
  path_left->header.frame_id = "map";
  path_left->header.stamp = test_node->now();
  for (int i = 0; i < 100; i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = test_node->now();
    pose.pose.position.x = i * 0.1;
    pose.pose.position.y = i * 0.1;
    path_left->poses.push_back(pose);
  }

  // Tranform from map to base_footprint
  tf2::Stamped<tf2::Transform> map2bf;
  map2bf.frame_id_ = "map";
  map2bf.stamp_ = tf2::timeFromSec(test_node->now().seconds());

  map2bf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.0);
  map2bf.setRotation(q);

  geometry_msgs::msg::TransformStamped map2bf_msg = tf2::toMsg(map2bf);
  map2bf_msg.child_frame_id = "base_footprint";

  br.sendTransform(map2bf_msg);

  auto start = test_node->now();
  rclcpp::Rate rate(10);
  while (rclcpp::ok() && (test_node->now() - start) < 2s && !finish) {
    rate.sleep();
  }

  // Testing straight
  auto point_at_1m_s = bt_node.get_point_at_distance_test(*path_straight, 1.0);
  ASSERT_NEAR(point_at_1m_s.pose.position.x, 1.0, 0.001);
  ASSERT_NEAR(bt_node.get_angle_to_point_test(point_at_1m_s), 0.0, 0.001);
 
  // Testing left
  auto point_at_1m_l = bt_node.get_point_at_distance_test(*path_right, 1.0);
  ASSERT_NEAR(point_at_1m_l.pose.position.x, 0.8, 0.001);
  ASSERT_NEAR(bt_node.get_angle_to_point_test(point_at_1m_l), -M_PI/2.0, 0.001);
  
  // Testing right
  auto point_at_1m_r = bt_node.get_point_at_distance_test(*path_left, 1.0);
  ASSERT_NEAR(point_at_1m_r.pose.position.x, 0.8, 0.001);
  ASSERT_NEAR(bt_node.get_angle_to_point_test(point_at_1m_r), M_PI/2.0, 0.001);
  
  // Rotate robot
  q.setRPY(0.0, 0.0, M_PI);
  map2bf.setRotation(q);
  map2bf.stamp_ = tf2::timeFromSec(test_node->now().seconds());
  map2bf_msg = tf2::toMsg(map2bf);
  map2bf_msg.child_frame_id = "base_footprint";
  br.sendTransform(map2bf_msg);

  start = test_node->now();
  while (rclcpp::ok() && (test_node->now() - start) < 2s && !finish) {
    rate.sleep();
  }


  // Testing straight
  point_at_1m_s = bt_node.get_point_at_distance_test(*path_straight, 1.0);
  ASSERT_NEAR(point_at_1m_s.pose.position.x, 1.0, 0.001);
  ASSERT_NEAR(bt_node.get_angle_to_point_test(point_at_1m_s), M_PI, 0.001);
 
  // Testing left
  point_at_1m_l = bt_node.get_point_at_distance_test(*path_right, 1.0);
  ASSERT_NEAR(point_at_1m_l.pose.position.x, 0.8, 0.001);
  ASSERT_NEAR(bt_node.get_angle_to_point_test(point_at_1m_l), -M_PI/2.0, 0.001);
  
  // Testing right
  point_at_1m_r = bt_node.get_point_at_distance_test(*path_left, 1.0);
  ASSERT_NEAR(point_at_1m_r.pose.position.x, 0.8, 0.001);
  ASSERT_NEAR(bt_node.get_angle_to_point_test(point_at_1m_r), M_PI/2.0, 0.001);

  try {
    finish = true;
    t.join();
  } catch (const std::exception & e) {
    std::cerr << "Error at joining: " << e.what() << std::endl;
  }
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
