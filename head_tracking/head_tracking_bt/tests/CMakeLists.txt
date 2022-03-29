
find_package(head_tracking REQUIRED)
find_package(trajectory_msgs REQUIRED)

set(dependencies
  ${dependencies}
  head_tracking
  trajectory_msgs
)



ament_add_gtest(bt_head_tracking_test bt_head_tracking_test.cpp)
ament_target_dependencies(bt_head_tracking_test ${dependencies})
