cmake_minimum_required(VERSION 3.5)
project(head_tracking_bt)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(head_tracking_msgs REQUIRED)

set(dependencies
    rclcpp
    rclcpp_lifecycle
    behaviortree_cpp_v3
    lifecycle_msgs
    geometry_msgs
    ament_index_cpp
    head_tracking_msgs
)

include_directories(include)

add_library(track_objects_bt_node SHARED src/head_tracking_bt/TrackObjects.cpp)
list(APPEND plugin_libs track_objects_bt_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS
  ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  add_subdirectory(tests)
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_export_libraries(${plugin_libs})

ament_package()
