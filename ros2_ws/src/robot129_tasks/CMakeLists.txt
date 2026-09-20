cmake_minimum_required(VERSION 3.16)
project(robot129_tasks)
find_package(ament_cmake REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Some transitive rclcpp/tracetools dependencies (fmt, lttng-ust, lttng-ust-common)
# are requested via bare -l flags without an accompanying -L, and this conda-based
# ROS env isn't on the linker's default search path. The unversioned .so symlinks
# exist (verified under $ENV{HOME}-relative env_robot129_ros/lib); they're just not
# found without this. Local to this package only, no change to the shared env.
if(DEFINED ENV{CONDA_PREFIX})
  link_directories($ENV{CONDA_PREFIX}/lib)
endif()

add_executable(robot129_mtc_plan src/robot129_mtc_plan.cpp)
ament_target_dependencies(robot129_mtc_plan moveit_task_constructor_core rclcpp sensor_msgs)

add_executable(robot129_mtc_pick_place src/robot129_mtc_pick_place.cpp)
ament_target_dependencies(robot129_mtc_pick_place
  moveit_task_constructor_core moveit_task_constructor_msgs moveit_msgs
  shape_msgs geometry_msgs rclcpp visualization_msgs)

install(TARGETS robot129_mtc_plan robot129_mtc_pick_place DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
