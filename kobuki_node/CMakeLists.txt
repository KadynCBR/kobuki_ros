cmake_minimum_required(VERSION 3.5)
project(kobuki_node)

find_package(ament_cmake_ros REQUIRED)
find_package(angles REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(ecl_build REQUIRED)
# TODO(clalancette): ecl_devices is a recursive dependency of kobuki_driver, but
# isn't being picked up properly for some reason.  Figure this out.
find_package(ecl_devices REQUIRED)
find_package(ecl_errors REQUIRED)
find_package(ecl_exceptions REQUIRED)
find_package(ecl_geometry REQUIRED)
find_package(ecl_linear_algebra REQUIRED)
# TODO(clalancette): ecl_mobile_robot is a recursive dependency of kobuki_driver,
# but isn't being picked up properly for some reason.  Figure this out.
find_package(ecl_mobile_robot REQUIRED)
find_package(ecl_sigslots REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kobuki_driver REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

ecl_enable_cxx14_compiler()
ecl_enable_cxx_warnings()

include_directories(include)

add_library(kobuki_ros SHARED
  src/diagnostics.cpp
  src/kobuki_ros.cpp
  src/odometry.cpp
)
ament_target_dependencies(kobuki_ros
  "angles"
  "diagnostic_msgs"
  "diagnostic_updater"
  "geometry_msgs"
  "kobuki_driver"
  "kobuki_ros_interfaces"
  "nav_msgs"
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "std_msgs"
  "tf2"
  "tf2_ros"
)
target_link_libraries(kobuki_ros
  ecl_devices::ecl_devices
  ecl_errors::ecl_errors
  ecl_exceptions::ecl_exceptions
  ecl_geometry::ecl_geometry
  ecl_linear_algebra::ecl_linear_algebra
  ecl_mobile_robot::ecl_mobile_robot
  ecl_sigslots::ecl_sigslots
)

add_executable(kobuki_ros_node src/kobuki_ros_node.cpp)
ament_target_dependencies(kobuki_ros_node
  "rclcpp"
)
target_link_libraries(kobuki_ros_node
  kobuki_ros
)

rclcpp_components_register_nodes(kobuki_ros
  "kobuki_node::KobukiRos")

install(TARGETS kobuki_ros
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS kobuki_ros_node
  DESTINATION lib/kobuki_node
)

# install(DIRECTORY include/${PROJECT_NAME}/
#         DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )
# install(DIRECTORY launch
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
# install(DIRECTORY param
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
# install(DIRECTORY image
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
# install(PROGRAMS scripts/getOdom2D.py
#                  scripts/getYaw.py
#         DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

ament_package()
