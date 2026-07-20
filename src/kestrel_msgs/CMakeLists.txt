cmake_minimum_required(VERSION 3.8)
project(kestrel_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Takeoff.srv"
  "srv/GotoLocal.srv"
  "msg/DefectEvent.msg"
  "msg/HealthAlert.msg"
  "msg/ObstacleHazard.msg"
  "msg/NavigationAdvisory.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
