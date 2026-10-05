cmake_minimum_required(VERSION 3.8)
project(messages)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(std_msgs REQUIRED) #utilie aussi pour le msg
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UsblMessage.msg"
  "msg/GnssMessage.msg"
  "msg/ImuMessage.msg"
  "msg/AcousticPoseMessage.msg"
  "msg/HeadingMessage.msg"
  "msg/PathFollowing2DPoint.msg"
  "srv/DubinsPath.srv"
  "msg/Pose2DEulerAngles.msg"
  "msg/RadarMessage.msg"
  "msg/AhrsMessage.msg"
  "msg/ActuatorControl.msg"
  "msg/ManualControl.msg"
  "msg/OverrideRCIn.msg"
  "msg/ParamEvent.msg"
  "msg/RCIn.msg"
  "msg/State.msg"
  "msg/VfrHud.msg"
  "srv/CommandBool.srv"
  "srv/Wset.srv"
  

  DEPENDENCIES geometry_msgs std_msgs rcl_interfaces sensor_msgs
)
find_package(rosidl_default_generators REQUIRED)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
