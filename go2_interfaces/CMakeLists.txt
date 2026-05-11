cmake_minimum_required(VERSION 3.8)
project(go2_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/BodyHeight.srv"
  "srv/ContinuousGait.srv"
  "srv/Euler.srv"
  "srv/FootRaiseHeight.srv"
  "srv/Mode.srv"
  "srv/Pose.srv"
  "srv/SpeedLevel.srv"
  "srv/SwitchGait.srv"
  "srv/SwitchJoystick.srv"

    # Original go2_interfaces messages
  "msg/Go2Cmd.msg"
  "msg/Go2State.msg"
  "msg/Go2Move.msg"
  "msg/Go2RpyCmd.msg"
  "msg/IMU.msg"
  
  # Merged from unitree_go
  "msg/AudioData.msg"
  "msg/BmsCmd.msg"
  "msg/BmsState.msg"
  "msg/Error.msg"
  "msg/Go2FrontVideoData.msg"
  "msg/HeightMap.msg"
  "msg/InterfaceConfig.msg"
  "msg/LidarState.msg"
  "msg/LowCmd.msg"
  "msg/LowState.msg"
  "msg/MotorCmd.msg"
  "msg/MotorCmds.msg"
  "msg/MotorState.msg"
  "msg/MotorStates.msg"
  "msg/PathPoint.msg"
  "msg/Req.msg"
  "msg/Res.msg"
  "msg/SportModeCmd.msg"
  "msg/SportModeState.msg"
  "msg/TimeSpec.msg"
  "msg/UwbState.msg"
  "msg/UwbSwitch.msg"
  "msg/VoxelMapCompressed.msg"
  "msg/VoxelHeightMapState.msg"
  "msg/WebRtcReq.msg"
  "msg/WirelessController.msg"

  DEPENDENCIES geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()