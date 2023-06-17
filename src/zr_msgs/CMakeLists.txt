cmake_minimum_required(VERSION 2.8.3)
project(zr_msgs)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	std_msgs
	geometry_msgs
  sensor_msgs
  rospy
)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  error_code.msg
  robot_monitor.msg
  mission.msg
  driving_cancel.msg
  delivery_cancel.msg
  delivery_status.msg
  driving_status.msg
  arrive.msg
  goal_info.msg
  external_action.msg
  lift_status.msg
  robot_status.msg
  locker_info.msg
  motor_stream.msg
  motor_info.msg
  battery_info.msg
  gps_anchor.msg
  gps_anchor_array.msg
  occupancy_rate.msg
  AvoidData.msg
  LED.msg
  Point_id.msg
  patrol_target.msg
  task_status.msg
  ultrasonic_info.msg
  EV_params.msg
  mission.msg
  order_info.msg
  single_operation.msg
  task.msg
  cabin_status.msg
  cabin.msg
  patrol_info.msg
  imu_date.msg
  clean_motor.msg
  clean_motor_pub.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  DeviceKey.srv
  DeviceStatus.srv
  DoorControl.srv
  Register.srv
  AccessControl.srv
  patrol_task.srv
  capture_panorama.srv
  take_elevator.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
  sensor_msgs
)
