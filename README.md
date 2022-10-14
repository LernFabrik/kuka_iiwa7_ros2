# kuka_iiwa7_ros2
`ros2 param set /move_group use_sim_time true`

ros2 run  --prefix 'gdbserver localhost:3000' wsg50_driver gripper_server_node

### Todo bugs
1. `getCurrentState()` is not working. See [Moveit2 Issue](https://github.com/ros-planning/moveit2/issues/1399). 