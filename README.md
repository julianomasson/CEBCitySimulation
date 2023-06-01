# CEBCitySimulation

1 - source /opt/ros/iron/setup.bash
2 - source install/setup.bash
3 - roscore?
4 - To build -> colcon build --packages-select py_pubsub
5 - To run -> ros2 run py_pubsub talker --ros-args -p json_path:=./json_talker.json
              ros2 run py_pubsub listener --ros-args -p json_path:=./json_listener.json
