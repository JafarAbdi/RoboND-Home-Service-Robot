#xterm -e "roscore" &
#sleep 5
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/jafar/catkin_ws/src/RoboND-Home-Service-Robot/World/MyWorld.world" &
sleep 5

xterm -e "roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 5

xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5

xterm -e "roslaunch turtlebot_teleop keyboard_teleop.launch"

