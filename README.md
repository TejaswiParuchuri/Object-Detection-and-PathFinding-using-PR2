# CSE-571-Artificial-Intelligence-Project

Follow the below specified steps in order to install the dependencies required to setup PR2 simulation.

      git clone https://github.com/PR2/pr2_simulator.git
      rosdep install pr2_gazebo
      rosmake pr2_gazebo
      sudo apt-get install ros-kinetic-pr2-teleop
      rosmake pr2_teleop
      
Sample run:

      roslaunch gazebo_ros empty_world.launch
      roslaunch pr2_gazebo pr2.launch
      roslaunch pr2_teleop teleop_keyboard.launch
      
To launch PR2 in the plaza environment, use the following commands:
      
      roslaunch pr2_gazebo pr2_plaza_world.launch
      roslaunch pr2_gazebo pr2.launch
      roslaunch pr2_teleop teleop_keyboard.launch

To Run the inclass API with pr2 copy the cse571_project folder to catkin_ws/src/ and run the demo as described in the api:

	./env_setup.sh
	roscore
	rosrun cse571_project server.py -sub 1 -b 1
	roslaunch cse571_project maze.launch
	rosrun cse571_project move_tbot3.py
	rosrun cse571_project random_walk.py
