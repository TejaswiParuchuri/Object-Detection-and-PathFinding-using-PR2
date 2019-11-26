# CSE-571-Artificial-Intelligence-Project

#### Install the required python libraries
	pip install -r requirements.txt
	
#### Installing MoveIt (a library which helps to control the movement of the end effectors of PR2's arms in simulation)
	sudo apt-get install ros-kinetic-moveit
	cd ~/catkin_ws/src
	git clone https://github.com/ros-planning/moveit_pr2.git

#### Follow the below specified steps in order to install the dependencies required to setup PR2 simulation.
      git clone https://github.com/PR2/pr2_simulator.git
      rosdep install pr2_gazebo
      rosmake pr2_gazebo
      sudo apt-get install ros-kinetic-pr2-teleop
      rosmake pr2_teleop

#### Running test pick
	./env_setup.sh
      roscore
      rosrun cse571_project server_fp.py
      roslaunch cse571_project maze.launch
      rosrun cse571_project move_pr2.py
      roslaunch pr2_moveit_config move_group.launch
      rosrun cse571_project test_pick.py

#### Running a random walk algorithm with PR2 in the plaza environment
      ./env_setup.sh
      roscore
      rosrun cse571_project server_fp.py
      roslaunch cse571_project maze.launch
      rosrun cse571_project move_pr2.py
      rosrun cse571_project random_walk.py      

#### To Run the inclass API with PR2, copy the cse571_project folder to catkin_ws/src/ and run the demo as described in the API:

	./env_setup.sh
	roscore
	rosrun cse571_project server.py -sub 1 -b 1
	roslaunch cse571_project maze.launch
	rosrun cse571_project move_tbot3.py
	rosrun cse571_project random_walk.py
