Requirements:
	
	Python libraries:
		pip install -r requirements.txt
	Moveit:
		sudo apt-get install ros-kinetic-moveit
	pr2_simulator:
		cd ~/catkin_ws/src
		git clone https://github.com/PR2/pr2_simulator.git
		rosmake rosdep
		rosdep install pr2_gazebo
		rosmake pr2_gazebo
	pr2_moveit:
		https://github.com/ros-planning/moveit_pr2.git (included in our project files. A separate install shouldn't be needed.)

Download the Image Classifier model from this link, unzip it, and place it in the smart_pr2/ directory:
	https://drive.google.com/file/d/1hyw8KuHT0pMXhbYn0Ch7UMeGbptjrNK2/view?usp=sharing
	
Known Issues:
	-> Running the Gazebo simulation on a virtual machine can cause a depth-ordering crash. This is well
	documented: http://answers.gazebosim.org/question/16045/camera-issue-when-using-virtual-machine/
		-> Recommended: Run the non-simulation experiments on VM, and use a non-VM machine for physical simulations
		
	-> Running the 'sense_demo.py' for the second time, will use the image from the previous robot state. Hence, it will try to
	predict based off the sensed image at the previous state. There is a problem with the way callback methods work. It is somehow 
	not able to update the data (Image) on every subsequent runs. Hence, it chooses the image from previous timesteps.

Team Members and Contributions:
	All team members contributed equal effort towards completing this project.

	In no particular order, the task allocation looked like this:
		Steve: Computer Vision, Neural Network Training, PR2 Setup
		Tejaswi: Path Finding, Deterministic Planning, Simulation Development
		Kirtus: Motion Planning, Experimental Design, Linking Project Elements

		All Members contributed equally to: Testing, bug fixing, and analysis


Code Citations:
We have made our best effort to cite our code sources within the source code. Some additional
reference came from the following:

	-> MoveIt Tutortials:
		-> http://docs.ros.org/kinetic/api/moveit_tutorials/html/index.html
	-> ROS PR2 Simulator Tutorials:
		-> http://wiki.ros.org/pr2_simulator/Tutorials

In several cases we have utilized implementations from the previous homeworks and modified them to suit
our new simulation environments. Credit for these resources should be attributed to the TAs:
	-> Pulkit Verma, verma.pulkit@asu.edu
	-> Abhyudaya Srinet, asrinet1@asu.edu

And the professor:
	-> Siddharth Srivastava, siddharths@asu.edu

The following experiments are demonstrations of the various AI techniques we have applied to the
PR2 robot.

Experiment 1:
	Name: Path Finding

	How to:
	Run the following commands:

	source ~/catkin_ws/devel/setup.bash
	catkin_make
	rosrun cse571_group5 server_fp.py -cans 4 -cups 4 -sim False
	rosrun cse571_group5 path_generation.py

	Result:
	This will return an example of a succeful path through the environment which can serve as a solution to
	to the task.

	AI Techniques:
	Planning in a deterministic, partially observable environment. Path finding with A*

Experiment 2:
	Name: Pick Demo

	How to:
	Run the following commands:

	source ~/catkin_ws/devel/setup.bash
	catkin_make
	rosrun cse571_group5 server_fp.py -cans 4 -cups 4 -sim True
	roslaunch cse571_group5 maze_1.launch
	roslaunch pr2_moveit_config move_group.launch
	rosrun cse571_group5 pick_demo.py

	Result:
	This will demonstrate the pr2 robot attempting to pick up an object in front of it.

	AI Techniques: Motion Planning in a noisy, stochastic environment. The MoveIt library can plan
	with joints that suffer minor error. The planning algorithms in MoveIt are a form of constraint satisfaction solvers because the joints share movement constraints with each other.

Experiment 3:
	Name: Sense Demo
	How to:
	Run the following commands:

	source ~/catkin_ws/devel/setup.bash
	catkin_make
	rosrun cse571_group5 server_fp.py -cans 4 -cups 4 -sim True
	roslaunch cse571_group5 maze_1.launch
	roslaunch pr2_moveit_config move_group.launch
	rosrun cse571_group5 sense_demo.py

	Result:
	This will demonstrate the pr2 robot using our neural network model to predict if it sees a can or a cup.

	AI Techniques: For this demo we use learning, neural networks, and computer vision to give the PR2 the ability to do useful sensing of its environment.

Experiment 4:
	Name: Full Task
	How to:
	Run the following commands:

	source ~/catkin_ws/devel/setup.bash
	catkin_make
	rosrun cse571_group5 server_fp.py -cans 4 -cups 4 -sim True
	roslaunch cse571_group5 maze_1.launch
	roslaunch pr2_moveit_config move_group.launch
	rosrun cse571_group5 move_pr2.py
	rosrun cse571_group5 path_generation.py

	Note: For this experiment we suggest setting the step size for physics simulation within Gazebo to 0.005

	Result:
	In this experiment we combine all the techniques shown in previous experiments. We attempt to deliver
	correct objects to the bin.

	AI Techniques: This demo combines all of the above techniques.
