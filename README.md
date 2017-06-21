# touch_avoidance_of_robot_manipulation
finally project

                                                  Hybrid Control

Summary
This method takes force or moment goals and combines them through the use of null space projections. We can try different types of task. Here we choice touch avoidance. Set the set-point to zero. Let the manipulator hang in the air, then with your finger press the end-effector in any direction (linear or angular motion). The manipulator should move away from the direction of push to return to a 0 force 0 torque configuration.

Quickstart
Here I will describe how to quickly use the program.
//First we need roscd to correct workspace.
~$ roscd
//If you can’t roscd to baxter_ws, you need source some setup.*sh files.
~$ source ~/ros/indigo/baxter_ws/devel/setup.bash
~$ roscd
//Then you need use roslaunch command add package add files to start up gazebo.
~$ roalaunch birl_baxter_descripution pick_n_box_gazebo.launch
~$ ./baxter.sh sim
~$ rosrun birl_baxter_simulator polishing.py
// To this place we start up our simulation

Overview
The main work of this final project is the use of force control to achieve collision avoidance. The core idea of the control is to increase the force change to the position increment, and then add this position increment to the current position，thus generate the next current position. 
1. Input a Fdes (the desired force), then let it add F of robot joints ,we can get it by FT sensor , then an error e; 
2. Taking this error e as the input of the PID control, the output u (F) is obtained using a suitable PID parameter; 
3. The output of the PID becomes the increment of the position; 
4. From the inverse kinematics the robot can move to its current position, then add increment of the position to position of the current robot by the kinematics. This sum is applied to the robot so that the robot reaches the next current position. This is a cycle. 
5. As described in 4, there is also another loop: repeat 1 we can gets the next PID input. 
All of the above description is the core idea of this final project.

Structure
Below is the control picture of the program：
(can not add picture)

FAQ
Our expect the input is read from the sensor, this value is instability, which makes the PID control input with uncertainty, this increase of the difficulty of PID control.
There is no detailed analysis of the mechanical model of baxter in our work. PID parameter adjustment is also a challenging problem.
