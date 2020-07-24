# Robotics-Walking-OP3

## Description
This is a graduation project in Hanyang University, 2020.   
This project is about applying walking algorithm that was stated in SIMBICON paper to the robot OP3. 
This is done in the simulation environment using Gazebo simulator
## Environment
OS    : Ubuntu 16.04   
ROS   : Kinetic   
Gazebo : 7.16.0   
## ROBOTICS
### ROS
In project we used ROS as a robot operating system. ROS is an open source meta-OS for robot. 
This provides several useful packages that could serve the message between the process, 
frequently used functionalities while dealing with robots, tool or library that could build, execute the code.
You can see the ROS official site link in below.   
https://www.ros.org/  
 
### Gazebo
Gazebo is a robot simulator that provides essential tool for testing algorithm, designing robots, etc.
We used this simulator to test the algorithm fits well with our model, OP3.       
You can see the Gazebo official site link on here.
http://gazebosim.org/
### ROBOTIS OP3 
![op3](https://user-images.githubusercontent.com/57992058/88378395-ddbe7580-cddb-11ea-9487-88a58b7b3100.jpg)  
ROBOTIS OP3 is the latest miniature humanoid robot platform from ROBOTIS.
With this robot model, we are developing walking algorithm in Gazebo simulator.
In ROBOTIS official site, it is providing the simulation model that could be tested in Gazebo.
You can see the official site link on here.
https://emanual.robotis.com/docs/en/platform/op3/introduction/   

## SIMBICON Walking Algorithm
![simbicon](https://user-images.githubusercontent.com/57992058/88378211-86b8a080-cddb-11ea-8879-43f2be3bfa3f.PNG)     

We refered to the thesis of SIMBICON: Simple Biped Locomotion Control published by Yin, Panne and Loken in University of British Columbia.
In this paper, it shows that biped locomotion could be held in 4 state transition.
With this transition, there are also several methods like maintaining the balance, controling stance leg torque.
We applied the 4 state transition, balance feedback and PD controller to perform walking through OP3.


site where you can see the thesis : https://www.cs.ubc.ca/~van/papers/simbicon.htm
## Progress (~ 2020.07.24)

