# Robot Butler Simulation for Cafe Delivery  

This repository contains the necessary files and instructions to simulate a robot butler in a cafe environment using ROS2. The robot navigates using the `turtlebot3_navigation2` package and executes delivery tasks based on the provided scenarios.

---

## Dependencies

- **ROS2 Humble** installed on your system.  
- **TurtleBot3** package installed (`turtlebot3` and `turtlebot3_navigation2`).  
- Ensure the `delivery_bot` package is in your workspace and properly sourced.  

---

## Setup Instructions  

### Step 1:
I have provided map1.yaml file and map1.pgm file in config folder , you have to move this in Home directory. then run the below command.

---->ros2 launch delivery_bot obstacle_course.launch.py map:=$HOME/map1.yaml

### Step 2:
 
 Then in New terminal we have to run the below command for running the task:-
 
 - ros2 run delivery_bot task1.py   ---> for task1 
 - ros2 run delivery_bot task2.py   ---> for task2
 - ros2 run delivery_bot task3.py   ---> for task3 ....and so on till 7th task

- For giving the confirmation, Run the following command:-  ros2 service call confirmation std_srvs/srv/Empty

- For giving the cancellation, Run the following command:- ros2 service call cancellation std_stvs/srv/Empty

### Important Note:

- For better results, increasing the Maximum range of lidar from the turtlebot3_gazebo package is advisable.

### Results 

[![Task1](https://img.youtube.com/vi/VIDEO_ID/0.jpg)](https://youtu.be/DHQkgJsfRQs?si=lCEfzitkars2R9lq)

[![Task2](https://img.youtube.com/vi/VIDEO_ID/0.jpg)](https://youtu.be/csxCIzHAGMA?si=chZpEBGF_Dxm6-1a)

[![Task3](https://img.youtube.com/vi/VIDEO_ID/0.jpg)](https://youtu.be/_I-FeR1ULCE?si=uSxTUY76h-ljf4LD)

[![Task4](https://img.youtube.com/vi/VIDEO_ID/0.jpg)](https://youtu.be/nz8fDAGP9uA)

[![Task6](https://img.youtube.com/vi/VIDEO_ID/0.jpg)](https://youtu.be/8pmem0Z-eYY)

[![Task7](https://img.youtube.com/vi/VIDEO_ID/0.jpg)](https://youtu.be/-z5vXbp-WlY)





  
