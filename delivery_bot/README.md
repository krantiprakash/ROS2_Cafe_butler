Step 1:
I have provided map1.yaml file and map1.pgm file in config folder , you have to move this in Home directory. then run the below command.

---->ros2 launch delivery_bot obstacle_course.launch.py map:=$HOME/map1.yaml

Step 2:
 
 Then in New terminal we have to run the command for running the task:- 
 ros2 run delivery_bot task1.py   ---> for task1 
 ros2 run delivery_bot task2.py   ---> for task2
 ros2 run delivery_bot task3.py   ---> for task3 ....


For giving the confirmation, Run the following command:-  ros2 service call confirmation std_srvs/srv/Empty

For giving the cancellation, Run the following cammand :- ros2 service call cancellation std_stvs/srv/Empty


depedndencies:---



You should have turtlebot3 package installed in your computer, because it use turtlebot3_navigation2 package to move.