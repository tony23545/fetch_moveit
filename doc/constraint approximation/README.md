# Planning with Approximated Constraint Manifolds  
This is a practice of this moveit [tutorial](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/planning_with_approximated_constraint_manifolds/planning_with_approximated_constraint_manifolds_tutorial.html)  
This readme file describes the following two works:  
-  construct database
-  make it work on [fetch](http://docs.fetchrobotics.com/introduction.html) robot and [BIT*](https://arxiv.org/pdf/1405.5848.pdf) algorithm  

#### Creating the Constraint Database  
A demo cpp file for the construction of the database for the fetch robot and PR2 robot is in the src folder  
They should be put in your_ws/src/moveit/moveit_planners/ompl/ompl_interface  
First start the robot  
```
roslaunch fetch_moveit_config demo.launch
```
Then run the construction code  
```
rosrun moveit_planners_ompl fetch_construct_state_database
```
Of course, this can integrated in a launch file  
#### Run On BIT*  
Add the following param at launch of the move_group node  
```
<param name="move_group/constraint_approximations_path" value="<path_to_database>"/>
```
Then, everything will be OK if you are using RRT* or other palnner, but there are some problems using BIT*.  
Substitute the PathLengthDirectInfSampler.cpp in ompl/base/samplers/informed/src with the one in src folder, that should works.  
