//adapt it for the fetch robot
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char **argv){
	ros::init(argc, argv, "move_group_interface_tutorial");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	sleep(20.0);

	//setup the MoveGroup class with the name of the movegroup
	moveit::planning_interface::MoveGroup group("arm");

	//use PlanningSceneInterface class to deal directly with world
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	//(optional) create a publisher for visualizing plans in Rviz
	ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path",1,true);
	moveit_msgs::DisplayTrajectory display_trajectory;

	//getting basic information
	//^^^^^^^^^^^^^^^^^^^^^^^^

	ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
	ROS_INFO("End Effector: %s", group.getEndEffectorLink().c_str());

	//planning to a pose goal
	//^^^^^^^^^^^^^^^^^^^^^^^
	//plan a motion for this group to a desired pose for the end-effector
	geometry_msgs::Pose target_pose1;
	target_pose1.orientation.w = 1.0;
	target_pose1.position.x = 0.5;
	target_pose1.position.y = -0.35;
	target_pose1.position.z = 1.0;
	group.setPoseTarget(target_pose1);

	//call the planner to compute plan and visualize it
	//not asking the move_group actually move the robot
	moveit::planning_interface::MoveGroup::Plan my_plan;
	bool success = group.plan(my_plan);

	ROS_INFO("Visualizing plan 1(pose goal) %s", success?"":"FAILED");
	sleep(5.0);

	//visualizing the plans
	if(1){
		ROS_INFO("visualizing plan 1 (again)");
		display_trajectory.trajectory_start = my_plan.start_state_;
		display_trajectory.trajectory.push_back(my_plan.trajectory_);
		display_publisher.publish(display_trajectory);
		sleep(5.0);
	}

	//moving to a pose goal 
	//uncomment below line when working with a real robot
	//this can also work if you run in a simulator(rviz)
	//group.move();

	//planning to a joint-space goal
	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	//first get the current set of joint values for the group
	//note that the current state is the arm keeping straight
	//planning will not change the robot state
	std::vector<double> group_varible_values;
	group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_varible_values);

	//now modify one of the joints, plan to the new joint space goal and visualize the plan
	group_varible_values[0] = -1.0;
	group.setJointValueTarget(group_varible_values);
	success = group.plan(my_plan);

	ROS_INFO("visualizing plan 2 (joint space goal) %s", success?"":"FAILED");
	sleep(5.0);

	//planning with path constraints
	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	//define the path constraint
	moveit_msgs::OrientationConstraint ocm;
	ocm.link_name = "wrist_roll_link";
	ocm.header.frame_id = "base_link";
	ocm.orientation.w = 1.0;
	ocm.absolute_x_axis_tolerance = 0.1;
	ocm.absolute_y_axis_tolerance = 0.1;
	ocm.absolute_z_axis_tolerance = 0.1;
	ocm.weight = 1.0;

	//now set it as the path constraints for the group
	moveit_msgs::Constraints test_constraints;
	test_constraints.orientation_constraints.push_back(ocm);
	group.setPathConstraints(test_constraints);


	//use the old goal that we have planned to it
	//this will only work if the current state already satisfies the path constraints
	//set the start state to a new pose
	robot_state::RobotState start_state(*group.getCurrentState());
	geometry_msgs::Pose start_pose2;
	start_pose2.orientation.w = 1.0;
	start_pose2.position.x = 0.55;
	start_pose2.position.y = 0.3;
	start_pose2.position.z = 0.8;
	const robot_state::JointModelGroup *joint_model_group = start_state.getJointModelGroup(group.getName());
	start_state.setFromIK(joint_model_group, start_pose2);
	group.setStartState(start_state);

	//now we will plan to the earlier pose target from the new start state that we have just created
	group.setPoseTarget(target_pose1);
	success = group.plan(my_plan);

	ROS_INFO("visualizing plan3 (constraints) %s", success?"":"FAILED");
	sleep(10.0);

	//when done with the path constraint be sure to clear it.
	group.clearPathConstraints();


	//Cartesian Paths
	//you can plan a cartesian path directly by specifying a list of waypoints
	//for the end-effector to go through
	//we start from the new start state above
	//the initial pose does not need to be added to the waypoints list
	
	//tony: the start state of the move group will not change even after we call the plan function?

	std::vector<geometry_msgs::Pose> waypoints;

	geometry_msgs::Pose target_pose3 = start_pose2;
	target_pose3.position.x += 0.2;
	target_pose3.position.z += 0.2;
	waypoints.push_back(target_pose3);

	target_pose3.position.y -= 0.2;
	waypoints.push_back(target_pose3);

	target_pose3.position.z -= 0.2;
	target_pose3.position.y += 0.2;
	target_pose3.position.x -= 0.2;
	waypoints.push_back(target_pose3);

	//cartesian path be interpolated at a resolution of 1cm
	//specify 0.01 as the max step in cartesian translation
	//specify the jump threshold as 0.0, effectively disabling it
	moveit_msgs::RobotTrajectory trajectory;
	double fraction = group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
	ROS_INFO("visualizing plan 4 (cartesian path) (%.2f%% achevied)", fraction*100.0);
	sleep(15.0);


	//adding/removing objects and attaching /detaching objects
	//first, define the collision object message
	moveit_msgs::CollisionObject collision_object;
	collision_object.header.frame_id = group.getPlanningFrame();

	collision_object.id = "box1";

	//define a box to add to the world
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 0.4;
	primitive.dimensions[1] = 0.1;
	primitive.dimensions[2] = 0.4;

	geometry_msgs::Pose box_pose;
	box_pose.orientation.w = 1.0;
	box_pose.position.x = 0.6;
	box_pose.position.y = 0;
	box_pose.position.z = 1.2;

	collision_object.primitives.push_back(primitive);
	collision_object.primitive_poses.push_back(box_pose);
	collision_object.operation = collision_object.ADD;

	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.push_back(collision_object);

	//add the collision object into the world
	ROS_INFO("add an object into the world");
	planning_scene_interface.addCollisionObjects(collision_objects);

	sleep(2.0);

	//set the time longer for planning with collision detection
	group.setPlanningTime(10.0);

	//now plan a trajectory avoiding the obstacle
	group.setStartState(*group.getCurrentState());
	group.setPoseTarget(target_pose1);
	success = group.plan(my_plan);

	ROS_INFO("visualizing plan 5 (pose goal move around box) %s", success?"":"FAILED");
	sleep(10.0);


	//attach the collision object to the robot
	ROS_INFO("attach the object to the robot");
	group.attachObject(collision_object.id);
	sleep(4.0);

	//detach the collision object
	ROS_INFO("detach the object from the robot");
	group.detachObject(collision_object.id);
	sleep(4.0);

	//remove the collision object from the world
	ROS_INFO("remove the object from the world");
	std::vector<std::string> object_ids;
	object_ids.push_back(collision_object.id);
	planning_scene_interface.removeCollisionObjects(object_ids);
	//in this way you can remove part of the objects in the world by using the ids?
	sleep(4.0);

	ros::shutdown();
	return 0;



}