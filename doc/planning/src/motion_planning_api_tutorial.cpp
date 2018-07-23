#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

#include <boost/scoped_ptr.hpp>

int main(int argc, char **argv){
	ros::init (argc, argv, "move_group_tutorial");
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::NodeHandle node_handle("~");

	//planner are setup as plugins in moveit, use pluginlib interface to load it
	//need two object, a RobotModel and a PlanningScene

	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

	//planning_scece, maintain the state of the world
	planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

	//construct a loader to load a planner, by name
	boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlanningManager>> planner_plugin_loader;
	planning_interface::PlanningManagerPtr planner_instance;
	std::string planner_plugin_name;

	//get the name of the planner from the ROS param server and then laod the planner
	if(!node_handle.getParam("planning_plugin", planner_plugin_name))
		ROS_FATAL_STREAM("could not find planner plugin name");
	try{
		planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
	}
	catch(pluginlib::PluginlibException& ex){
		ROS_FATAL_STREAM("Exception while creating planning plugin loader" << ex.what());
	}
	try{
		planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
		if(!planner_instance->initialize(robot_model, node_handle.getNamespace()))
			ROS_FATAL_STREAM("Could not initialize planner instance");
		ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
	}
	catch(pluginlib::PluginlibException& ex){
		const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
		std::stringstream ss;
		for(std::size_t i = 0; i < classes.size(); ++i)
			ss << classes[i] << " ";
		ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "':" << ex.what() <<" Available plugins: " << ss.str());
	}

	ros::WallDuration sleep_time(15.0);
	sleep_time.sleep();


	//Pose goal
	planning_interface::MotionPlanRequest req;
	planning_interface::MotionPlanResponse res;
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = "base_link";
	pose.pose.position.x = 0.75;
	pose.pose.position.y = 0.0;
	pose.pose.position.z = 0.0;
	pose.pose.orientation.w = 1.0;

	//specified in position and orientation
	std::vector<double> tolerance_pose(3, 0.01);
	std::vector<double> tolerance_angle(3, 0.01);

	//create the request as a constraint using a helper function
	//from kinematic_constraints package

	req.group_name = "arm";
	moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("wrist_roll_link", pose, tolerance_pose, tolerance_angle);
	req.goal_constraints.push_back(pose_goal);

	//construct a planning contex
	planning_interface::PlanningContexPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
	context->solve(res);
	if(res.error_code_.val != res.error_code_.SUCCESS){
		ROS_ERROR("Could not compute plan successfully");
		return 0;
	}

	//visualize the result
	ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	moveit_msgs::DisplayTrajectory display_trajectory;

	//visualize the trajectory
	ROS_INFO("Visualizing the trajectory");
	moveit_msgs::MotionPlanResponse response;
	res.getMessage(response);

	display_trajectory.trajectory_start = response.trajectory_start;
	display_trajectory.trajectory.push_back(response.trajectory);
	display_publisher.publish(display_trajectory);

	sleep_time.sleep();


	//joint space goal
	//first set the state in the planning state to the final state of the last plan
	robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();
	planning_scene->setCurrentState(response.trajectory_start);
	const robot_state::JointModelGroup* joint_model_group = robot_state.getJointModelGroup("arm");
	robot_state.setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);

	//now, set a joint space goal
	robot_state::RobotState goal_state(robot_model);
	std::vector<double> joint_values(7, 0.0);
	joint_values[0] = -2.0;
	joint_values[3] = -0.2;
	joint_values[5] = -0.15;
	goal_state.setJointGroupPositions(joint_model_group, joint_values);
	moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_mode_group);
	req.goal_constraints.clear();
	req.goal_constraints.push_back(joint_goal);

	//call the planner and visualizing the trajectory
	context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
	context->solve(res);
	if(res.error_code_.val != res.error_code_.SUCCESS){
		ROS_ERROR("Could not compute plan successfully");
		return 0;
	}

	//visualize the trajectory
	ROS_INFO("Visualizing the trajectory");
	res.getMessage(response);
	display_trajectory.trajectory_start = response.trajectory_start;
	display_trajectory.trajectory.push_back(response.trajectory);

	//now two planned trajectory in series
	display_publisher.publish(display_trajectory);

	//add one more goals
	robot_state.setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);

	req.goal_constraints.clear();
	req.goal_constraints.push_back(pose_goal);
	context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
	context->solve(res);
	res.getMessage(response);
	display_trajectory.trajectory.push_back(response.trajectory);
	display_trajectory.publish(display_trajectory);

	//add path constraints
	pose.pose.position.x = 0.65;
	pose.pose.position.y = -0.2;
	pose.pose.position.z = -0.1;
	moveit_msgs::Constraints pose_goal_2 = kinematic_constraints::constructGoalConstraints("wrist_roll_link", pose, tolerance_pose, tolerance_angle);
	//first set the state in the planning scene to the final state of the last plan
	robot_state.setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
	//try to move to this new pose goal
	req.goal_constraints.clear();
	req.goal_constraints.push_back(pose_goal_2);

	//impose a path constraints

 


}