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
	ros::init(argc, argv, "move_group_tutorial");
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::NodeHandle node_handle("~");

	//planner can be loaded using pluginlib interface
	//need two objects, a RobotModel, a PlanningScene

	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr robot_model = robot_model.getModel();

	//construct a planning_scene that maintain the state of the world
	planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

	//construct a loader to load a planner
	//note that we are using the ROS pluginlib here
	boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
	planning_interface::PlannerManagerPtr planner_instance;
	std::string planner_plugin_name;

	//get the name of planning plugin we want to load from the ROS param server
	//then load the planner making sure to catch all the exceptions
	

}