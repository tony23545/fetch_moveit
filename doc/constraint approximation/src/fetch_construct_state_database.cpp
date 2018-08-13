#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/profiler/profiler.h>

#include <boost/math/constants/constants.hpp>

static const std::string ROBOT_DESCRIPTION = "robot_description";

moveit_msgs::Constraints getConstraints(){
	moveit_msgs::OrientationConstraint ocm;
	ocm.link_name = "wrist_roll_link";
	ocm.header.frame_id = "base_link";
	ocm.orientation.x = 0;
	ocm.orientation.y = 0;
  	ocm.orientation.z = 0;
  	ocm.orientation.w = 1.0;
  	ocm.absolute_x_axis_tolerance = 0.1;
  	ocm.absolute_y_axis_tolerance = 0.1;
  	ocm.absolute_z_axis_tolerance = boost::math::constants::pi<double>();
  	ocm.weight = 1.0;
  	moveit_msgs::Constraints cmsg;
  	cmsg.orientation_constraints.resize(1, ocm);
  	cmsg.name = ocm.link_name + ":upright";
  	return cmsg;
}

void computeDB(const robot_model::RobotModelPtr& robot_model, unsigned int ns, unsigned int ne)
{
  planning_scene::PlanningScenePtr ps(new planning_scene::PlanningScene(robot_model));
  ompl_interface::OMPLInterface ompl_interface(robot_model);
  moveit_msgs::Constraints c = getConstraints();
  ompl_interface::ConstraintApproximationConstructionOptions opt;
  opt.state_space_parameterization = "PoseModel";
  opt.samples = ns;
  opt.edges_per_sample = ne;
  opt.explicit_motions = true;
  opt.max_edge_length = 0.2;
  opt.explicit_points_resolution = 0.05;
  opt.max_explicit_points = 10;

  ompl_interface.getConstraintsLibrary().addConstraintApproximation(c, "arm", ps, opt);
  ompl_interface.getConstraintsLibrary().saveConstraintApproximations("constraint_database/constraints_approximation_database");
  ROS_INFO("Done");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "construct_ompl_state_database", ros::init_options::AnonymousName);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  unsigned int nstates = 10000;
  unsigned int nedges = 0;

  if (argc > 1)
    try
    {
      nstates = boost::lexical_cast<unsigned int>(argv[1]);
    }
    catch (...)
    {
    }

  if (argc > 2)
    try
    {
      nedges = boost::lexical_cast<unsigned int>(argv[2]);
    }
    catch (...)
    {
    }

  robot_model_loader::RobotModelLoader rml(ROBOT_DESCRIPTION);
  computeDB(rml.getModel(), nstates, nedges);

  ros::shutdown();
  return 0;
}
