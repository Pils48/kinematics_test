/*********************************************************************
 * In this file we will test a kinematics algorithm trac-ik by
 * solving forward and inverse kinematic
 *********************************************************************/

#include <ros/ros.h>

#include <vector>
#include <chrono>
#include <thread>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <Eigen/Geometry>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#define STANDARD_INTERPOLATION_STEP 0.01
#define EXPERIMENTAL_DISTANCE_CONSTRAINT 0.005

typedef bool (*validateFnc)(robot_state::RobotStatePtr, robot_state::RobotStatePtr, double, double);

using namespace std;
using namespace moveit;
using namespace core;

/** Interpolate trajectory using slerp quaternion algorithm and linear algorithms
 * for translation parameter. Return true in case of success.*/
bool linearInterpolation(vector<robot_state::RobotStatePtr>& trail,
                         robot_state::RobotState kinematic_state, const Eigen::Affine3d& goal_transform,
                         size_t translation_steps, bool global_reference_frame = true){
	
	const robot_state::JointModelGroup* jmg = kinematic_state.getJointModelGroup("manipulator");
	vector<robot_state::RobotStatePtr> trail_copy(trail);
	trail_copy.push_back(robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_state)));
	const moveit::core::LinkModel* ptr_link_model = kinematic_state.getLinkModel("tool0");
	// effector
	
	Eigen::Affine3d start_pose = kinematic_state.getGlobalLinkTransform(ptr_link_model);
	
	// the target can be in the local reference frame (in which case we rotate it)
	Eigen::Affine3d rotated_target = global_reference_frame ? goal_transform : start_pose * goal_transform;
	
	Eigen::Quaterniond start_quaternion(start_pose.rotation());
	Eigen::Quaterniond target_quaternion(rotated_target.rotation());
	double rotation_distance = target_quaternion.angularDistance(start_quaternion);
	double translation_distance = (rotated_target.translation() - start_pose.translation()).norm();
	
	size_t steps = translation_steps + 1;
	
	for (size_t i = 1; i <= steps; ++i)
	{
		double percentage = (double)i / (double)steps;
		
		Eigen::Affine3d pose(start_quaternion.slerp(percentage, target_quaternion));
		
		pose.translation() = percentage * rotated_target.translation() + (1 - percentage) * start_pose.translation();
		
		if (kinematic_state.setFromIK(jmg, pose, ptr_link_model->getName()))
			trail_copy.push_back(robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_state)));
		else{
			ROS_ERROR("Impossible to create whole path! Check self-collision or limits excess.");
			return false;
		}
		
	}
	trail = trail_copy;
	return true;
}

double getFullTranslation(const robot_state::RobotStatePtr state, const robot_state::RobotStatePtr next_state,
                          Eigen::Vector3d& link_extends, string link_name){
	
	const Eigen::Affine3d state_transform = state->getGlobalLinkTransform(link_name);
	const Eigen::Affine3d next_state_transform = next_state->getGlobalLinkTransform(link_name);
	Eigen::Quaterniond start_quaternion(state_transform.rotation());
	Eigen::Quaterniond target_quaternion(next_state_transform.rotation());
	
	//Find sin of angle between start_quaternion and target_quaternion
	double sin_between_quaternions = 1 - pow(start_quaternion.dot(target_quaternion) / (start_quaternion.norm() *
	                                                                                    target_quaternion.norm()), 2) ;
	//Find which otctant
	int x_coeff, y_coeff, z_coeff;
	x_coeff = state_transform.translation()(0) > 0 ? 1 : -1;
	y_coeff = state_transform.translation()(1) > 0 ? 1 : -1;
	z_coeff = state_transform.translation()(2) > 0 ? 1 : -1;
	
	const Eigen::Translation3d origin_to_diagonal(x_coeff* link_extends[0], y_coeff * link_extends[1],z_coeff * link_extends[2]);
	//Translate origin on diagonal length
	double linear_angular_distance = (state_transform.translation() + origin_to_diagonal.translation()).norm() *
	                                 sin_between_quaternions;
	return (state_transform.translation() - next_state_transform.translation()).norm() + linear_angular_distance;
	
}

pair<string, vector<double> > findLinkDistance(vector<robot_state::RobotStatePtr>& trail,
		const robot_state::LinkModel* link, double critical_distance, size_t jump_state_idx = 0){
	
	pair<string, vector<double> > result_pair;
	
	result_pair.first = link->getName();
	vector<double> distances(0);
	result_pair.second = distances;
	
	//Work with the greatest translation of the link
	//Get shape dimensions
	auto link_mesh = link->getShapes()[0].get(); //Refactor
	Eigen::Vector3d link_extends = shapes::computeShapeExtents(link_mesh);
	
	for (size_t state_idx = 0; state_idx < trail.size() - 1; state_idx++){
		
		double translation_distance = getFullTranslation(trail[state_idx], trail[state_idx + 1],
		                                                 link_extends, link->getName());
		
		while (translation_distance > critical_distance){
			ROS_WARN("%s has to great translation: %f", link->getName().c_str(), translation_distance);
			vector<robot_state::RobotStatePtr> segment_to_check(0);
			bool is_interpolated = linearInterpolation(segment_to_check, *trail[state_idx], trail[state_idx + 1]
					->getGlobalLinkTransform("tool0"), 1);
			if (is_interpolated){
				trail.insert(trail.begin() + state_idx + 1, segment_to_check[1]);
				translation_distance = getFullTranslation(trail[state_idx], trail[state_idx + 1],
				                                          link_extends, link->getName());
			}
			else{
				ROS_ERROR("Space jump happened! Truncate trajectory after jump!");
				throw Exception("Invalid trajectory!");
			}
			
		}
		result_pair.second.push_back(translation_distance);
		ROS_INFO("%s translate : %f", link->getName().c_str(), translation_distance);
	}
	
	return result_pair;
}

int main(int argc, char** argv)
{
	//Initialization
	ros::init(argc, argv, "kinematics_test");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();
	
	string PLANNING_GROUP = "manipulator";
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	
	robot_model_loader::RobotModelLoader kt_robot_model_loader("robot_description");
	robot_model::RobotModelConstPtr kinematic_model = kt_robot_model_loader.getModel();
	planning_scene_monitor::PlanningSceneMonitor kt_planning_scene_monitor("robot_description");
	planning_scene::PlanningScenePtr kt_planning_scene = kt_planning_scene_monitor.getPlanningScene();
	robot_state::RobotState kinematic_state(kinematic_model);
	ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
	
	kinematic_state.setToDefaultValues();
	const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
	
	moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
	namespace rvt = rviz_visual_tools;
	visual_tools.deleteAllMarkers();
	visual_tools.loadRemoteControl();
	
	Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
	text_pose.translation().z() = 1.75;
	visual_tools.publishText(text_pose, " Demo", rvt::WHITE, rvt::XLARGE);
	visual_tools.trigger();

//	visual_tools.prompt("Press next to continue execution...");
	//end of initialization
	
	/** Test trac-ik
	kinematic_state.setToRandomPositions(joint_model_group);
	const Eigen::Affine3d& end_effector_state = kinematic_state.getGlobalLinkTransform("tool0");

	//Visualization
	visual_tools.publishRobotState(kinematic_state, rvt::colors::DARK_GREY);
	visual_tools.prompt("Press next to continue execution...");

	vector<double> joint_values;
	double timeout = 0.1;
	bool found_ik = kinematic_state.setFromIK(joint_model_group, end_effector_state, timeout);

	if(found_ik){
		visual_tools.publishRobotState(kinematic_state, rvt::colors::BLUE);
	}
	visual_tools.prompt("Press next to continue execution...");

	kinematic_state.setToDefaultValues(); */
	
	const Eigen::Affine3d tool0_frame = kinematic_state.getGlobalLinkTransform("tool0");
	
	Eigen::Affine3d goal_transform(Eigen::Translation3d(-1, 0.7, 1.5)); //sx -1 sy 1 sz 1,5
	const Eigen::Affine3d start_pose(Eigen::Translation3d(-1, 0.0, -0.3));
	kinematic_state.setFromIK(joint_model_group, start_pose);
	visual_tools.publishRobotState(kinematic_state, rvt::BLUE);
	
	vector<robot_state::RobotStatePtr> traj(0);
	size_t approximate_steps = floor((goal_transform.translation() - start_pose.translation()).norm() /
	                                 STANDARD_INTERPOLATION_STEP);
	bool is_interpolated = linearInterpolation(traj, kinematic_state, goal_transform, approximate_steps);
	
	map<string, vector<double> > links_distances;
	if (is_interpolated){
		for (size_t link_idx = 1; link_idx <= 6; link_idx++){
			string link_name = string("link_") + to_string(link_idx);
			pair<string, vector<double> > link_distance_pair = findLinkDistance(traj, kinematic_state.getLinkModel(link_name),
					EXPERIMENTAL_DISTANCE_CONSTRAINT);
			links_distances.insert(links_distances.end(), link_distance_pair);
		}
	}
	
	
	//Visualize trajectory
	for (size_t i = 0; i < traj.size(); i++){
		this_thread::sleep_for(chrono::milliseconds(20));
		visual_tools.publishRobotState(traj[i]);
		this_thread::sleep_for(chrono::milliseconds(20));
		visual_tools.deleteAllMarkers();
	}
	
}