/*********************************************************************
 * Kinematics test for linear planner with collision checking
 * using trac-ik as a kinematic solver.
 *********************************************************************/


#include <ros/ros.h>

#include <vector>
#include <list>
#include <chrono>
#include <thread>

#include <geometric_shapes/shape_operations.h>
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>
#include <tf_conversions/tf_eigen.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#define STANDARD_INTERPOLATION_STEP 0.01
#define EXPERIMENTAL_DISTANCE_CONSTRAINT 0.005
#define FANUC_M20IA_END_EFFECTOR "tool0"
#define DEFAULT_ROBOT_DESCRIPTION "robot_description"
#define PLANNING_GROUP "manipulator"

using namespace std;
using namespace moveit;
using namespace core;

/** Interpolate trajectory using slerp quaternion algorithm and linear algorithms
 * for translation parameter. Return true in case of success. Trail assumed to be empty*/
bool linearInterpolation(list<robot_state::RobotStatePtr>& trail,
		robot_state::RobotState kinematic_state, const Eigen::Affine3d& goal_transform, size_t translation_steps,
			bool global_reference_frame = true){
	auto inserter = back_inserter(trail);
	const robot_state::JointModelGroup* jmg_ptr = kinematic_state.getJointModelGroup(PLANNING_GROUP);
	inserter = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_state));
	const moveit::core::LinkModel* ptr_link_model = kinematic_state.getLinkModel(FANUC_M20IA_END_EFFECTOR);
	
	Eigen::Affine3d start_pose = kinematic_state.getGlobalLinkTransform(ptr_link_model);
	
	// the target can be in the local reference frame (in which case we rotate it)
	Eigen::Affine3d rotated_target = global_reference_frame ? goal_transform : start_pose * goal_transform;
	
	Eigen::Quaterniond start_quaternion(start_pose.rotation());
	Eigen::Quaterniond target_quaternion(rotated_target.rotation());
	
	size_t steps = translation_steps + 1;
	
	for (size_t i = 1; i <= steps; ++i)
	{
		double percentage = (double)i / (double)steps;
		
		Eigen::Affine3d pose(start_quaternion.slerp(percentage, target_quaternion));
		
		pose.translation() = percentage * rotated_target.translation() + (1 - percentage) * start_pose.translation();
		
		if (kinematic_state.setFromIK(jmg_ptr, pose, ptr_link_model->getName()))
			inserter = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_state));
		else{
			ROS_ERROR("Impossible to create whole path! Check self-collision or limits excess.");
			trail.clear();
			return false;
		}
		
	}
	
	return true;
}

double getFullTranslation(const robot_state::RobotStatePtr state, const robot_state::RobotStatePtr next_state,
		Eigen::Vector3d& link_extends, string link_name){
	
	const Eigen::Affine3d state_transform = state->getGlobalLinkTransform(link_name);
	const Eigen::Affine3d next_state_transform = next_state->getGlobalLinkTransform(link_name);
	Eigen::Quaterniond start_quaternion(state_transform.rotation());
	Eigen::Quaterniond target_quaternion(next_state_transform.rotation());
	
	//Чекать все точки бокса на удаленность от ориджина и смотреть октанты
	double sin_between_quaternions = sin(start_quaternion.angularDistance(target_quaternion));
	double diagonal_length = sqrt(pow(link_extends[0], 2) + pow(link_extends[1], 2) + pow(link_extends[2], 2));
	
	//Translate origin on diagonal length
	double linear_angular_distance = (state_transform.translation().norm() + diagonal_length) * sin_between_quaternions;
	return (state_transform.translation() - next_state_transform.translation()).norm() + linear_angular_distance;
	
}

void findLinkDistance(list<robot_state::RobotStatePtr>& trail,
		const robot_state::LinkModel* link, double critical_distance, planning_scene::PlanningScenePtr current_scene){
	
	//Work with the greatest translation of the link
	//Get shape dimensions
	//Выбирать наибольшую диагональ у shape нам все равно какой столкнется
	//но кадждая линка пройдет свой путь и может оказаться что беря наибольшиймеш мы слишком
	//перестраховываемся
	const shapes::Shape* link_mesh_ptr = link->getShapes()[0].get();
	Eigen::Vector3d link_extends = shapes::computeShapeExtents(link_mesh_ptr);
	
	for (auto state_it = trail.begin(); state_it != --trail.end(); ++state_it){
		
		double translation_distance = getFullTranslation(*state_it, *next(state_it),
		                                                 link_extends, link->getName());
		
		while (translation_distance > critical_distance){
			ROS_WARN("%s has to great translation: %f", link->getName().c_str(), translation_distance);
			list<robot_state::RobotStatePtr> segment_to_check;
			bool is_interpolated = linearInterpolation(segment_to_check, **state_it,
					(*next(state_it))->getGlobalLinkTransform(FANUC_M20IA_END_EFFECTOR), 1);
			if (is_interpolated){
				auto insert_it = inserter(trail, next(state_it));
				insert_it = *next(segment_to_check.begin());
				translation_distance = getFullTranslation(*state_it, *next(state_it),
				                                          link_extends, link->getName());
			}
			else {
				ROS_ERROR("Space jump happened!");
				throw runtime_error("Invalid trajectory!");
			}
			
		}
		
		ROS_INFO("%s translate : %f", link->getName().c_str(), translation_distance);
	}
	
}

void check_collision(list<robot_state::RobotStatePtr> trajectory,
		planning_scene::PlanningScenePtr current_scene){
	for (robot_state::RobotStatePtr state : trajectory){
		if (current_scene->isStateColliding(*state, PLANNING_GROUP, true)){
			ROS_ERROR("Collision during the trajectory processing!");
			throw runtime_error("Invalid trajectory!");
		}
	}
}

int main(int argc, char** argv)
{
	//Initialization
	ros::init(argc, argv, "kinematics_test");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();
	
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	
	robot_model_loader::RobotModelLoader kt_robot_model_loader(DEFAULT_ROBOT_DESCRIPTION);
	robot_model::RobotModelConstPtr kt_kinematic_model = kt_robot_model_loader.getModel();
	planning_scene_monitor::PlanningSceneMonitor kt_planning_scene_monitor(DEFAULT_ROBOT_DESCRIPTION);
	planning_scene::PlanningScenePtr kt_planning_scene = kt_planning_scene_monitor.getPlanningScene();
	robot_state::RobotState kt_kinematic_state(kt_kinematic_model);
	ROS_INFO("Model frame: %s", kt_kinematic_model->getModelFrame().c_str());
	
	kt_kinematic_state.setToDefaultValues();
	const robot_state::JointModelGroup* joint_model_group_ptr = kt_kinematic_model->getJointModelGroup(PLANNING_GROUP);
	
	moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
	namespace rvt = rviz_visual_tools;
	visual_tools.deleteAllMarkers();
	visual_tools.loadRemoteControl();
	
	Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
	text_pose.translation().z() = 1.75;
	visual_tools.publishText(text_pose, "Kinematic_test demo", rvt::WHITE, rvt::XLARGE);
	visual_tools.trigger();
	//end of initialization
	
	tf::Transform tf_start(tf::createQuaternionFromRPY(M_PI_2, M_PI_2, 0), tf::Vector3(1.085, 0.1, 1.565));
	tf::Transform tf_goal(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(1.085, -0.1, 1.565));
	const Eigen::Affine3d end_effector_frame = kt_kinematic_state.getGlobalLinkTransform(FANUC_M20IA_END_EFFECTOR);
	Eigen::Affine3d goal_transform(Eigen::Translation3d(1.085, 0.1, 1.565));
	Eigen::Affine3d start_transform(Eigen::Translation3d(1.085, -0.1, 1.565));
	
	tf::transformTFToEigen(tf_start, start_transform);
	tf::transformTFToEigen(tf_goal, goal_transform);
	
	kt_kinematic_state.setFromIK(joint_model_group_ptr,  start_transform);
	visual_tools.publishRobotState(kt_kinematic_state, rvt::BLUE);
	
	list<robot_state::RobotStatePtr> trajectory(0);
	size_t approximate_steps = floor((goal_transform.translation() - start_transform.translation()).norm() /
			STANDARD_INTERPOLATION_STEP);
	bool is_interpolated = linearInterpolation(trajectory, kt_kinematic_state, goal_transform, approximate_steps);
	
	if (is_interpolated){
		for (string link_name : joint_model_group_ptr->getLinkModelNamesWithCollisionGeometry()){
			thread check_collision_thread(check_collision, trajectory, kt_planning_scene);
			findLinkDistance(trajectory, kt_kinematic_state.getLinkModel(link_name), EXPERIMENTAL_DISTANCE_CONSTRAINT, kt_planning_scene);
			check_collision_thread.join();
		}
	}
	
	//Construct and publish trajectory line
	vector<geometry_msgs::Pose> waypoints;
	for (robot_state::RobotStatePtr state : trajectory){
		Eigen::Affine3d pose = state->getGlobalLinkTransform(FANUC_M20IA_END_EFFECTOR);
		waypoints.push_back(tf2::toMsg(pose));
	}
	visual_tools.publishPath(waypoints, rvt::GREEN, rvt::SMALL);
	visual_tools.trigger();
	
	//Visualize trajectory
	for (list<robot_state::RobotStatePtr>::iterator it = trajectory.begin(); it != trajectory.end(); ++it){
		this_thread::sleep_for(chrono::milliseconds(10));
		visual_tools.publishRobotState(*it);
		this_thread::sleep_for(chrono::milliseconds(10));
		visual_tools.deleteAllMarkers();
	}
}