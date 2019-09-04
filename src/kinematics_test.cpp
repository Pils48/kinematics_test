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

#define FANUC_M20IA_END_EFFECTOR "tool0"
#define DEFAULT_ROBOT_DESCRIPTION "robot_description"
#define PLANNING_GROUP "manipulator"

using namespace std;
using namespace moveit;
using namespace core;

static const double CONTINUITY_CHECK_THRESHOLD = M_PI;
static const double ALLOWED_COLLISION_DEPTH = 0.0000001;
static const double STANDARD_INTERPOLATION_STEP = 0.01;
static const double EXPERIMENTAL_DISTANCE_CONSTRAINT = 0.005;

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
		else {
			ROS_ERROR("Impossible to create whole path! Check self-collision or limits excess.");
			trail.clear();
			return false;
		}
		
	}
	
	return true;
}

double getFullTranslation(const robot_state::RobotStatePtr state, const robot_state::RobotStatePtr next_state,
			string link_name){
	auto link_mesh_ptr = state->getLinkModel(link_name)->getShapes()[0].get();
	Eigen::Vector3d link_extends = shapes::computeShapeExtents(link_mesh_ptr);
	
	const Eigen::Affine3d state_transform = state->getGlobalLinkTransform(link_name);
	const Eigen::Affine3d next_state_transform = next_state->getGlobalLinkTransform(link_name);
	Eigen::Quaterniond start_quaternion(state_transform.rotation());
	Eigen::Quaterniond target_quaternion(next_state_transform.rotation());
	
	//Чекать все точки бокса на удаленность от ориджина и смотреть октанты
	double sin_between_quaternions = sin(start_quaternion.angularDistance(target_quaternion));
	double diagonal_length = sqrt(pow(link_extends[0], 2) + pow(link_extends[1], 2) + pow(link_extends[2], 2));
	
	double linear_angular_distance = (state_transform.translation().norm() + diagonal_length) * sin_between_quaternions;
	
	return (state_transform.translation() - next_state_transform.translation()).norm() + linear_angular_distance;
}

pair<string, double> getMaxTranslation(const robot_state::RobotStatePtr state,
		const robot_state::RobotStatePtr next_state){
	
	pair<string, double> max_pair = make_pair("", 0);
	for (auto link : state->getJointModelGroup(PLANNING_GROUP)->getUpdatedLinkModelsWithGeometry()){
		if (getFullTranslation(state, next_state, link->getName()) >= max_pair.second){
			max_pair.first = link->getName();
			max_pair.second = getFullTranslation(state, next_state, link->getName());
		}
		
	}
	
	return max_pair;
	
}

void splitTrajectorySegment(list<robot_state::RobotStatePtr>& trail, double critical_distance,
		planning_scene::PlanningScenePtr current_scene){
	
	for (auto state_it = trail.begin(); state_it != --trail.end(); ++state_it){
		
		pair<string, double> translation_pair = getMaxTranslation(*state_it, *next(state_it));
		
		while (translation_pair.second > critical_distance){
			ROS_WARN("%s has to great translation: %f", translation_pair.first.c_str(),
					translation_pair.second);
			list<robot_state::RobotStatePtr> segment_to_check;
			bool is_interpolated = linearInterpolation(segment_to_check, **state_it,
					(*next(state_it))->getGlobalLinkTransform(FANUC_M20IA_END_EFFECTOR), 1);
			if (is_interpolated){
				auto insert_it = inserter(trail, next(state_it));
				insert_it = *next(segment_to_check.begin());
				translation_pair.second = getFullTranslation(*state_it, *next(state_it), translation_pair.first);
			}
			else
				throw runtime_error("Space jump happened!\nInvalid trajectory!");
		}
		ROS_INFO("%s translate : %f", translation_pair.first.c_str(), translation_pair.second);
	}
}

void checkCollisionAllowed(robot_state::RobotState& state, planning_scene::PlanningScenePtr current_scene){
	
	collision_detection::CollisionRequest req;
	collision_detection::CollisionResult res;
	current_scene->checkCollision(req, res, state);
	auto contact_map = res.contacts;
	
	if (res.collision == false && contact_map.empty())
		return;
	auto contact_vector = contact_map.begin()->second;
	
	//Here we are sure that in initial and final state we dont have more than one contact
	if ((contact_map.size() == 1) && (contact_vector.size() == 1)) {
			double contact_depth = contact_vector[0].depth;
			if (contact_depth > ALLOWED_COLLISION_DEPTH)
				throw runtime_error("Collision during the trajectory processing!\nInvalid trajectory!");
	}
	else
		throw runtime_error("Collision during the trajectory processing!\nInvalid trajectory!");
	
}

void check_jump(list<robot_state::RobotStatePtr> trajectory, bool f){
	for (string link_name : trajectory.front()->getJointModelGroup(PLANNING_GROUP)->getLinkModelNamesWithCollisionGeometry()){
		for (auto state_it = trajectory.begin(); state_it != prev(trajectory.end()); ++state_it){
			
			auto left_state = **state_it;
			auto right_state = **next(state_it);
			const Eigen::Affine3d state_transform = (*state_it)->getGlobalLinkTransform(link_name);
			const Eigen::Affine3d next_state_transform = (*next(state_it))->getGlobalLinkTransform(link_name);
			double left = state_transform.translation().norm();
			double right = next_state_transform.translation().norm();
			double dist = (*state_it)->distance(**next(state_it));
			while (right - left > EXPERIMENTAL_DISTANCE_CONSTRAINT){
				auto mid_state = left_state;
				double mid = (left + right) / 2;
				list<robot_state::RobotStatePtr> tmp_vector;
				bool is_interpolated = linearInterpolation(tmp_vector, **state_it, next_state_transform, 1);
				double lm = left_state.distance(mid_state);
				double mr = mid_state.distance(right_state);
				if (lm > mr)
				{
					dist = lm;
					right = mid;
					right_state = mid_state;
				}
				else
				{
					dist = mr;
					left = mid;
					left_state = mid_state;
				}
			}
			
			if (dist / EXPERIMENTAL_DISTANCE_CONSTRAINT < CONTINUITY_CHECK_THRESHOLD)
				throw runtime_error("Space jump happened!\nInvalid Trajectory!");
			
		}
	}
}

void check_collision(list<robot_state::RobotStatePtr> trajectory,
		planning_scene::PlanningScenePtr current_scene){
	for (robot_state::RobotStatePtr state : trajectory){
		if (current_scene->isStateColliding(*state, PLANNING_GROUP, true)){
			throw runtime_error("Collision during the trajectory processing!\nInvalid trajectory!");
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
	
	moveit_visual_tools::MoveItVisualTools visual_tools(kt_kinematic_model->getModelFrame());
	namespace rvt = rviz_visual_tools;
	visual_tools.deleteAllMarkers();
	visual_tools.loadRemoteControl();
	
	Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
	text_pose.translation().z() = 1.75;
	visual_tools.publishText(text_pose, "Kinematic_test demo", rvt::WHITE, rvt::XLARGE);
	visual_tools.trigger();
	//end of initialization
	
	tf::Transform tf_start(tf::createQuaternionFromRPY(0, M_PI_2, 0), tf::Vector3(1.085, 0.1, 1.565));
	tf::Transform tf_goal(tf::createQuaternionFromRPY(0, M_PI_2, 0), tf::Vector3(1.085, -0.1, 1.565));
	const Eigen::Affine3d end_effector_frame = kt_kinematic_state.getGlobalLinkTransform(FANUC_M20IA_END_EFFECTOR);
	Eigen::Affine3d goal_transform(Eigen::Translation3d(1.085, 0.1, 1.565));
	Eigen::Affine3d start_transform(Eigen::Translation3d(1.085, -0.1, 1.565));
	
	tf::transformTFToEigen(tf_start, start_transform);
	tf::transformTFToEigen(tf_goal, goal_transform);
	//Check first and last state on allowed collision
//	kt_kinematic_state.setFromIK(joint_model_group_ptr, goal_transform);
//	checkCollisionAllowed(kt_kinematic_state, kt_planning_scene);
//	kt_kinematic_state.setFromIK(joint_model_group_ptr, start_transform);
//	checkCollisionAllowed(kt_kinematic_state, kt_planning_scene);
	kt_kinematic_state.setFromIK(joint_model_group_ptr, start_transform);
	visual_tools.publishRobotState(kt_kinematic_state, rvt::BLUE);
	
	list<robot_state::RobotStatePtr> trajectory;
	size_t approximate_steps = floor((goal_transform.translation() - start_transform.translation()).norm() /
			STANDARD_INTERPOLATION_STEP);
	bool is_interpolated = linearInterpolation(trajectory, kt_kinematic_state, goal_transform, approximate_steps);
	
	if (is_interpolated){
		thread check_collision_thread(check_collision, trajectory, kt_planning_scene);
		thread check_jump_thread(check_jump, trajectory);
		splitTrajectorySegment(trajectory, EXPERIMENTAL_DISTANCE_CONSTRAINT, kt_planning_scene);
		check_collision_thread.join();
		check_jump_thread.join();
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
	for (auto it = trajectory.begin(); it != trajectory.end(); ++it){
		this_thread::sleep_for(chrono::milliseconds(10));
		visual_tools.publishRobotState(*it);
		this_thread::sleep_for(chrono::milliseconds(10));
		visual_tools.deleteAllMarkers();
	}
}