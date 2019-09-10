/*********************************************************************
 * Kinematics test for linear planner with collision checking
 * using trac-ik as a kinematic solver.
 *********************************************************************/


#include <ros/ros.h>

#include <list>
#include <chrono>
#include <thread>

#include <geometric_shapes/shape_operations.h>
#include <Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#define FANUC_M20IA_END_EFFECTOR "tool0"
#define DEFAULT_ROBOT_DESCRIPTION "robot_description"
#define PLANNING_GROUP "manipulator"

using namespace std;
using namespace moveit;
using namespace core;
using namespace robot_state;
using namespace Eigen;

static constexpr double CONTINUITY_CHECK_THRESHOLD = M_PI * 0.0001;
static const double ALLOWED_COLLISION_DEPTH = 0.0000001;
static const double DISTANCE_TOLERANCE = 0.00015;
static const double STANDARD_INTERPOLATION_STEP = 0.01;
static const double EXPERIMENTAL_DISTANCE_CONSTRAINT = 0.005;


bool validateIK(RobotState* robot_state, const JointModelGroup* joint_group,
        const double* joint_group_variable_values)
{
    return robot_state->satisfiesBounds(joint_group);
}

/** Interpolate trajectory using slerp quaternion algorithm and linear algorithms
 * for translation parameter. Return true in case of success. Trail assumed to be empty*/
bool linearInterpolation(list<RobotStatePtr>& trail, RobotState kinematic_state, const Affine3d& goal_transform,
        size_t translation_steps, bool global_reference_frame = true)
{
    auto *jmg_ptr = kinematic_state.getJointModelGroup(PLANNING_GROUP);
    auto inserter = back_inserter(trail);
    inserter = RobotStatePtr(new RobotState(kinematic_state));
    auto *ptr_link_model = kinematic_state.getLinkModel(FANUC_M20IA_END_EFFECTOR);

    Affine3d start_pose = kinematic_state.getGlobalLinkTransform(ptr_link_model);

    // the target can be in the local reference frame (in which case we rotate it)
    Affine3d rotated_target = global_reference_frame ? goal_transform : start_pose * goal_transform;

    Quaterniond start_quaternion(start_pose.rotation());
    Quaterniond target_quaternion(rotated_target.rotation());

    size_t steps = translation_steps + 1;

    for (size_t i = 1; i <= steps; ++i) {
        double percentage = (double) i / (double) steps;

        Affine3d pose(start_quaternion.slerp(percentage, target_quaternion));

        pose.translation() = percentage * rotated_target.translation() + (1 - percentage) * start_pose.translation();

        if (kinematic_state.setFromIK(jmg_ptr, pose, ptr_link_model->getName(),
                validateIK(&kinematic_state, jmg_ptr, kinematic_state.getVariablePositions())))
            inserter = RobotStatePtr(new RobotState(kinematic_state));
        else {
            ROS_ERROR("Impossible to create whole path! Check self-collision or limits excess.");
            trail.clear();
            return false;
        }
    }
    return true;
}

RobotStatePtr getMiddleState(RobotState state, RobotState next_state)
{
    list<RobotStatePtr> segment_to_check;
    auto is_interpolated = linearInterpolation(segment_to_check, state, next_state.getGlobalLinkTransform(FANUC_M20IA_END_EFFECTOR), 1);
    return is_interpolated ? *next(segment_to_check.begin()) : nullptr;
}

double getFullTranslation(const RobotStatePtr state, const RobotStatePtr next_state, string link_name)
{
	auto link_mesh_ptr = state->getLinkModel(link_name)->getShapes()[0].get();
	Vector3d link_extends = shapes::computeShapeExtents(link_mesh_ptr);

	const Affine3d state_transform = state->getGlobalLinkTransform(link_name);
	const Affine3d next_state_transform = next_state->getGlobalLinkTransform(link_name);
	Quaterniond start_quaternion(state_transform.rotation());
	Quaterniond target_quaternion(next_state_transform.rotation());

	double sin_between_quaternions = sin(start_quaternion.angularDistance(target_quaternion));
	double diagonal_length = sqrt(pow(link_extends[0], 2) + pow(link_extends[1], 2) + pow(link_extends[2], 2));

	double linear_angular_distance = (state_transform.translation().norm() + diagonal_length) * sin_between_quaternions;

	return (state_transform.translation() - next_state_transform.translation()).norm() + linear_angular_distance;
}

pair<string, double> getMaxTranslation(const RobotStatePtr state, const RobotStatePtr next_state)
{
	pair<string, double> max_pair = make_pair("", 0);
	for (auto link : state->getJointModelGroup(PLANNING_GROUP)->getUpdatedLinkModelsWithGeometry()){
		if (getFullTranslation(state, next_state, link->getName()) >= max_pair.second){
			max_pair.first = link->getName();
			max_pair.second = getFullTranslation(state, next_state, link->getName());
		}
	}
	return max_pair;
}

void checkAllowedCollision(RobotState& state, planning_scene::PlanningScenePtr current_scene)
{
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

void checkJump(list<RobotStatePtr> trajectory)
{
    for (auto state_it = trajectory.begin(); state_it != prev(trajectory.end()); ++state_it){
        auto right = *next(state_it);
        auto left = *state_it;
        auto dist = right->distance(*left);

        while (dist > CONTINUITY_CHECK_THRESHOLD){
            auto mid = getMiddleState(*right, *left);
            if (!mid)
                throw runtime_error("Space jump happened!\nInvalid trajectory!");
            dist /= 2;
            auto lm = left->distance(*mid);
            auto mr = mid->distance(*right);
            (lm > mr) ? right = mid : left = mid;
        }
        if (right->distance(*left) >= CONTINUITY_CHECK_THRESHOLD + DISTANCE_TOLERANCE)
            throw runtime_error("Space jump happened!\nInvalid trajectory!");
    }
}


void splitTrajectorySegment(list<RobotStatePtr>& trail, double critical_distance)
{
	for (auto state_it = trail.begin(); state_it != prev(trail.end()); ++state_it){
		auto translation_pair = getMaxTranslation(*state_it, *next(state_it));

		while (translation_pair.second > critical_distance) {
            auto mid_state = getMiddleState(**state_it, **next(state_it));
            if (mid_state) {
                auto insert_it = inserter(trail, next(state_it));
                insert_it = mid_state;
                translation_pair.second = getFullTranslation(*state_it, *next(state_it), translation_pair.first);
            } else
                throw runtime_error("Space jump happened!\nInvalid trajectory!");
        }
	}
}

void checkCollision(list<RobotStatePtr> trajectory, planning_scene::PlanningScenePtr current_scene)
{
	for (auto state_it = next(trajectory.begin()); state_it != prev(trajectory.end()); ++state_it){
		if (current_scene->isStateColliding(**state_it, PLANNING_GROUP, true))
			throw runtime_error("Collision during the trajectory processing!\nInvalid trajectory!");
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
	RobotState kt_kinematic_state(kt_kinematic_model);
	ROS_INFO("Model frame: %s", kt_kinematic_model->getModelFrame().c_str());

	kt_kinematic_state.setToDefaultValues();
	const JointModelGroup* joint_model_group_ptr = kt_kinematic_model->getJointModelGroup(PLANNING_GROUP);

	moveit_visual_tools::MoveItVisualTools visual_tools(kt_kinematic_model->getModelFrame());
	namespace rvt = rviz_visual_tools;
	visual_tools.deleteAllMarkers();
	visual_tools.loadRemoteControl();
	//end of initialization

	tf::Transform tf_start(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(1.085, 0.0, 1.565));
	tf::Transform tf_goal(tf::createQuaternionFromRPY(0, M_PI * 0.6, 0), tf::Vector3(1.085, 0.0, 1.565));
	Affine3d goal_transform;
	Affine3d start_transform;

	tf::transformTFToEigen(tf_start, start_transform);
	tf::transformTFToEigen(tf_goal, goal_transform);

	//Check first and last state on allowed collision
	kt_kinematic_state.setFromIK(joint_model_group_ptr, goal_transform);
	checkAllowedCollision(kt_kinematic_state, kt_planning_scene);
	kt_kinematic_state.setFromIK(joint_model_group_ptr, start_transform);
	checkAllowedCollision(kt_kinematic_state, kt_planning_scene);

	list<RobotStatePtr> trajectory;
	size_t approximate_steps = floor((goal_transform.translation() - start_transform.translation()).norm() /
			STANDARD_INTERPOLATION_STEP);
	bool is_interpolated = linearInterpolation(trajectory, kt_kinematic_state, goal_transform, approximate_steps);

	if (is_interpolated){
		thread check_collision_thread(checkCollision, trajectory, kt_planning_scene);
        checkJump(trajectory);
        splitTrajectorySegment(trajectory, EXPERIMENTAL_DISTANCE_CONSTRAINT);
		check_collision_thread.join();
	}
}