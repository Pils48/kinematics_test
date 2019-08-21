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
#define EXPERIMENTAL_DISTANCE_CONSTRAINT 2

using namespace std;
using namespace moveit;
using namespace core;

/** Interpolate trajectory using slerp quaternion algorithm and linear algorithms
 * for translation parameter. Return true in case of success.*/
bool linearInterpolation(std::vector<robot_state::RobotStatePtr>& trail,
        robot_state::RobotState kinematic_state, const Eigen::Affine3d& goal_transform, JumpThreshold& jump_threshold,
        size_t translation_steps, bool global_reference_frame = true){

    const robot_state::JointModelGroup* jmg = kinematic_state.getJointModelGroup("manipulator");
//    std::vector<robot_state::RobotStatePtr> trail_copy = trail;
    robot_state::RobotState current_state();
    trail.push_back(robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_state)));
    const moveit::core::LinkModel* ptr_link_model = kinematic_state.getLinkModel("tool0");
    // effector

    Eigen::Affine3d start_pose = kinematic_state.getGlobalLinkTransform(ptr_link_model);

    // the target can be in the local reference frame (in which case we rotate it)
    Eigen::Affine3d rotated_target = global_reference_frame ? goal_transform : start_pose * goal_transform;

    Eigen::Quaterniond start_quaternion(start_pose.rotation());
    Eigen::Quaterniond target_quaternion(rotated_target.rotation());
    double rotation_distance = start_quaternion.angularDistance(target_quaternion);
    double translation_distance = (rotated_target.translation() - start_pose.translation()).norm();

    std::size_t steps = translation_steps + 1;

    std::vector<double> consistency_limits;
    if (jump_threshold.prismatic > 0 || jump_threshold.revolute > 0)
        for (const JointModel* jm : jmg->getActiveJointModels())
        {
            double limit;
            switch (jm->getType())
            {
                case JointModel::REVOLUTE:
                    limit = jump_threshold.revolute;
                    break;
                case JointModel::PRISMATIC:
                    limit = jump_threshold.prismatic;
                    break;
                default:
                    limit = 0.0;
            }
            if (limit == 0.0)
                limit = jm->getMaximumExtent();
            consistency_limits.push_back(limit);
        }

    double last_valid_percentage = 0.0;
    for (std::size_t i = 1; i <= steps; ++i)
    {
        double percentage = (double)i / (double)steps;

        Eigen::Affine3d pose(start_quaternion.slerp(percentage, target_quaternion));

        pose.translation() = percentage * rotated_target.translation() + (1 - percentage) * start_pose.translation();

        if (kinematic_state.setFromIK(jmg, pose, ptr_link_model->getName(), consistency_limits))
            trail.push_back(robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_state))); //tmp work
            // with original
        else{
            ROS_ERROR("Impossible to create whole path! Check self-collision or limits excess.");
            return false;
        }
        last_valid_percentage = percentage;
    }
//    last_valid_percentage *= CartesianInterpolator::checkJointSpaceJump(jmg, trail_copy,
//            jump_threshold);

    return true;
}

map<string, double*> findLinksDistance(vector<robot_state::RobotStatePtr>& trail,
        robot_state::RobotState& current_state, double critical_distance){

    map<string, double*> result_distances;

    for (size_t idx = 1; idx <= 6; idx++){

        string link_name = string("link_") + to_string(idx);
        pair<string, double*> key_value;
        key_value.first = link_name;
        double link_distances[trail.size() - 1];
        key_value.second = link_distances;

        const LinkModel* ptr_link = current_state.getLinkModel(link_name);

        //Work with the greatest translation of the link
        const shapes::Shape* link_mesh = ptr_link->getShapes().at(0).get();
        Eigen::Vector3d link_extends = shapes::computeShapeExtents(link_mesh);
        Eigen::Affine3d mesh_origin_transform = ptr_link->getVisualMeshOrigin();

        for (size_t i = 0; i < trail.size() - 1; i++){

            Eigen::Affine3d state = trail[i]->getGlobalLinkTransform(link_name);
            Eigen::Affine3d next_state = trail[i + 1]->getGlobalLinkTransform(link_name);

            Eigen::Quaterniond start_quaternion(state.rotation());
            Eigen::Quaterniond target_quaternion(next_state.rotation());
            /**Quaternion start_quaternion --> target_quaternion
             *target_quaternion = trans_quaternion * start_quaternion =>
             *=> trans_quaternion = target_quaternion * start_quaternion^(-1)*/
            Eigen::Quaterniond trans_quaternion = target_quaternion * start_quaternion.inverse();

            /**Get the farthest point:
             *take the point, that has a maximum distance to global origin, in case of different axis orientation
             * introduce coefficients that will mean whether we should add or subtract half extend size
            */
            int x_coeff, y_coeff, z_coeff;
            mesh_origin_transform(0, 0) > 0 ? x_coeff = 1 : x_coeff = -1;
            mesh_origin_transform(1, 1) > 0 ? y_coeff = 1 : y_coeff = -1;
            mesh_origin_transform(2, 2) > 0 ? z_coeff = 1 : z_coeff = -1;

            Eigen::Vector3d diagonal_point_link = Eigen::Vector3d(x_coeff * link_extends(0) / 2,
                    y_coeff * link_extends(1) / 2, z_coeff * link_extends(2) / 2);
            Eigen::Vector3d diagonal_point_global = mesh_origin_transform * diagonal_point_link;
            //Rotate vector(global_origin, diagonal_point_link)
            Eigen::Vector3d end_point_global = trans_quaternion.toRotationMatrix() * diagonal_point_global;
            double linear_angular_distance = (end_point_global - diagonal_point_global).norm();

            double translation_distance = (next_state.translation() - state.translation()).norm() + linear_angular_distance;
            key_value.second[i] = translation_distance;

            ROS_INFO("%s the greatest translation distance: %f", link_name.c_str(), translation_distance);
            if (translation_distance >= critical_distance){
                ROS_WARN("%s seams to pass to great distance: %f", link_name.c_str(), translation_distance);
                //We twice steps number
                JumpThreshold jump_threshold;
                jump_threshold.prismatic = 5.0;
                jump_threshold.factor = 1.0;
                jump_threshold.revolute = 1.0; //Random values
                robot_state::RobotState kinematic_state = *trail[0];
                const Eigen::Affine3d& goal_transform = trail[trail.size() - 1]->getGlobalLinkTransform("tool0");
                size_t translation_steps = (trail.size() - 1) * 2;
                trail.clear();
                bool is_interpolated = linearInterpolation(trail, kinematic_state, goal_transform,jump_threshold,
                        translation_steps);
                findLinksDistance(trail, kinematic_state, critical_distance);
            }
        }

        result_distances.insert(result_distances.end(), key_value);
    }
    return result_distances;
};

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
	
	std::vector<double> joint_values;
	double timeout = 0.1;
	bool found_ik = kinematic_state.setFromIK(joint_model_group, end_effector_state, timeout);
	
	if(found_ik){
		visual_tools.publishRobotState(kinematic_state, rvt::colors::BLUE);
	}
	visual_tools.prompt("Press next to continue execution...");

    kinematic_state.setToDefaultValues(); */

	Eigen::Affine3d goal_transform(Eigen::Translation3d(1.0, 0.0, 0.4));
    const Eigen::Affine3d start_pose(Eigen::Translation3d(1.0, 0.1, 0.3));
    kinematic_state.setFromIK(joint_model_group, start_pose);
    visual_tools.publishRobotState(kinematic_state, rvt::BLUE);

    JumpThreshold jump_threshold;
    jump_threshold.prismatic = 5.0;
    jump_threshold.factor = 1.0;
    jump_threshold.revolute = 1.0; //Random values
    std::vector<robot_state::RobotStatePtr> traj(0);

    size_t approximate_steps = floor((goal_transform.translation() - start_pose.translation()).norm() /
            STANDARD_INTERPOLATION_STEP);
    bool is_interpolated = linearInterpolation(traj, kinematic_state, goal_transform,
            jump_threshold, approximate_steps);

    robot_trajectory::RobotTrajectory trail(kinematic_model, joint_model_group);

    map<string, double*> links_distances = findLinksDistance(traj, kinematic_state, EXPERIMENTAL_DISTANCE_CONSTRAINT);

//    moveit_msgs::RobotTrajectory traj_msg;
//    trail.getRobotTrajectoryMsg(traj_msg);
//    bool success = visual_tools.publishTrajectoryPath(trail);
//    bool success = visual_tools.publishTrajectoryLine(traj_msg, joint_model_group, rvt::colors::GREEN);
//    visual_tools.trigger();

    //Visualize trajectory
     for (std::size_t i = 0; i < traj.size(); i++){
         std::this_thread::sleep_for(std::chrono::milliseconds(200));
         visual_tools.publishRobotState(traj[i]);
         std::this_thread::sleep_for(std::chrono::milliseconds(200));
         visual_tools.deleteAllMarkers();
     }

}