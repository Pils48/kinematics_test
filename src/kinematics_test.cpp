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

using namespace std;
using namespace moveit;
using namespace core;

vector<robot_state::RobotStatePtr> linearInterpolation(robot_state::RobotState kinematic_state,
        Eigen::Affine3d& goal_transform,
		const robot_state::JointModelGroup* jmg, size_t translation_steps, size_t rotation_steps, bool
		global_reference_frame = true){
    //Add JumpThreshold maintenance
    robot_state::RobotState current_state();
    std::vector<robot_state::RobotStatePtr> trail(0);
    trail.push_back(robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_state)));
    const moveit::core::LinkModel* ptr_link_model = kinematic_state.getLinkModel("tool0");
    // effector

    Eigen::Affine3d start_pose = kinematic_state.getGlobalLinkTransform(ptr_link_model);
    double* dbg_value = start_pose.data();

    // the target can be in the local reference frame (in which case we rotate it)
    Eigen::Affine3d rotated_target = global_reference_frame ? goal_transform : start_pose * goal_transform;

    Eigen::Quaterniond start_quaternion(start_pose.rotation());
    Eigen::Quaterniond target_quaternion(rotated_target.rotation());
    double rotation_distance = start_quaternion.angularDistance(target_quaternion);
    double translation_distance = (rotated_target.translation() - start_pose.translation()).norm();

    std::size_t steps = std::max(translation_steps, rotation_steps) + 1;

//    double last_valid_percentage = 0.0;
    for (std::size_t i = 1; i <= steps; ++i)
    {
        double percentage = (double)i / (double)steps;

        Eigen::Affine3d pose(start_quaternion.slerp(percentage, target_quaternion));
        double* dbg_mtx = pose.data();

        pose.translation() = percentage * rotated_target.translation() + (1 - percentage) * start_pose.translation();
        double* dbg_value = pose.translation().data();

        if (kinematic_state.setFromIK(jmg, pose, ptr_link_model->getName()))
            trail.push_back(robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_state)));
        else
            break;

//        last_valid_percentage = percentage;
    }
    return trail;
}

map<string, double*> findLinksDistance(vector<robot_state::RobotStatePtr>& trail,
        robot_state::RobotState& current_state){

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
            //Quaternion start_quaternion --> target_quaternion
            //target_quaternion = trans_quaternion * start_quaternion =>
            // => trans_quaternion = target_quaternion * start_quaternion^(-1)
            Eigen::Quaterniond trans_quaternion = target_quaternion * start_quaternion.inverse();

            //Get the farthest point
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
//	visual_tools.prompt("Press next to continue execution...");
    //end of initialization

	/* Test trac-ik
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
    const Eigen::Affine3d start_pose(Eigen::Translation3d(1.0, -0.1, 0.2));
    kinematic_state.setFromIK(joint_model_group, start_pose);
    visual_tools.publishRobotState(kinematic_state, rvt::BLUE);

    vector<robot_state::RobotStatePtr> traj = linearInterpolation(kinematic_state, goal_transform,
            joint_model_group, 6, 6); //Approximate steps

    robot_trajectory::RobotTrajectory trail(kinematic_model, joint_model_group);
    for (robot_state::RobotStatePtr state : traj){
        trail.addSuffixWayPoint(state, 1.0);
    }

    map<string, double*> links_distances = findLinksDistance(traj, kinematic_state);

    moveit_msgs::RobotTrajectory traj_msg;
    trail.getRobotTrajectoryMsg(traj_msg);
//    bool success = visual_tools.publishTrajectoryPath(trail);
    bool success = visual_tools.publishTrajectoryLine(traj_msg, joint_model_group, rvt::colors::GREEN);
    visual_tools.trigger();
    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "Motion Planning API Demo", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    visual_tools.publishRobotState(kinematic_state, rvt::WHITE);

    kt_planning_scene->getCollisionRobot();

    //Visualize trajectory
     for (std::size_t i = 0; i < traj.size(); i++){
         std::this_thread::sleep_for(std::chrono::milliseconds(200));
         visual_tools.publishRobotState(traj[i]);
         std::this_thread::sleep_for(std::chrono::milliseconds(200));
         visual_tools.deleteAllMarkers();
     }

}