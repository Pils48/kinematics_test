/*********************************************************************
 * Kinematics test for linear planner with collision checking
 * using trac-ik as a kinematic solver.
 *********************************************************************/

#include <ros/ros.h>

#include <list>
#include <chrono>
#include <thread>
#include <iterator>

#include <math.h>
#include <angles/angles.h>
#include <geometric_shapes/shape_operations.h>
#include <Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include "../include/test_linear.h"

using namespace std;
using namespace moveit;
using namespace core;
using namespace tf;

int main(int argc, char **argv)
{
    //Initialization
    ros::init(argc, argv, "kinematics_test");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    robot_model_loader::RobotModelLoader kt_robot_model_loader(DEFAULT_ROBOT_DESCRIPTION);
    robot_model::RobotModelConstPtr kt_kinematic_model = kt_robot_model_loader.getModel();
    planning_scene_monitor::PlanningSceneMonitor kt_planning_scene_monitor(DEFAULT_ROBOT_DESCRIPTION);
    planning_scene::PlanningScenePtr kt_planning_scene = kt_planning_scene_monitor.getPlanningScene();
    RobotState kt_kinematic_state(kt_kinematic_model);
    ROS_INFO("Model frame: %s", kt_kinematic_model->getModelFrame().c_str());

    kt_kinematic_state.setToDefaultValues();
    const JointModelGroup *joint_model_group_ptr = kt_kinematic_model->getJointModelGroup(PLANNING_GROUP);

    moveit_visual_tools::MoveItVisualTools visual_tools(kt_kinematic_model->getModelFrame());
    namespace rvt = rviz_visual_tools;
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    //end of initialization

    Transform tf_start(createQuaternionFromRPY(0, 0, 0), Vector3(1.085, 0, 1.565));
    Transform tf_goal(createQuaternionFromRPY(0, M_PI_2 * 0.7, 0), Vector3(1.185, 0, 1.565));
    Eigen::Isometry3d start_transform;
    Eigen::Isometry3d goal_transform;
    transformTFToEigen(tf_goal, goal_transform);
    transformTFToEigen(tf_start, start_transform);

    //Check first and last state on allowed collision
//    kt_kinematic_state.setFromIK(joint_model_group_ptr, goal_transform);
//    checkAllowedCollision(kt_kinematic_state, kt_planning_scene);
//    kt_kinematic_state.setFromIK(joint_model_group_ptr, start_transform);
//    checkAllowedCollision(kt_kinematic_state, kt_planning_scene);

//    kt_kinematic_state.setFromIK(joint_model_group_ptr, goal_transform);
//    RobotState goal_state(kt_kinematic_state);
//    kt_kinematic_state.setFromIK(joint_model_group_ptr, start_transform);
//    RobotState start_state(kt_kinematic_state);
//
//    list<RobotState> trajectory;
//    LinearParams params = {joint_model_group_ptr, joint_model_group_ptr->getLinkModel(FANUC_M20IA_END_EFFECTOR),
//                           STANDARD_INTERPOLATION_STEP};
//    size_t approximate_steps = floor(getMaxTranslation(start_state, goal_state) / STANDARD_INTERPOLATION_STEP);
//    linearInterpolationTemplate(params, start_state, PoseAndStateInterpolator(tf_start, tf_goal, start_state, goal_state), TestIKSolver(),
//            approximate_steps, back_inserter(trajectory));
//
//    for (auto state_it = trajectory.begin(); state_it != prev(trajectory.end()); ++state_it) {
//        poseEigenToTF(state_it->getGlobalLinkTransform(FANUC_M20IA_END_EFFECTOR), tf_start);
//        poseEigenToTF(next(state_it)->getGlobalLinkTransform(FANUC_M20IA_END_EFFECTOR), tf_goal);
//        if (getMaxTranslation(*state_it, *next(state_it)) > LINEAR_TARGET_PRECISION){
//            if (!checkJumpTemplate(params,PoseAndStateInterpolator(tf_start, tf_goal, *state_it, *next(state_it)),
//                    TestIKSolver(), *state_it, *next(state_it)))
//                throw runtime_error("Invalid trajectory!");
//            advance(state_it,splitTrajectoryTemplate(inserter(trajectory, next(state_it)),
//                    PoseAndStateInterpolator(tf_start, tf_goal, *state_it, *next(state_it)), TestIKSolver(), params, *state_it, *next(state_it)));
//        }
//    }
//    checkCollision(trajectory, kt_planning_scene);
}