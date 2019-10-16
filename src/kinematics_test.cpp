/*********************************************************************
 * Kinematics test for linear planner with collision checking
 * using trac-ik as a kinematic solver.
 *********************************************************************/

#include <ros/ros.h>

#include <list>
#include <chrono>
#include <thread>

#include <math.h>
#include <Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include "../include/test_linear.h"
//#include "../src/test_linear.cpp"

using namespace std;
using namespace moveit;
using namespace core;
using namespace tf;

static const double STANDARD_INTERPOLATION_STEP = 0.01;
static const double LINEAR_TARGET_PRECISION = 0.005;

int main(int argc, char **argv) {
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

    const Transform tf_start(createQuaternionFromRPY(0, 0, 0), Vector3(1.085, 0, 1.565));
    const Transform tf_goal(createQuaternionFromRPY(0, M_PI_4, 0), Vector3(1.086, 0, 1.565));

    LinearParams params = {joint_model_group_ptr, joint_model_group_ptr->getLinkModel(FANUC_M20IA_END_EFFECTOR), Transform::getIdentity()};
    vector<RobotInterpolationState> trajectory;
    bool ok = computeCartesianPath(params, tf_start, tf_goal, kt_kinematic_state, trajectory, STANDARD_INTERPOLATION_STEP);

    for (auto it = trajectory.begin(); it != trajectory.end(); ++it){
        this_thread::sleep_for(chrono::milliseconds(10));
        Eigen::Isometry3d e_t;
        transformTFToEigen(it->ee_pose, e_t);
        kt_kinematic_state.setFromIK(joint_model_group_ptr, e_t);
        visual_tools.publishRobotState(kt_kinematic_state, rvt::BLUE);
        this_thread::sleep_for(chrono::milliseconds(10));
        visual_tools.deleteAllMarkers();
    }
}