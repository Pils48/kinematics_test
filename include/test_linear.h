//
// Created by pils48 on 09.10.2019.
//

#include <string>
#include <moveit/robot_state/robot_state.h>

#ifndef KINEMATICS_TEST_TEST_LINEAR_H
#define KINEMATICS_TEST_TEST_LINEAR_H
#define FANUC_M20IA_END_EFFECTOR "tool0"
#define DEFAULT_ROBOT_DESCRIPTION "robot_description"
#define PLANNING_GROUP "manipulator"

struct LinearParams {
    const moveit::core::JointModelGroup *group;
    const moveit::core::LinkModel *end_effector;
    const double interpolation_step;
};

double getFullTranslation(moveit::core::RobotState &state, moveit::core::RobotState &next_state, std::string link_name);

double getMaxTranslation(moveit::core::RobotState &state, moveit::core::RobotState &next_state);

template<typename Interpolator, typename IKSolver, typename OutputIterator>
bool linearInterpolationTemplate(const LinearParams &params, const moveit::core::RobotState &base_state, Interpolator &&interpolator,
                                 IKSolver &&solver, size_t steps, OutputIterator &&out);

template<typename OutputIterator, typename Interpolator, typename IKSolver>
size_t splitTrajectoryTemplate(OutputIterator &&out, Interpolator &&interpolator, IKSolver &&solver, const LinearParams &params,
                               moveit::core::RobotState left, moveit::core::RobotState right);

template<typename Interpolator, typename IKSolver>
bool checkJumpTemplate(const LinearParams &params, Interpolator &&interpolator, IKSolver &&solver, moveit::core::RobotState left, moveit::core::RobotState right);

void checkAllowedCollision(moveit::core::RobotState &state, planning_scene::PlanningScenePtr current_scene);

void checkCollision(std::list<moveit::core::RobotState> trajectory, planning_scene::PlanningScenePtr current_scene);

bool computeCartesianPath(const LinearParams &params,
                          const moveit::core::RobotState &start_state,
                          std::vector<moveit::core::RobotStatePtr>& traj,
                          const std::vector<tf::Transform>& waypoints,
                          const std::vector<moveit::core::RobotState> &base_waypoint_states,
                          double const_step);

#endif //KINEMATICS_TEST_TEST_LINEAR_H
