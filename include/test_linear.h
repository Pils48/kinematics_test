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

class TransformSlerper {
public:
    TransformSlerper(const tf::Transform &source, const tf::Transform &target)
            : source(source), target(target), source_rotation(source.getRotation()),
              target_rotation(target.getRotation()), total_distance(target.getOrigin().distance(source.getOrigin())),
              total_angle(source_rotation.angleShortestPath(target_rotation)) {}

    tf::Transform slerpByPercentage(double percentage) const {
        return tf::Transform(
                source_rotation.slerp(target_rotation, percentage),
                percentage * target.getOrigin() + (1 - percentage) * source.getOrigin()
        );
    }

    tf::Transform slerpByDistance(double distance) const {
        if (total_distance < std::numeric_limits<double>::epsilon())
            return source;
        return slerpByPercentage(distance / total_distance);
    }

    tf::Transform slerpByAngle(double angle) const {
        if (total_angle < std::numeric_limits<double>::epsilon())
            return source;
        return slerpByPercentage(angle / total_angle);
    }

    double totalDistance() const {
        return total_distance;
    }

    double totalAngle() const {
        return total_angle;
    }

private:
    const tf::Transform &source;
    const tf::Transform &target;
    tf::Quaternion source_rotation, target_rotation;
    double total_distance;
    double total_angle;
};

class PoseAndStateInterpolator {
public:
    PoseAndStateInterpolator(
            const tf::Transform &source,
            const tf::Transform &target,
            const moveit::core::RobotState &start_state,
            const moveit::core::RobotState &end_state
    ) : _slerper(source, target), _start_state(start_state), _end_state(end_state) {
    }

    void interpolateByPercentage(double percentage, tf::Transform &pose, moveit::core::RobotState &state) {
        pose = _slerper.slerpByPercentage(percentage);
        _start_state.interpolate(_end_state, percentage, state);
        state.update();
    }

    void interpolateByDistance(double distance, tf::Transform &pose, moveit::core::RobotState &state) const {
        pose = _slerper.slerpByDistance(distance);
        _start_state.interpolate(_end_state, distance / totalDistance(), state);
        state.update();
    }

    void interpolateByAngle(double angle, tf::Transform &pose, moveit::core::RobotState &state) const {
        pose = _slerper.slerpByAngle(angle);
        _start_state.interpolate(_end_state, angle / totalAngle(), state);
        state.update();
    }

    double totalDistance() const {
        return _slerper.totalDistance();
    }

    double totalAngle() const {
        return _slerper.totalAngle();
    }

private:
    TransformSlerper _slerper;
    const moveit::core::RobotState &_start_state;
    const moveit::core::RobotState &_end_state;
};

struct LinearParams {
    const moveit::core::JointModelGroup *group;
    const moveit::core::LinkModel *end_effector;
    const tf::Transform &ee_offset;
};

struct RobotInterpolationState{
     const tf::Transform &ee_pose;
     const moveit::core::RobotState &base_state; //base_state for some targets are the same
     double percentage;
};

class TestIKSolver {
public:
    bool setStateFromIK(const LinearParams &params, const tf::Transform &pose, moveit::core::RobotState &state) {
        Eigen::Isometry3d eigen_pose;
        poseTFToEigen(pose, eigen_pose);
        return state.setFromIK(params.group, eigen_pose, 0.0, [](moveit::core::RobotState *robot_state, const moveit::core::JointModelGroup *joint_group,
                                                                 const double *joint_group_variable_values){
            return true;
        });
    }
};

double getFullTranslation(moveit::core::RobotState &state, moveit::core::RobotState &next_state, std::string link_name);

double getMaxTranslation(moveit::core::RobotState &state, moveit::core::RobotState &next_state);

double getMaxTranslation(const RobotInterpolationState &state, const RobotInterpolationState &next_state);

void checkAllowedCollision(moveit::core::RobotState &state, planning_scene::PlanningScenePtr current_scene);

void checkCollision(std::list<moveit::core::RobotState> trajectory, planning_scene::PlanningScenePtr current_scene);

bool computeCartesianPath(
        const LinearParams &params,
        const tf::Transform start_pose,
        const tf::Transform goal_pose,
        const moveit::core::RobotState &base_state,
        std::vector<RobotInterpolationState> &traj,
        double const_step);

#endif //KINEMATICS_TEST_TEST_LINEAR_H
