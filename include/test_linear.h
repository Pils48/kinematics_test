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

using namespace std;
using namespace moveit;
using namespace core;
using namespace tf;

class TransformSlerper {
public:
    TransformSlerper(const Transform &source, const Transform &target)
            : source(source), target(target), source_rotation(source.getRotation()),
              target_rotation(target.getRotation()), total_distance(target.getOrigin().distance(source.getOrigin())),
              total_angle(source_rotation.angleShortestPath(target_rotation)) {}

    Transform slerpByPercentage(double percentage) const {
        return Transform(
                source_rotation.slerp(target_rotation, percentage),
                percentage * target.getOrigin() + (1 - percentage) * source.getOrigin()
        );
    }

    Transform slerpByDistance(double distance) const {
        if (total_distance < numeric_limits<double>::epsilon())
            return source;
        return slerpByPercentage(distance / total_distance);
    }

    Transform slerpByAngle(double angle) const {
        if (total_angle < numeric_limits<double>::epsilon())
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
    const Transform &source;
    const Transform &target;
    Quaternion source_rotation, target_rotation;
    double total_distance;
    double total_angle;
};

class PoseAndStateInterpolator {
public:
    PoseAndStateInterpolator(
            const Transform &source,
            const Transform &target,
            const RobotState &start_state,
            const RobotState &end_state
    ) : _slerper(source, target), _start_state(start_state), _end_state(end_state) {
    }

    void interpolateByPercentage(double percentage, Transform &pose, RobotState &state) {
        pose = _slerper.slerpByPercentage(percentage);
        _start_state.interpolate(_end_state, percentage, state);
        state.update();
    }

    void interpolateByDistance(double distance, Transform &pose, RobotState &state) const {
        pose = _slerper.slerpByDistance(distance);
        _start_state.interpolate(_end_state, distance / totalDistance(), state);
        state.update();
    }

    void interpolateByAngle(double angle, Transform &pose, RobotState &state) const {
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
    const RobotState &_start_state;
    const RobotState &_end_state;
};

struct LinearParams {
    const JointModelGroup *group;
    const LinkModel *end_effector;
    const Transform &ee_offset;
};

struct RobotInterpolationState{
     Transform ee_pose;
     RobotState base_state; //base_state for some targets are the same
     double percentage;
};

class TestIKSolver {
public:
    bool setStateFromIK(const LinearParams &params, const Transform &pose, RobotState &state) {
        Eigen::Isometry3d eigen_pose;
        poseTFToEigen(pose, eigen_pose);
        return state.setFromIK(params.group, eigen_pose, 0.0, [](RobotState *robot_state, const JointModelGroup *joint_group,
                                                                 const double *joint_group_variable_values){
            return true;
        });
    }
};

double getFullTranslation(RobotState &state, RobotState &next_state, string link_name);

double getMaxTranslation(RobotState &state, RobotState &next_state);

double getMaxTranslation(const RobotInterpolationState &state, const RobotInterpolationState &next_state);

void checkAllowedCollision(RobotState &state, planning_scene::PlanningScenePtr current_scene);

void checkCollision(list<RobotState> trajectory, planning_scene::PlanningScenePtr current_scene);

bool computeCartesianPath(
        const LinearParams &params,
        Transform start_pose,
        Transform goal_pose,
        RobotState &base_state,
        vector<RobotInterpolationState> &traj,
        double const_step);

#endif //KINEMATICS_TEST_TEST_LINEAR_H
