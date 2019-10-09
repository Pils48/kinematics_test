//
// Created by pils48 on 09.10.2019.
//

#include <list>
#include <chrono>
#include <iterator>

#include <math.h>
#include <angles/angles.h>
#include <geometric_shapes/shape_operations.h>
#include <Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include "../include/test_linear.h"

static constexpr double CONTINUITY_CHECK_THRESHOLD = M_PI * 0.001;
static const double ALLOWED_COLLISION_DEPTH = 0.0000001;
static const double LINEAR_TARGET_PRECISION = 0.005;
static const double STANDARD_INTERPOLATION_STEP = 0.01;

using namespace std;
using namespace moveit;
using namespace core;
using namespace tf;

struct RobotPosition {
    const RobotState &base_state;
    Transform &robot_pose;
};

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
    const Transform source;
    const Transform target;
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

class TestIKSolver {
public:
    bool setStateFromIK(const LinearParams &params, Transform &pose, RobotState &state) {
        Eigen::Isometry3d eigen_pose;
        poseTFToEigen(pose, eigen_pose);
        return state.setFromIK(params.group, eigen_pose, 0.0, [](RobotState *robot_state, const JointModelGroup *joint_group,
                                                                 const double *joint_group_variable_values){
            return true;
        });
    }
};

double getFullTranslation(RobotState &state, RobotState &next_state, string link_name)
{
    auto link_mesh_ptr = state.getLinkModel(link_name)->getShapes()[0].get();
    Eigen::Vector3d link_extends = shapes::computeShapeExtents(link_mesh_ptr);

    auto state_transform = state.getGlobalLinkTransform(link_name);
    auto next_state_transform = next_state.getGlobalLinkTransform(link_name);
    Eigen::Quaterniond start_quaternion(state_transform.rotation());
    Eigen::Quaterniond target_quaternion(next_state_transform.rotation());
    //Use shortestAngle in tf
    double sin_between_quaternions = sin(start_quaternion.angularDistance(target_quaternion));
    double diagonal_length = sqrt(pow(link_extends[0], 2) + pow(link_extends[1], 2) + pow(link_extends[2], 2));

    double linear_angular_distance = (state_transform.translation().norm() + diagonal_length) * sin_between_quaternions;

    return (state_transform.translation() - next_state_transform.translation()).norm() + linear_angular_distance;
}

double getMaxTranslation(RobotState &state, RobotState &next_state)
{
    double max_dist = 0;
    for (auto link_name : state.getJointModelGroup(PLANNING_GROUP)->getUpdatedLinkModelsWithGeometryNames())
        max_dist = max(max_dist, getFullTranslation(state, next_state, link_name));
    return max_dist;
}

template<typename Interpolator, typename IKSolver, typename OutputIterator>
bool linearInterpolationTemplate(const LinearParams &params, const RobotState &base_state, Interpolator &&interpolator,
                                 IKSolver &&solver, size_t steps, OutputIterator &&out)
{
    RobotState current(base_state);
    *out++ = current;
    Transform pose;
    for (size_t i = 1; i <= steps; ++i) {
        double percentage = (double) i / (double) steps;
        interpolator.interpolateByPercentage(percentage, pose, current);
        if (!solver.setStateFromIK(params, pose, current)){
            return false;
        }
        *out++ = current;
    }
    return true;
}

template<typename OutputIterator, typename Interpolator, typename IKSolver>
size_t splitTrajectoryTemplate(OutputIterator &&out, Interpolator &&interpolator, IKSolver &&solver, const LinearParams &params,
                                RobotState left, RobotState right)
{
    size_t segments;
    double percentage = 1;
    Transform mid_pose;
    RobotState mid(right);
    deque<RobotState> state_stack;
    state_stack.push_back(mid);
    while (!state_stack.empty()) {
        right = state_stack.back();
        interpolator.interpolateByPercentage(percentage, mid_pose, left);
        if (!solver.setStateFromIK(params, mid_pose, mid)){
            throw runtime_error("Invalid trajectory!");
        }
        if (getMaxTranslation(left, right) >= LINEAR_TARGET_PRECISION){
            percentage *= 0.5;
            state_stack.push_back(mid);
        }
        else {
            percentage /= 0.5;
            *out = mid;
            segments++;
            state_stack.pop_back();
            left = mid;
        }
    }
    return segments;
}

template<typename Interpolator, typename IKSolver>
bool checkJumpTemplate(const LinearParams &params, Interpolator &&interpolator, IKSolver &&solver, RobotState left, RobotState right)
{
    Transform mid_pose;
    RobotState mid(left);
    auto dist = left.distance(right);
    double offset = 0;
    double part = 1;
    while (part >= 0.00005 && dist > CONTINUITY_CHECK_THRESHOLD) {
        part *= 0.5;
        interpolator.interpolateByPercentage(0.5 + offset, mid_pose, mid);
        if (!solver.setStateFromIK(params, mid_pose, mid)){
            throw runtime_error("Invalid trajectory!");
        }
        auto lm = left.distance(mid);
        auto mr = mid.distance(right);
        if (lm < mr) {
            offset += part;
            left = mid;
            dist = mid.distance(right);
        } else {
            offset -= part;
            right = mid;
            dist = left.distance(mid);
        }
    }
    if (part < 0.00005)
        return false;

    return true;
}

void checkAllowedCollision(RobotState &state, planning_scene::PlanningScenePtr current_scene)
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
    } else
        throw runtime_error("Collision during the trajectory processing!\nInvalid trajectory!");
}

void checkCollision(list<RobotState> trajectory, planning_scene::PlanningScenePtr current_scene)
{
    for (auto state_it = next(trajectory.begin()); state_it != prev(trajectory.end()); ++state_it) {
        if (current_scene->isStateColliding(*state_it, PLANNING_GROUP, true))
            throw runtime_error("Collision during the trajectory processing!\nInvalid trajectory!");
    }
}

bool computeCartesianPath(const LinearParams &params,
        const moveit::core::RobotState &start_state,
        std::vector<moveit::core::RobotStatePtr>& traj,
        const std::vector<tf::Transform>& waypoints,
        const std::vector<moveit::core::RobotState> &base_waypoint_states,
        double const_step)
{

}

