//
// Created by pils48 on 09.10.2019.
//

#include <list>
#include <chrono>
#include <iterator>


#include <math.h>
#include <geometric_shapes/shape_operations.h>
#include <Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>

#include <moveit/robot_state/robot_state.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include "../include/test_linear.h"

static constexpr double CONTINUITY_CHECK_THRESHOLD = M_PI * 0.001;
static const double ALLOWED_COLLISION_DEPTH = 0.0000001;
static const double DISTANCE_TOLERANCE = 0.0001; //Random

using namespace std;
using namespace moveit;
using namespace core;
using namespace tf;

static const double STANDARD_INTERPOLATION_STEP = 0.01;
static const double LINEAR_TARGET_PRECISION = 0.005;

double getFullTranslation(const Transform &left, const Transform &right, const LinkModel* link)
{
    //Подумать стоит ли добавлять расстояние от оси
    auto link_mesh_ptr = link->getShapes()[0].get();
    Eigen::Vector3d link_extends = shapes::computeShapeExtents(link_mesh_ptr);

    double sin_between_quaternions = sin(left.getRotation().angleShortestPath(right.getRotation()));
    double diagonal_length = sqrt(pow(link_extends[0], 2) + pow(link_extends[1], 2) + pow(link_extends[2], 2));
//    double linear_angular_distance = (left.getOrigin().length() + diagonal_length) * sin_between_quaternions;
    double linear_angular_distance = diagonal_length * sin_between_quaternions;

    return left.getOrigin().distance(right.getOrigin()) + linear_angular_distance;
}

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

double getMaxTranslation(const RobotInterpolationState &state, const RobotInterpolationState &next_state)
{
    double max_dist = 0;
    for (auto link : state.base_state.getJointModelGroup(PLANNING_GROUP)->getUpdatedLinkModelsWithGeometry())
        max_dist = max(max_dist, getFullTranslation(state.ee_pose, next_state.ee_pose, link));
    return max_dist;
}

template<typename Interpolator, typename IKSolver, typename OutputIterator>
bool linearInterpolationTemplate(const LinearParams &params, const RobotState &base_state, Interpolator &interpolator,
                                 IKSolver &&solver, size_t steps, OutputIterator &&out)
{
    Transform pose;
    RobotState current(base_state);
    double percentage = 0;
    for (size_t i = 1; i <= steps; ++i) {
        percentage = (double) i / (double) steps;
        interpolator.interpolateByPercentage(percentage, pose, current);
        *out++ = {pose, base_state, percentage};
    }
    return true;
}

template<typename OutputIterator, typename Interpolator, typename IKSolver>
size_t splitTrajectoryTemplate(OutputIterator &&out, Interpolator &interpolator, IKSolver &&solver, const LinearParams &params,
                                RobotInterpolationState left, RobotInterpolationState right)
{
    size_t segments = 0;
    Transform mid_pose;
    RobotInterpolationState mid_inter_state(right);
    deque<RobotInterpolationState> state_stack(1, right);
    while (!state_stack.empty()) {
        right = state_stack.back();
        interpolator.interpolateByPercentage((right.percentage + left.percentage) * 0.5, mid_pose, left.base_state);
        mid_inter_state.percentage = (right.percentage + left.percentage) * 0.5;
        mid_inter_state.ee_pose = mid_pose;
        if (getMaxTranslation(left, right) >= LINEAR_TARGET_PRECISION) {
            state_stack.push_back(mid_inter_state);
        } else {
            //пушим лишнее
            if (state_stack.size() != 1)
                *out++ = mid_inter_state;
            segments++;
            state_stack.pop_back();
            left = mid_inter_state;
        }
    }
//    ROS_WARN("Segments : %d", segments);
    return segments;
}

template<typename Interpolator, typename IKSolver>
bool checkJumpTemplate(const LinearParams &params, Interpolator &interpolator, IKSolver &&solver,
                                const RobotInterpolationState &left_state, const RobotInterpolationState &right_state)
{
    Transform mid_pose;
    Eigen::Isometry3d left_transform, right_transform;
    transformTFToEigen(left_state.ee_pose, left_transform);
    transformTFToEigen(right_state.ee_pose, right_transform);
    RobotState left(left_state.base_state);
    left.setFromIK(params.group, left_transform);
    RobotState right(right_state.base_state);
    right.setFromIK(params.group, right_transform);
    RobotState mid(left_state.base_state);

    auto dist = left.distance(right);
    double offset = 0;
    double percentage = 1;
    while (percentage >= 0.00005 && dist > CONTINUITY_CHECK_THRESHOLD) {
        percentage *= 0.5;
        interpolator.interpolateByPercentage(percentage + offset, mid_pose, mid);
        if (!solver.setStateFromIK(params, mid_pose, mid))
            throw runtime_error("Invalid trajectory!");
        auto lm = left.distance(mid);
        auto mr = mid.distance(right);
        if (lm < mr) {
            offset += percentage;
            left = mid;
            dist = mid.distance(right);
        } else {
            offset = (offset - percentage) > 0 ? offset - percentage : 0;
            right = mid;
            dist = left.distance(mid);
        }
    }
    if (percentage < 0.00005)
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

template <typename Interpolator, typename IKSolver>
bool computeCartesianPath(
        const LinearParams &params,
        const RobotInterpolationState &left_state,
        const RobotInterpolationState &right_state,
        vector<RobotInterpolationState> &traj,
        IKSolver &&solver,
        double const_step,
        double distance_constraint)
{
    RobotState start_state(left_state.base_state);

    bool use_const_step = const_step > 0.0;

    Interpolator interpolator(left_state.ee_pose, right_state.ee_pose, left_state.base_state, right_state.base_state);

    if (solver.setStateFromIK(params, left_state.ee_pose, start_state)){//что делает эта проверка
        traj.push_back(left_state);
    }

    //Проверка траектории на малость
    //NOP

    //Интерполяция, добавить исключительные ситуации
    size_t approximate_steps = floor(getMaxTranslation(left_state, right_state) / const_step);
    linearInterpolationTemplate(params, start_state, interpolator, TestIKSolver(), approximate_steps, back_inserter(traj));

    //Сегментация траектории
    for (auto state_it = traj.begin(); state_it != prev(traj.end()); ++state_it) {
        if (getMaxTranslation(*state_it, *next(state_it)) > LINEAR_TARGET_PRECISION){
            if (!checkJumpTemplate(params,  interpolator, TestIKSolver(), *state_it, *next(state_it)))
                throw runtime_error("Invalid trajectory!");
//            advance(state_it, splitTrajectoryTemplate(inserter(traj, state_it), interpolator,
//                    TestIKSolver(), params, *state_it, *next(state_it)));
//                splitTrajectoryTemplate(inserter(traj, state_it), interpolator,
//                    TestIKSolver(), params, *state_it, *next(state_it));
        }
    }
    return true;
}

bool computeCartesianPath(
        const LinearParams &params,
        Transform start_pose,
        Transform goal_pose,
        RobotState &base_state,
        vector<RobotInterpolationState> &traj,
        double const_step)
{
    const RobotInterpolationState start_state = {start_pose, base_state, 0};
    const RobotInterpolationState goal_state = {goal_pose, base_state, 1};

    vector<RobotInterpolationState> wp_traj;
    if (!computeCartesianPath<PoseAndStateInterpolator>(params, start_state, goal_state, wp_traj, TestIKSolver(), STANDARD_INTERPOLATION_STEP, LINEAR_TARGET_PRECISION))
        throw runtime_error("Invalid trajectory!");
}


