/*********************************************************************
 * Kinematics test for linear planner with collision checking
 * using trac-ik as a kinematic solver.
 *********************************************************************/

#include <ros/ros.h>

#include <list>
#include <chrono>
#include <thread>
#include <iterator>

#include <angles/angles.h>
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
using namespace Eigen;

static constexpr double CONTINUITY_CHECK_THRESHOLD = M_PI * 0.0001;
static const double ALLOWED_COLLISION_DEPTH = 0.0000001;
static const double LINEAR_TARGET_PRECISION = 0.003;
static const double STANDARD_INTERPOLATION_STEP = 0.01;

struct RobotPosition
{
    RobotState& base_state;
    tf::Transform robot_pose;
};

class TransformSlerper
{
public:
    TransformSlerper(const tf::Transform &source, const tf::Transform &target)
            : source(source)
            , target(target)
            , source_rotation(source.getRotation())
            , target_rotation(target.getRotation())
            , total_distance(target.getOrigin().distance(source.getOrigin()))
            , total_angle(source_rotation.angleShortestPath(target_rotation))
    {}

    tf::Transform slerpByPercentage(double percentage) const
    {
        return tf::Transform(
                source_rotation.slerp(target_rotation, percentage),
                percentage * target.getOrigin() + (1 - percentage) * source.getOrigin()
        );
    }

    tf::Transform slerpByDistance(double distance) const
    {
        if (total_distance < std::numeric_limits<double>::epsilon())
            return source;
        return slerpByPercentage(distance / total_distance);
    }

    tf::Transform slerpByAngle(double angle) const
    {
        if (total_angle < std::numeric_limits<double>::epsilon())
            return source;
        return slerpByPercentage(angle / total_angle);
    }

    double totalDistance() const
    {
        return total_distance;
    }

    double totalAngle() const
    {
        return total_angle;
    }

private:
    const tf::Transform source;
    const tf::Transform target;
    tf::Quaternion source_rotation, target_rotation;
    double total_distance;
    double total_angle;
};

class PoseAndStateInterpolator
{
public:
    PoseAndStateInterpolator(
            const tf::Transform &source,
            const tf::Transform &target,
            const RobotState &start_state,
            const RobotState &end_state
            ) : _slerper(source, target)
            , _start_state(start_state)
            , _end_state(end_state)
    {
    }

    PoseAndStateInterpolator(
            const Isometry3d& source,
            const Isometry3d& target,
            const RobotState& start_state,
            const RobotState& end_state
            ) : _start_state(start_state)
            , _end_state(end_state)
            , _slerper(paramToTF(source), paramToTF(target))
    {
    }

    tf::Transform paramToTF(const Isometry3d& source){
        tf::Transform res_pose;
        tf::poseEigenToTF(source, res_pose);
        return res_pose;
    }

    void interpolateByPercentage(double percentage, tf::Transform &pose, RobotState &state)
    {
        pose = _slerper.slerpByPercentage(percentage);
        _start_state.interpolate(_end_state, percentage, state);
        state.update();
    }

    void interpolateByDistance(double distance, tf::Transform &pose, RobotState &state) const
    {
        pose = _slerper.slerpByDistance(distance);
        _start_state.interpolate(_end_state, distance / totalDistance(), state);
        state.update();
    }

    void interpolateByAngle(double angle, tf::Transform &pose, RobotState &state) const
    {
        pose = _slerper.slerpByAngle(angle);
        _start_state.interpolate(_end_state, angle / totalAngle(), state);
        state.update();
    }

    double totalDistance() const
    {
        return _slerper.totalDistance();
    }

    double totalAngle() const
    {
        return _slerper.totalAngle();
    }

private:
    TransformSlerper _slerper;
    const RobotState &_start_state;
    const RobotState &_end_state;
};

struct LinearParams{
    const JointModelGroup* group;
    const LinkModel* end_effector;
    const double interpolation_step;
};

class TestIKSolver{
public:
    bool setStateFromIK(const LinearParams& params, tf::Transform& pose, RobotState& state)
    {
        Isometry3d eigen_pose;
        tf::poseTFToEigen(pose, eigen_pose);
        return state.setFromIK(params.group, eigen_pose);
    }
};

template <typename Interpolator>
Interpolator getInterpolator(const LinearParams& params, RobotState& left, RobotState& right)
{
    tf::Transform left_pose;
    tf::Transform right_pose;
    tf::poseEigenToTF(left.getGlobalLinkTransform(params.end_effector), left_pose);
    tf::poseEigenToTF(right.getGlobalLinkTransform(params.end_effector), right_pose);
    Interpolator interpolator(left_pose, right_pose, left, right);
    return interpolator;
}

template <typename Interpolator>
RobotState getMidState(const LinearParams& params, RobotState& start_state, RobotState& end_state)
{
    tf::Transform pose;
    RobotState result(start_state);
    //Use interface to make abstract factory??
    auto interpolator = getInterpolator<PoseAndStateInterpolator>(params, start_state, end_state);
    interpolator.interpolateByPercentage(0.5, pose, result);
    return result;
}

double getFullTranslation(RobotState& state, RobotState& next_state, string link_name)
{
    auto link_mesh_ptr = state.getLinkModel(link_name)->getShapes()[0].get();
    Vector3d link_extends = shapes::computeShapeExtents(link_mesh_ptr);

    auto state_transform = state.getGlobalLinkTransform(link_name);
    auto next_state_transform = next_state.getGlobalLinkTransform(link_name);
    Quaterniond start_quaternion(state_transform.rotation());
    Quaterniond target_quaternion(next_state_transform.rotation());

    double sin_between_quaternions = sin(start_quaternion.angularDistance(target_quaternion));
    double diagonal_length = sqrt(pow(link_extends[0], 2) + pow(link_extends[1], 2) + pow(link_extends[2], 2));

    double linear_angular_distance = (state_transform.translation().norm() + diagonal_length) * sin_between_quaternions;

    return (state_transform.translation() - next_state_transform.translation()).norm() + linear_angular_distance;
}

double getMaxTranslation(RobotState& state, RobotState& next_state){
    double max_dist = 0;
    for (auto link : state.getJointModelGroup(PLANNING_GROUP)->getUpdatedLinkModelsWithGeometry())
        max_dist = max(max_dist, getFullTranslation(state, next_state, link->getName()));
    return max_dist;
}

template <typename Interpolator, typename IKSolver, typename OutputIterator>
bool linearInterpolationTemplate(const LinearParams& params, const RobotState& base_state, Interpolator&& interpolator,
        IKSolver&& solver, size_t steps, OutputIterator out)
{
    RobotState current(base_state);
    *out++ = current;
    tf::Transform pose;
    for (size_t i = 1; i <= steps; ++i) {
        double percentage = (double) i / (double) steps;
        interpolator.interpolateByPercentage(percentage, pose, current);
        *out++ = current;
    }
    return true;
}

template <typename OutputIterator, typename Interpolator>
size_t splitTrajectoryTemplate(OutputIterator out, Interpolator&& interpolator, const LinearParams& params,
        RobotState& state, RobotState& next_state)
{
    size_t segments;
    RobotState left(state);
    RobotState right(next_state);
    RobotState mid(next_state);
    deque<RobotState> state_stack;
    vector<RobotState> buffer;
    state_stack.push_back(mid);
    while (!state_stack.empty()){
        right = state_stack.back();
        mid = getMidState<Interpolator>(params, left, right);
        double translation = getMaxTranslation(left, right);
        ROS_WARN("Too great translation: %f", translation);
        if (getMaxTranslation(left, right) >= LINEAR_TARGET_PRECISION)
            state_stack.push_back(mid);
        else {
            buffer.push_back(mid);
            state_stack.pop_back();
            left = mid;
        }
    }
    copy(buffer.begin(), buffer.end(), out);
    segments = buffer.size();
    return segments;
}

template <typename Interpolator, typename IKSolver>
bool checkJumpTemplate(const LinearParams& params, Interpolator&& interpolator, IKSolver&& solver,
                RobotState state, RobotState next_state)
{
    tf::Transform mid_pose;
    RobotState mid(state);
    auto dist = state.distance(next_state);
    double percentage = 1;
    while (percentage >= 0.00005 && dist > CONTINUITY_CHECK_THRESHOLD){
        percentage *= 0.5;
        interpolator.interpolateByDistance(params.interpolation_step / 2, mid_pose, mid);
        if (!solver.setStateFromIK(params, mid_pose, mid))
            return false;
        auto lm = state.distance(mid);
        auto mr = mid.distance(next_state);
        if (lm < mr){
            state = mid;
            dist = mid.distance(next_state);
        }
        else
        {
            next_state = mid;
            dist = state.distance(mid);
        }
    }
    if (percentage < 0.00005)
        return false;

    return true;
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

    planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

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

    tf::Transform tf_start(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(1.085, 0, 1.565));
    tf::Transform tf_goal(tf::createQuaternionFromRPY(0,0, 0), tf::Vector3(1.085, 0, 1.565));
    Isometry3d goal_transform;
    Isometry3d start_transform;

    tf::transformTFToEigen(tf_start, start_transform);
    tf::transformTFToEigen(tf_goal, goal_transform);

    //Check first and last state on allowed collision
//    kt_kinematic_state.setFromIK(joint_model_group_ptr, goal_transform);
//    checkAllowedCollision(kt_kinematic_state, kt_planning_scene);
//    kt_kinematic_state.setFromIK(joint_model_group_ptr, start_transform);
//    checkAllowedCollision(kt_kinematic_state, kt_planning_scene);

    kt_kinematic_state.setFromIK(joint_model_group_ptr, goal_transform);
    RobotState goal_state(kt_kinematic_state);
    kt_kinematic_state.setFromIK(joint_model_group_ptr, start_transform);
    RobotState start_state(kt_kinematic_state);

    list<RobotState> trajectory;
    auto out = back_inserter(trajectory);
    LinearParams params = {joint_model_group_ptr, joint_model_group_ptr->getLinkModel(FANUC_M20IA_END_EFFECTOR), STANDARD_INTERPOLATION_STEP};
    TestIKSolver solver;
    size_t approximate_steps = floor(getMaxTranslation(start_state, goal_state) / STANDARD_INTERPOLATION_STEP);
    PoseAndStateInterpolator interpolator(tf_start, tf_goal, start_state, goal_state);
    linearInterpolationTemplate(params, kt_kinematic_state, interpolator, solver, (++approximate_steps), out);
    for (auto state_it = trajectory.begin(); state_it != prev(trajectory.end()); ++state_it)
    {
        auto insert_it = inserter(trajectory, next(state_it));
        if (getMaxTranslation(*state_it, *next(state_it)) > LINEAR_TARGET_PRECISION)
            advance(state_it, splitTrajectoryTemplate(insert_it, interpolator, params, *state_it, *next(state_it)));
    }

    for (auto it = trajectory.begin(); it != trajectory.end(); ++it){
        this_thread::sleep_for(chrono::milliseconds(10));
        visual_tools.publishRobotState(*it);
        this_thread::sleep_for(chrono::milliseconds(10));
        visual_tools.deleteAllMarkers();
    }
}