#include "mpc_local_planner/mpc_local_planner.hpp"
#include <pluginlib/class_list_macros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <angles/angles.h>

#include <algorithm>
#include <cmath>
#include <limits>

PLUGINLIB_EXPORT_CLASS(mpc_local_planner::MpcLocalPlanner, nav_core::BaseLocalPlanner)

namespace mpc_local_planner
{

void MpcLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
    if (initialized_)
    {
        ROS_WARN("MpcLocalPlanner has already been initialized, doing nothing.");
        return;
    }

    tf_ = tf;
    costmap_ros_ = costmap_ros;

    // move_base 可能传入完整类名 "mpc_local_planner/MpcLocalPlanner"，
    // 也可能只传入类名 "MpcLocalPlanner"；统一使用类名作为参数命名空间
    std::string planner_name = name;
    size_t slash_pos = planner_name.find_last_of('/');
    if (slash_pos != std::string::npos)
    {
        planner_name = planner_name.substr(slash_pos + 1);
    }
    ros::NodeHandle private_nh("~" + planner_name);
    ros::NodeHandle nh;

    loadParameters(private_nh);
    initializeRosInterfaces(nh);
    initializeMpc();

    initialized_ = true;
    ROS_INFO("MpcLocalPlanner initialized. wheel_radius=%.3f, max_speed=%.3f, L=%.3f, W=%.3f",
             wheel_radius_, max_speed_, param_.L, param_.W);
}

void MpcLocalPlanner::loadParameters(const ros::NodeHandle& private_nh)
{
    private_nh.param("wheel_radius", wheel_radius_, 0.15);
    private_nh.param("max_speed", max_speed_, 0.5);
    private_nh.param("target_speed", target_speed_, 0.5);
    private_nh.param("max_wheel_speed", param_.control_limits.max_wheel_speed, max_speed_);
    private_nh.param("max_wheel_acceleration", param_.control_limits.max_wheel_acceleration, 1.0);
    private_nh.param("max_steering_angle", param_.control_limits.max_steering_angle, 0.8726646259971648);
    private_nh.param("max_steering_rate", param_.control_limits.max_steering_rate, 0.5);
    private_nh.param("transform_timeout", transform_timeout_, 0.05);
    private_nh.param("forward_window", forward_window_, 80);
    private_nh.param("back_buffer", back_buffer_, 10);
    private_nh.param("path_smooth_window", path_smooth_window_, 1);
    private_nh.param("terminal_path_yaw_lookback", terminal_path_yaw_lookback_, 0.2);
    std::string goal_yaw_mode;
    private_nh.param<std::string>("goal_yaw_mode", goal_yaw_mode, std::string("auto"));
    private_nh.param("goal_xy_tolerance", goal_xy_tolerance_, 0.10);
    private_nh.param("goal_yaw_tolerance", goal_yaw_tolerance_, 0.05);
    private_nh.param("goal_align_xy_abort_tolerance", goal_align_xy_abort_tolerance_, 0.20);
    private_nh.param("align_yaw_threshold", align_yaw_threshold_, M_PI / 3.0);
    private_nh.param("align_yaw_exit_threshold", align_yaw_exit_threshold_, M_PI / 18.0);
    private_nh.param("align_recenter_max_step", align_recenter_max_step_, 0.05);
    private_nh.param("align_recenter_tolerance", align_recenter_tolerance_, 0.01);
    private_nh.param("align_max_omega", align_max_omega_, 0.5);
    private_nh.param("align_kp", align_kp_, 1.0);

    if (!areControlLimitsValid(param_.control_limits))
    {
        ROS_ERROR("MpcLocalPlanner received invalid control limits; using conservative defaults.");
        param_.control_limits = ControlLimits();
    }
    if (!std::isfinite(max_speed_) || max_speed_ <= 0.0)
    {
        max_speed_ = param_.control_limits.max_wheel_speed;
    }
    max_speed_ = std::min(max_speed_, param_.control_limits.max_wheel_speed);
    if (!std::isfinite(target_speed_) || target_speed_ <= 0.0)
    {
        target_speed_ = max_speed_;
    }
    target_speed_ = std::min(target_speed_, max_speed_);

    if (path_smooth_window_ < 1 || path_smooth_window_ > 9)
    {
        const int requested_window = path_smooth_window_;
        path_smooth_window_ = std::clamp(path_smooth_window_, 1, 9);
        ROS_WARN("MpcLocalPlanner: path_smooth_window=%d is outside [1, 9]; using %d.",
                 requested_window, path_smooth_window_);
    }
    if (!std::isfinite(terminal_path_yaw_lookback_) || terminal_path_yaw_lookback_ <= 0.0)
    {
        terminal_path_yaw_lookback_ = 0.2;
    }
    if (!parseGoalYawMode(goal_yaw_mode, goal_yaw_mode_))
    {
        ROS_WARN("MpcLocalPlanner: unknown goal_yaw_mode='%s'; using 'auto'. "
                 "Valid values are auto, pose_orientation, and path_tangent.",
                 goal_yaw_mode.c_str());
        goal_yaw_mode_ = GoalYawMode::AUTO;
    }
    if (!std::isfinite(goal_xy_tolerance_) || goal_xy_tolerance_ <= 0.0)
    {
        goal_xy_tolerance_ = 0.1;
    }
    if (!std::isfinite(goal_yaw_tolerance_) || goal_yaw_tolerance_ <= 0.0)
    {
        goal_yaw_tolerance_ = 0.2;
    }
    if (!std::isfinite(goal_align_xy_abort_tolerance_) ||
        goal_align_xy_abort_tolerance_ < goal_xy_tolerance_)
    {
        goal_align_xy_abort_tolerance_ = 2.0 * goal_xy_tolerance_;
    }

    // MPC parameters from diffmpc.hpp defaults, allow override
    private_nh.param("L", param_.L, 0.4615);
    private_nh.param("W", param_.W, 0.4);
    private_nh.param("L_front", L_front_, 0.4615);
    private_nh.param("L_rear", L_rear_, 0.4725);
    param_.dt = NMPC_DT;
}

void MpcLocalPlanner::initializeRosInterfaces(ros::NodeHandle& nh)
{
    // 订阅 /odom，作为位姿获取失败时的备用数据
    odom_sub_ = nh.subscribe("/odom", 1, &MpcLocalPlanner::odomCallback, this);

    // 发布 8 个底盘控制话题
    pub_whell_front_L_ = nh.advertise<std_msgs::Float64>("/smart/front_left_svelocity_controller/command", 1);
    pub_steer_front_L_ = nh.advertise<std_msgs::Float64>("/smart/front_left_str_controller/command", 1);
    pub_whell_front_R_ = nh.advertise<std_msgs::Float64>("/smart/front_right_velocity_controller/command", 1);
    pub_steer_front_R_ = nh.advertise<std_msgs::Float64>("/smart/front_right_str_controller/command", 1);
    pub_whell_rear_L_  = nh.advertise<std_msgs::Float64>("/smart/rear_left_velocity_controller/command", 1);
    pub_steer_rear_L_  = nh.advertise<std_msgs::Float64>("/smart/rear_left_str_controller/command", 1);
    pub_whell_rear_R_  = nh.advertise<std_msgs::Float64>("/smart/rear_right_velocity_controller/command", 1);
    pub_steer_rear_R_  = nh.advertise<std_msgs::Float64>("/smart/rear_right_str_controller/command", 1);
}

void MpcLocalPlanner::initializeMpc()
{
    mpc_.reset(new diffMpcController());
    last_control_command_ = Eigen::VectorXd::Zero(NMPC_NU);
    has_last_control_command_ = false;
}

bool MpcLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
    if (!initialized_)      // 查看初始化是否成功
    {
        ROS_ERROR("MpcLocalPlanner has not been initialized, please call initialize() before using this planner.");
        return false;
    }
    // 检查路径是否为空
    if (plan.empty())
    {
        ROS_WARN("Received empty plan.");
        return false;
    }
    // 将全局规划器的路径统一转换到同一个坐标系下，方便后续计算 确定路径坐标系
    const std::string plan_frame = plan.front().header.frame_id;
    if (plan_frame.empty())
    {
        ROS_ERROR("MpcLocalPlanner received a plan without a frame_id.");
        return false;
    }

    const auto& original_goal_orientation = plan.back().pose.orientation;
    const bool goal_orientation_components_finite =
        std::isfinite(original_goal_orientation.x) &&
        std::isfinite(original_goal_orientation.y) &&
        std::isfinite(original_goal_orientation.z) &&
        std::isfinite(original_goal_orientation.w);
    const double original_goal_orientation_norm_squared =
        original_goal_orientation.x * original_goal_orientation.x +
        original_goal_orientation.y * original_goal_orientation.y +
        original_goal_orientation.z * original_goal_orientation.z +
        original_goal_orientation.w * original_goal_orientation.w;
    if (!goal_orientation_components_finite ||
        !std::isfinite(original_goal_orientation_norm_squared))
    {
        ROS_WARN("MpcLocalPlanner received an invalid goal orientation; it will be "
                 "treated as unavailable according to goal_yaw_mode=%s.",
                 goalYawModeName(goal_yaw_mode_));
    }
    // geometry_msgs 默认构造出的四元数全为 0，可将它明确识别为“未提供方向”。
    const bool goal_orientation_provided =
        goal_orientation_components_finite &&
        std::isfinite(original_goal_orientation_norm_squared) &&
        original_goal_orientation_norm_squared > 1e-12;

    // 统一所有路径点的坐标系
    std::vector<geometry_msgs::PoseStamped> normalized_plan;
    if (!normalizePlanFrames(plan, plan_frame, normalized_plan))
    {
        return false;
    }

    // 读取真实目标点
    const auto& normalized_goal = normalized_plan.back().pose;
    std::vector<double> terminal_path_x;
    std::vector<double> terminal_path_y;
    terminal_path_x.reserve(normalized_plan.size());
    terminal_path_y.reserve(normalized_plan.size());
    for (size_t i = 0; i < normalized_plan.size(); ++i)
    {
        const double x = normalized_plan[i].pose.position.x;
        const double y = normalized_plan[i].pose.position.y;
        if (!std::isfinite(x) || !std::isfinite(y))
        {
            ROS_ERROR("MpcLocalPlanner received a non-finite position at plan pose %zu.", i);
            return false;
        }
        terminal_path_x.push_back(x);
        terminal_path_y.push_back(y);
    }

    // 与实际 MPC 参考路径使用相同的平滑规则，再计算最终停车方向。
    std::vector<double> smoothed_terminal_x =
        movingAverage(terminal_path_x, path_smooth_window_);
    std::vector<double> smoothed_terminal_y =
        movingAverage(terminal_path_y, path_smooth_window_);
    smoothed_terminal_x.front() = terminal_path_x.front();
    smoothed_terminal_x.back() = terminal_path_x.back();
    smoothed_terminal_y.front() = terminal_path_y.front();
    smoothed_terminal_y.back() = terminal_path_y.back();

    double terminal_path_yaw = 0.0;
    const bool terminal_path_yaw_valid = tryCalculateTerminalPathYaw(
        smoothed_terminal_x, smoothed_terminal_y,
        terminal_path_yaw_lookback_, terminal_path_yaw);

    double requested_goal_yaw = std::numeric_limits<double>::quiet_NaN();
    if (goal_orientation_provided)
    {
        requested_goal_yaw = getYaw(normalized_goal.orientation);
    }
    if (goal_orientation_provided && !std::isfinite(requested_goal_yaw))
    {
        ROS_WARN("MpcLocalPlanner could not extract a valid yaw from the transformed "
                 "goal orientation; it will be treated as unavailable.");
    }

    const GoalYawSelection selected_goal_yaw = selectGoalYaw(
        goal_yaw_mode_, requested_goal_yaw,
        terminal_path_yaw_valid, terminal_path_yaw);
    if (!selected_goal_yaw.valid)
    {
        if (goal_yaw_mode_ == GoalYawMode::POSE_ORIENTATION)
        {
            ROS_ERROR("MpcLocalPlanner: goal_yaw_mode=pose_orientation, but the final "
                      "path pose has no valid orientation.");
        }
        else if (goal_yaw_mode_ == GoalYawMode::PATH_TANGENT)
        {
            ROS_ERROR("MpcLocalPlanner: goal_yaw_mode=path_tangent requires at least "
                      "two distinct finite XY path points.");
        }
        else
        {
            ROS_ERROR("MpcLocalPlanner cannot determine the goal yaw: the final pose has "
                      "no valid orientation and the path has no terminal direction.");
        }
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    // 保存旧目标和旧状态
    const GoalPose2D previous_goal = goal_;
    const bool had_goal = has_goal_;
    const State previous_state = state_;
    const std::string previous_plan_frame = plan_frame_;
    plan_frame_ = plan_frame;   // 将路径坐标系存储为成员变量
    convertPlanToReference(normalized_plan);   // 把统一坐标系下的 ROS 路径转成 MPC 参考轨迹
    // 独立保存目标姿态
    goal_.x = normalized_goal.position.x;
    goal_.y = normalized_goal.position.y;
    goal_.yaw = angles::normalize_angle(selected_goal_yaw.yaw);
    goal_yaw_from_pose_ = selected_goal_yaw.used_pose_orientation;
    has_goal_ = true;
    const double same_goal_xy_tolerance =
        std::max(1e-4, std::min(0.01, 0.1 * goal_xy_tolerance_));
    const double same_goal_yaw_tolerance =
        std::max(1e-4, std::min(0.02, 0.1 * goal_yaw_tolerance_));
    const bool same_goal = had_goal && previous_plan_frame == plan_frame_ &&
        std::hypot(previous_goal.x - goal_.x, previous_goal.y - goal_.y) <=
            same_goal_xy_tolerance &&
        std::fabs(angles::shortest_angular_distance(previous_goal.yaw, goal_.yaw)) <=
            same_goal_yaw_tolerance;
    last_min_index_ = 0;    // 重置最近点索引
    has_plan_ = true;   
    if (same_goal)
    {
        state_ = previous_state;
    }
    else
    {
        updateStateForNewPlan();
    }

    // 调用 mpc_->reset()，重置 MPC 内部上一拍控制量
    mpc_->reset();
    if (has_last_control_command_ && last_control_command_.size() == NMPC_NU)
    {
        mpc_->U = last_control_command_;
    }

    const double logged_terminal_path_yaw = terminal_path_yaw_valid
        ? terminal_path_yaw
        : std::numeric_limits<double>::quiet_NaN();
    ROS_INFO("MpcLocalPlanner received plan with %zu points in frame '%s' "
             "(smooth_window=%d, goal_yaw_mode=%s, goal_yaw_source=%s, "
             "goal_yaw=%.3f, terminal_path_yaw=%.3f).",
             plan.size(), plan_frame_.c_str(), path_smooth_window_,
             goalYawModeName(goal_yaw_mode_),
             goal_yaw_from_pose_ ? "pose_orientation" : "path_tangent",
             goal_.yaw, logged_terminal_path_yaw);
    return true;
}

bool MpcLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    std::lock_guard<std::mutex> lock(mutex_);
    // 检查初始化状态
    if (!initialized_)
    {
        ROS_ERROR("MpcLocalPlanner has not been initialized.");
        return false;
    }
    // 检查是否已经收到路径
    if (!has_plan_)
    {
        ROS_WARN_THROTTLE(5.0, "MpcLocalPlanner has no plan yet.");
        publishZeroCommands();
        return false;
    }

    // 获取小车当前位置和航向
    Eigen::Vector3d initial_x;
    if (!getRobotPose(initial_x))
    {
        ROS_WARN_THROTTLE(5.0, "Failed to get robot pose from costmap.");
        publishZeroCommands();
        return false;
    }

    const GoalEvaluation goal = evaluateCurrentGoal(initial_x);
    if (state_ == State::GOAL_HOLD)
    {
        if (goal.reached())
        {
            return startGoalRecenter(initial_x, cmd_vel);
        }
        state_ = State::TRACKING;
    }
    if (state_ == State::GOAL_ALIGN_STEERING)
    {
        return prepareGoalAlignmentSteering(initial_x, cmd_vel);
    }
    if (state_ == State::GOAL_ALIGNING)
    {
        return handleGoalAlignment(initial_x, cmd_vel);
    }
    if (state_ == State::GOAL_RECENTERING)
    {
        return recenterGoalSteering(initial_x, cmd_vel);
    }
    if (goal.valid && goal.position_reached)
    {
        if (goal.yaw_reached)
        {
            return startGoalRecenter(initial_x, cmd_vel);
        }

        state_ = State::GOAL_ALIGN_STEERING;
        ROS_INFO("MpcLocalPlanner: goal position reached; prepare final yaw alignment (error %.2f deg).",
                 goal.yaw_error * 180.0 / M_PI);
        return prepareGoalAlignmentSteering(initial_x, cmd_vel);
    }

    if (state_ == State::RECENTERING)
    {
        return recenterSteering(cmd_vel);
    }
    // 如果处于 ALIGN_STEERING，先缓慢转舵到原地旋转角度。
    if (state_ == State::ALIGN_STEERING)
    {
        return prepareAlignmentSteering(initial_x, cmd_vel);
    }

    // 如果处于 ALIGNING，先原地旋转；完成后先 RECENTERING 回正转角，再进入 TRACKING
    if (state_ == State::ALIGNING)
    {
        return handleAlignment(initial_x, cmd_vel);
    }

    // 找当前机器人在参考路径上的最近点
    auto [min_index, min_e] = calcForwardNearestIndex(initial_x(0), initial_x(1));

    // 如果横向误差 min_e > 1.0，代码会用当前参考点的速度和曲率构造一个近似的参考轮速/转角，写入 mpc_->U
    // 给 MPC 一个更合理的初值，避免偏离路径太远时优化器从奇怪的上一拍控制量开始算
    resetControlToReference(min_index, min_e);

    // Solve MPC
    Eigen::VectorXd U_solve;
    if (!solveMpcCommand(initial_x, min_index, U_solve))
    {
        return false;
    }
    // std::cout << "v1 = " << U_solve(0) << std::endl;
    // Publish wheel commands
    if (!publishWheelCommands(U_solve))
    {
        publishZeroCommands();
        return false;
    }

    // 返回给 move_base 的 cmd_vel
    computeEquivalentTwist(U_solve, cmd_vel);

    return true;
}

bool MpcLocalPlanner::checkGoalReached(const Eigen::Vector3d& current_state) const
{
    if (!initialized_ || !has_plan_ || !has_goal_)
    {
        return false;
    }

    const GoalEvaluation goal = evaluateCurrentGoal(current_state);
    const bool controls_settled = has_last_control_command_ &&
        last_control_command_.size() == NMPC_NU &&
        last_control_command_.head(4).cwiseAbs().maxCoeff() <= 1e-3 &&
        last_control_command_.tail(4).cwiseAbs().maxCoeff() <= 1e-6;
    const bool steering_recentered = state_ == State::GOAL_HOLD && controls_settled;
    if (canReportGoalReached(goal, steering_recentered))
    {
        ROS_INFO("MpcLocalPlanner: reached goal");
        return true;
    }
    if (goal.reached() && !steering_recentered)
    {
        ROS_INFO_THROTTLE(
            1.0,
            "MpcLocalPlanner: goal pose reached; waiting for steering to recenter before reporting success.");
    }
    if (goal.valid && goal.position_reached && !goal.yaw_reached)
    {
        const double path_yaw = ryaw_.empty() ? 0.0 : angles::normalize_angle(ryaw_.back());
        ROS_WARN_THROTTLE(
            0.5,
            "goal position reached but yaw is outside tolerance: "
            "dist=%.3f/%.3f yaw_err=%.3f/%.3f goal_yaw=%.3f path_yaw=%.3f current_yaw=%.3f",
            goal.distance, goal_xy_tolerance_, std::fabs(goal.yaw_error),
            goal_yaw_tolerance_, goal_.yaw, path_yaw, current_state(2));
    }
    return false;
}

bool MpcLocalPlanner::isGoalReached()
{
    std::lock_guard<std::mutex> lock(mutex_);

    Eigen::Vector3d current_state;
    if (!getRobotPose(current_state))
    {
        return false;
    }

    return checkGoalReached(current_state);
}

void MpcLocalPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    latest_odom_pose_.header = msg->header;
    latest_odom_pose_.pose = msg->pose.pose;
    has_odom_ = true;
}

double MpcLocalPlanner::getYaw(const geometry_msgs::Quaternion& q) const
{
    const double norm_squared = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (!std::isfinite(norm_squared) || norm_squared <= 1e-12)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    quat.normalize();
    tf2::Matrix3x3 m(quat);
    double r, p, y;
    m.getRPY(r, p, y);
    return y;
}

bool MpcLocalPlanner::getRobotPose(Eigen::Vector3d& state) const
{
    geometry_msgs::PoseStamped robot_pose;  // 声明一个带坐标系和时间戳的位姿
    if (!costmap_ros_->getRobotPose(robot_pose))    // 如果 costmap 获取失败，回退到最近一帧 /odom
    {
        if (!has_odom_)
        {
            return false;
        }
        robot_pose = latest_odom_pose_;
    }
    // 将机器人位姿转换到全局路径坐标
    geometry_msgs::PoseStamped plan_pose;
    if (!transformPoseToPlanFrame(robot_pose, plan_pose))
    {
        return false;
    }

    const double yaw = getYaw(plan_pose.pose.orientation);
    if (!std::isfinite(plan_pose.pose.position.x) ||
        !std::isfinite(plan_pose.pose.position.y) || !std::isfinite(yaw))
    {
        ROS_WARN_THROTTLE(1.0, "MpcLocalPlanner received a non-finite robot pose.");
        return false;
    }

    state << plan_pose.pose.position.x,
             plan_pose.pose.position.y,
             yaw;
    return true;
}

bool MpcLocalPlanner::transformPoseToPlanFrame(
    const geometry_msgs::PoseStamped& source_pose,
    geometry_msgs::PoseStamped& plan_pose) const
{
    if (source_pose.header.frame_id.empty())
    {
        ROS_ERROR_THROTTLE(1.0, "MpcLocalPlanner cannot transform a pose without a frame_id.");
        return false;
    }

    if (plan_frame_.empty())
    {
        ROS_ERROR_THROTTLE(1.0, "MpcLocalPlanner has no plan frame for pose transformation.");
        return false;
    }

    if (source_pose.header.frame_id == plan_frame_)
    {
        plan_pose = source_pose;
        return true;
    }

    if (tf_ == nullptr)
    {
        ROS_ERROR_THROTTLE(1.0, "MpcLocalPlanner has no TF buffer for pose transformation.");
        return false;
    }

    try
    {
        tf_->transform(source_pose, plan_pose, plan_frame_, ros::Duration(transform_timeout_));
    }
    catch (const tf2::TransformException& ex)
    {
        ROS_WARN_THROTTLE(1.0,
                          "MpcLocalPlanner failed to transform robot pose from '%s' to '%s': %s",
                          source_pose.header.frame_id.c_str(), plan_frame_.c_str(), ex.what());
        return false;
    }

    return true;
}

bool MpcLocalPlanner::normalizePlanFrames(
    const std::vector<geometry_msgs::PoseStamped>& plan,
    const std::string& plan_frame,
    std::vector<geometry_msgs::PoseStamped>& normalized_plan) const
{
    normalized_plan.clear();
    normalized_plan.reserve(plan.size());

    for (size_t i = 0; i < plan.size(); ++i)
    {
        geometry_msgs::PoseStamped source_pose = plan[i];
        if (source_pose.header.frame_id.empty())
        {
            ROS_ERROR("MpcLocalPlanner plan pose %zu has no frame_id.", i);
            return false;
        }

        // TF2 cannot transform a pose containing a zero/invalid quaternion.
        // Orientation is ignored for path samples, and setPlan() separately
        // remembers whether the original terminal pose supplied a valid yaw.
        if (!std::isfinite(getYaw(source_pose.pose.orientation)))
        {
            source_pose.pose.orientation.x = 0.0;
            source_pose.pose.orientation.y = 0.0;
            source_pose.pose.orientation.z = 0.0;
            source_pose.pose.orientation.w = 1.0;
        }

        if (source_pose.header.frame_id == plan_frame)
        {
            normalized_plan.push_back(std::move(source_pose));
            continue;
        }

        if (tf_ == nullptr)
        {
            ROS_ERROR("MpcLocalPlanner has no TF buffer to normalize plan frames.");
            return false;
        }

        geometry_msgs::PoseStamped transformed_pose;
        try
        {
            tf_->transform(source_pose, transformed_pose, plan_frame,
                           ros::Duration(transform_timeout_));
        }
        catch (const tf2::TransformException& ex)
        {
            ROS_ERROR("MpcLocalPlanner failed to transform plan pose from '%s' to '%s': %s",
                      source_pose.header.frame_id.c_str(), plan_frame.c_str(), ex.what());
            return false;
        }
        normalized_plan.push_back(std::move(transformed_pose));
    }

    return true;
}

// 将接收到的全局路径转换为参考轨迹，包括位置、航向、曲率和速度
void MpcLocalPlanner::convertPlanToReference(const std::vector<geometry_msgs::PoseStamped>& plan)
{
    // 先清空之前的参考轨迹数据
    r_x_.clear();
    r_y_.clear();
    ryaw_.clear();
    rcurvature_.clear();
    speed_profile_.clear();
    // 计算参考轨迹的平滑路径、航向角、曲率和速度
    const int speed_smooth_window = 5;
    const int curvature_smooth_window = 5;
    const double stop_distance = 0.4;

    setReferencePositions(plan);    // 这里传入的 plan 已经是统一坐标系下的 ROS 路径
    smoothReferencePath(path_smooth_window_);
    updateReferenceYaw();
    updateReferenceCurvature(curvature_smooth_window);
    updateSpeedProfile(speed_smooth_window);
    applyStopProfile(stop_distance);
}

void MpcLocalPlanner::setReferencePositions(const std::vector<geometry_msgs::PoseStamped>& plan)
{
    // 简单的将 ROS 路径的 x,y 坐标提取到 r_x_ 和 r_y_ 中
    for (const auto& pose : plan)
    {
        r_x_.push_back(pose.pose.position.x);
        r_y_.push_back(pose.pose.position.y);
    }
}

void MpcLocalPlanner::smoothReferencePath(int window)
{
    // 先对路径坐标做滑动平均平滑，降低全局规划器稀疏路点造成的抖动
    // 但保持首尾点不变，避免终点被拖离原始目标位置
    std::vector<double> sx = movingAverage(r_x_, window);
    std::vector<double> sy = movingAverage(r_y_, window);
    // 将首尾点恢复为原始路径的首尾点，避免平滑后偏离目标
    if (!r_x_.empty())
    {
        sx.front() = r_x_.front();
        sx.back()  = r_x_.back();
    }
    if (!r_y_.empty())
    {
        sy.front() = r_y_.front();
        sy.back()  = r_y_.back();
    }
    // 将平滑后的路径赋值回 r_x_ 和 r_y_
    r_x_ = std::move(sx);
    r_y_ = std::move(sy);
}

void MpcLocalPlanner::updateReferenceYaw()
{
    // 从平滑后的坐标重新计算航向角（不依赖 pose.orientation，兼容性更好）
    ryaw_.resize(r_x_.size(), 0.0);
    // 计算航向角时，使用前后点的差分来估计切线方向，避免使用当前点和下一个点的差分，这样可以更平滑
    for (size_t i = 0; i < r_x_.size(); ++i)
    {
        size_t ip1 = std::min(i + 1, r_x_.size() - 1);  // 下一个点索引，防止越界
        size_t im1 = (i == 0) ? 0 : i - 1;  // 上一个点索引，首点时使用自身
        // dx 和 dy 实际上构成了当前点所在位置的近似切线向量
        const double dx = r_x_[ip1] - r_x_[im1];
        const double dy = r_y_[ip1] - r_y_[im1];
        if (std::hypot(dx, dy) > 1e-6)
        {
            ryaw_[i] = std::atan2(dy, dx);
        }
        else if (i > 0)
        {
            ryaw_[i] = ryaw_[i - 1];
        }
    }

    // The exact goal appended by a grid planner can create a very short or
    // reversed final segment. Estimate the terminal path tangent over a fixed
    // physical distance so it is independent of the smoothing window.
    if (!ryaw_.empty())
    {
        ryaw_.back() = calculateTerminalPathYaw(
            r_x_, r_y_, terminal_path_yaw_lookback_, ryaw_.back());
    }

    // 展开航向角，避免曲率计算时出现正负 pi 跳变
    mpc_->smooth_yaw(ryaw_);
}

void MpcLocalPlanner::updateReferenceCurvature(int smoothing_window)
{
    // 计算曲率
    rcurvature_.resize(r_x_.size(), 0.0);
    // 计算某一个点的曲率，至少需要知道它的“前一个点”和“后一个点”。因此，路径的第一个点（没有前一个点）
    // 和最后一个点（没有后一个点）无法直接计算，代码选择直接赋予它们 0.0（即视为直道），然后跳过当前循环
    for (size_t i = 0; i < r_x_.size(); ++i)
    {
        if (i == 0 || i + 1 >= r_x_.size())
        {
            rcurvature_[i] = 0.0;
            continue;
        }
        // 计算的是相邻点之间的距离 ds1 是当前点到下一个点的距离    ds0 是前一个点到当前点的距离
        // ds 是这两段距离的平均值  用这个平均值 ds 来近似代替微分几何中的微元弧长
        size_t ip1 = i + 1;
        size_t im1 = i - 1;
        double dx1 = r_x_[ip1] - r_x_[i];
        double dy1 = r_y_[ip1] - r_y_[i];
        double dx0 = r_x_[i] - r_x_[im1];
        double dy0 = r_y_[i] - r_y_[im1];
        double ds1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
        double ds0 = std::sqrt(dx0 * dx0 + dy0 * dy0);
        double ds = 0.5 * (ds1 + ds0);
        // 用到了上一轮（updateReferenceYaw）计算出来的航向角 ryaw_
        double dyaw = ryaw_[ip1] - ryaw_[im1];
        dyaw = angles::normalize_angle(dyaw);
        // 曲率公式：kappa = dtheta / ds
        // 安全保护 (ds > 1e-6)： 如果两个路径点重合（距离极小，接近 0），
        // 作为分母会导致“除以零”的严重崩溃。因此加上条件判断，如果距离太小，直接把曲率设为 0
        rcurvature_[i] = (ds > 1e-6) ? (dyaw / ds) : 0.0;
    }
    // 平滑曲率，避免速度曲线剧烈抖动
    rcurvature_ = movingAverage(rcurvature_, smoothing_window);
}

void MpcLocalPlanner::updateSpeedProfile(int smoothing_window)
{
    // 计算参考速度，考虑曲率和最大速度约束
    // rcurvature_是上一步中计算出来的曲率，target_speed_是用户在参数中设置的目标速度
    speed_profile_ = mpc_->calculateReferenceSpeeds(rcurvature_, target_speed_);

    // 把速度钳制在 [min_speed, max_speed] 范围内
    const double min_speed = 0.15;
    for (auto& v : speed_profile_)
    {
        v = std::max(min_speed, std::min(max_speed_, v));
    }
    // 对速度曲线做滑动平均平滑，避免急弯处速度骤降导致 MPC 急刹停住
    // 消除相邻路径点速度变化过大的问题
    // 例如 smoothing_window=5 时，一个速度点会参考它附近最多 5 个点的平均值，使速度曲线更加连续
    speed_profile_ = movingAverage(speed_profile_, smoothing_window);
}

void MpcLocalPlanner::applyStopProfile(double stop_distance)
{
    // 终点前减速到 0，避免到了终点还在以 min_speed 硬冲
    std::vector<double> remaining(speed_profile_.size(), 0.0);
    for (int i = static_cast<int>(speed_profile_.size()) - 2; i >= 0; --i)
    {
        double dx = r_x_[i + 1] - r_x_[i];
        double dy = r_y_[i + 1] - r_y_[i];
        remaining[i] = remaining[i + 1] + std::sqrt(dx * dx + dy * dy);
    }
    for (size_t i = 0; i < speed_profile_.size(); ++i)
    {
        if (remaining[i] < stop_distance)
        {
            double ratio = remaining[i] / stop_distance;
            speed_profile_[i] *= std::max(0.0, ratio);
        }
    }

    // 最终钳制到 [0, max_speed]
    for (auto& v : speed_profile_)
    {
        v = std::max(0.0, std::min(max_speed_, v));
    }
}

std::tuple<int, double> MpcLocalPlanner::calcForwardNearestIndex(double current_x, double current_y)
{
    // 在上一次最近点附近找最近点
    int n = r_x_.size();
    int start_idx = std::max(0, last_min_index_ - back_buffer_);
    int end_idx = std::min(n, last_min_index_ + forward_window_);

    double mind = std::numeric_limits<double>::max();
    int min_index = start_idx;

    for (int i = start_idx; i < end_idx; i++)
    {
        double idx = current_x - r_x_[i];
        double idy = current_y - r_y_[i];
        double d_e = std::sqrt(idx * idx + idy * idy);

        if (d_e < mind)
        {
            mind = d_e;
            min_index = i;
        }
    }

    // If forward window cannot find a reasonable point, fall back to global search
    if (mind > 5.0 && start_idx > 0)
    {
        return mpc_->calc_ref_trajectory(current_x, current_y, r_x_, r_y_);
    }

    last_min_index_ = min_index;
    return std::make_tuple(min_index, mind);
}

void MpcLocalPlanner::updateStateForNewPlan()
{
    // 根据当前车头朝向和路径起点航向决定是否先原地对准。
    Eigen::Vector3d current_state;
    if (!getRobotPose(current_state) || ryaw_.empty())
    {
        state_ = State::TRACKING;
        return;
    }

    const double heading_error = angles::shortest_angular_distance(current_state(2), ryaw_[0]);
    if (std::fabs(heading_error) > align_yaw_threshold_)
    {
        state_ = State::ALIGN_STEERING;
        ROS_INFO("MpcLocalPlanner: initial heading error %.2f deg > %.2f deg, prepare alignment steering first.",
                 heading_error * 180.0 / M_PI, align_yaw_threshold_ * 180.0 / M_PI);
    }
    else if (has_last_steer_cmd_ &&
             last_steer_cmd_.cwiseAbs().maxCoeff() > 1e-6)
    {
        align_recenter_steer_ = last_steer_cmd_;
        state_ = State::RECENTERING;
        ROS_INFO("MpcLocalPlanner: new goal received with non-zero steering; recenter before tracking.");
    }
    else
    {
        state_ = State::TRACKING;
    }
}

bool MpcLocalPlanner::prepareAlignmentSteering(const Eigen::Vector3d& current_state, geometry_msgs::Twist& cmd_vel)
{
    if (ryaw_.empty())
    {
        publishZeroCommands();
        return false;
    }
    // 如果当前航向已经接近目标航向，则直接进入 RECENTERING 状态，避免不必要的原地旋转
    const double heading_error = angles::shortest_angular_distance(current_state(2), ryaw_[0]);
    if (std::fabs(heading_error) <= align_yaw_exit_threshold_)
    {
        state_ = State::RECENTERING;
        ROS_INFO("MpcLocalPlanner: alignment skipped, heading error %.2f deg <= %.2f deg, recenter steering.",
                 heading_error * 180.0 / M_PI, align_yaw_exit_threshold_ * 180.0 / M_PI);
        return recenterSteering(cmd_vel);
    }
    // 原地旋转的舵角构型与随后选择的旋转方向无关
    const Eigen::Vector4d target_steer = makeInPlaceSteeringTarget();
    Eigen::Vector4d stepped_steer;
    // 将当前转角逐步调整到目标转角，避免一次性跳变
    const bool requested_steering_ready = stepSteeringToward(target_steer, stepped_steer);
    // 将逐步调整后的转角写入控制量 U，轮速保持为 0
    Eigen::VectorXd U = Eigen::VectorXd::Zero(NMPC_NU);
    U(4) = stepped_steer(0);
    U(5) = stepped_steer(1);
    U(6) = stepped_steer(2);
    U(7) = stepped_steer(3);
    if (!publishWheelCommands(U))
    {
        publishZeroCommands();
        return false;
    }
    cmd_vel = geometry_msgs::Twist();
    // 检查当前转角是否已经接近目标转角，如果接近则可以进入 ALIGNING 状态，开始原地旋转
    const Eigen::Vector4d applied_steer(U(4), U(5), U(6), U(7));
    // cwiseAbs().maxCoeff() 表示取四个轮子中最大的转角误差。只有所有轮子的误差都不超过 0.01 rad，才认为准备完成
    const bool steering_ready = requested_steering_ready &&
        (target_steer - applied_steer).cwiseAbs().maxCoeff() <= align_recenter_tolerance_;
    // 如果转角已经准备好，则进入 ALIGNING 状态，开始原地旋转
    if (steering_ready)
    {
        align_recenter_steer_ = target_steer;
        state_ = State::ALIGNING;
        ROS_INFO("MpcLocalPlanner: alignment steering ready, start in-place rotation.");
    }
    else
    {
        ROS_INFO_THROTTLE(1.0,
                          "MpcLocalPlanner: preparing alignment steering, steer_cmds=[%.3f, %.3f, %.3f, %.3f]",
                          U(4), U(5), U(6), U(7));
    }

    return true;
}

bool MpcLocalPlanner::handleAlignment(const Eigen::Vector3d& current_state, geometry_msgs::Twist& cmd_vel)
{
    if (ryaw_.empty())
    {
        publishZeroCommands();
        return false;
    }

    const double heading_error = angles::shortest_angular_distance(current_state(2), ryaw_[0]);
    if (std::fabs(heading_error) <= align_yaw_exit_threshold_)
    {
        state_ = State::RECENTERING;
        ROS_INFO("MpcLocalPlanner: alignment done, heading error %.2f deg <= %.2f deg, recenter steering.",
                 heading_error * 180.0 / M_PI, align_yaw_exit_threshold_ * 180.0 / M_PI);
        return recenterSteering(cmd_vel);
    }

    double omega = align_kp_ * heading_error;
    omega = std::max(-align_max_omega_, std::min(align_max_omega_, omega));

    Eigen::VectorXd U = makeInPlaceRotationCommand(omega);
    if (!publishWheelCommands(U))
    {
        publishZeroCommands();
        return false;
    }
    align_recenter_steer_ << U(4), U(5), U(6), U(7);
    computeEquivalentTwist(U, cmd_vel);

    ROS_INFO_THROTTLE(1.0,
                      "MpcLocalPlanner: aligning, err=%.2f deg, omega=%.3f",
                      heading_error * 180.0 / M_PI, omega);
    return true;
}

GoalEvaluation MpcLocalPlanner::evaluateCurrentGoal(const Eigen::Vector3d& current_state) const
{
    if (!has_goal_)
    {
        return GoalEvaluation();
    }
    return evaluateGoal(current_state(0), current_state(1), current_state(2), goal_,
                        goal_xy_tolerance_, goal_yaw_tolerance_);
}

bool MpcLocalPlanner::holdGoal(geometry_msgs::Twist& cmd_vel)
{
    state_ = State::GOAL_HOLD;
    publishStoppedCommands();
    cmd_vel = geometry_msgs::Twist();
    return true;
}

bool MpcLocalPlanner::prepareGoalAlignmentSteering(
    const Eigen::Vector3d& current_state,
    geometry_msgs::Twist& cmd_vel)
{
    const GoalEvaluation goal = evaluateCurrentGoal(current_state);
    if (!goal.valid)
    {
        publishZeroCommands();
        return false;
    }
    if (goal.distance > goal_align_xy_abort_tolerance_)
    {
        align_recenter_steer_ = has_last_steer_cmd_
                                    ? last_steer_cmd_
                                    : Eigen::Vector4d::Zero();
        state_ = State::RECENTERING;
        ROS_WARN("MpcLocalPlanner: final alignment drifted %.3f m from goal; return to tracking.",
                 goal.distance);
        return recenterSteering(cmd_vel);
    }
    if (goal.yaw_reached)
    {
        if (goal.position_reached)
        {
            return startGoalRecenter(current_state, cmd_vel);
        }
        align_recenter_steer_ = has_last_steer_cmd_
                                    ? last_steer_cmd_
                                    : Eigen::Vector4d::Zero();
        state_ = State::RECENTERING;
        return recenterSteering(cmd_vel);
    }

    // Finish braking before changing the wheel angles. Otherwise the slew
    // limiter can leave non-zero wheel speeds while the steering moves toward
    // the in-place-rotation configuration, causing avoidable position drift.
    if (has_last_control_command_ && last_control_command_.size() == NMPC_NU &&
        last_control_command_.head(4).cwiseAbs().maxCoeff() > 1e-3)
    {
        Eigen::VectorXd U = last_control_command_;
        U.head(4).setZero();
        if (!publishWheelCommands(U))
        {
            publishZeroCommands();
            return false;
        }
        align_recenter_steer_ << U(4), U(5), U(6), U(7);
        computeEquivalentTwist(U, cmd_vel);
        ROS_INFO_THROTTLE(
            1.0,
            "MpcLocalPlanner: braking before final yaw alignment, wheel_speed_max=%.3f.",
            U.head(4).cwiseAbs().maxCoeff());
        return true;
    }

    const Eigen::Vector4d target_steer = makeInPlaceSteeringTarget();
    Eigen::Vector4d stepped_steer;
    const bool requested_steering_ready =
        stepSteeringToward(target_steer, stepped_steer);

    Eigen::VectorXd U = Eigen::VectorXd::Zero(NMPC_NU);
    U(4) = stepped_steer(0);
    U(5) = stepped_steer(1);
    U(6) = stepped_steer(2);
    U(7) = stepped_steer(3);
    if (!publishWheelCommands(U))
    {
        publishZeroCommands();
        return false;
    }

    align_recenter_steer_ << U(4), U(5), U(6), U(7);
    cmd_vel = geometry_msgs::Twist();
    const bool steering_ready = requested_steering_ready &&
        (target_steer - align_recenter_steer_).cwiseAbs().maxCoeff() <=
            align_recenter_tolerance_;
    if (steering_ready)
    {
        state_ = State::GOAL_ALIGNING;
        ROS_INFO("MpcLocalPlanner: final alignment steering ready, start in-place rotation.");
    }
    else
    {
        ROS_INFO_THROTTLE(
            1.0,
            "MpcLocalPlanner: preparing final alignment steering, "
            "steer_cmds=[%.3f, %.3f, %.3f, %.3f]",
            U(4), U(5), U(6), U(7));
    }
    return true;
}

bool MpcLocalPlanner::handleGoalAlignment(
    const Eigen::Vector3d& current_state,
    geometry_msgs::Twist& cmd_vel)
{
    const GoalEvaluation goal = evaluateCurrentGoal(current_state);
    if (!goal.valid)
    {
        publishZeroCommands();
        return false;
    }
    if (goal.distance > goal_align_xy_abort_tolerance_)
    {
        align_recenter_steer_ = has_last_steer_cmd_
                                    ? last_steer_cmd_
                                    : Eigen::Vector4d::Zero();
        state_ = State::RECENTERING;
        ROS_WARN("MpcLocalPlanner: final alignment drifted %.3f m from goal; return to tracking.",
                 goal.distance);
        return recenterSteering(cmd_vel);
    }
    if (goal.yaw_reached)
    {
        if (goal.position_reached)
        {
            ROS_INFO("MpcLocalPlanner: final yaw alignment complete.");
            return startGoalRecenter(current_state, cmd_vel);
        }
        align_recenter_steer_ = has_last_steer_cmd_
                                    ? last_steer_cmd_
                                    : Eigen::Vector4d::Zero();
        state_ = State::RECENTERING;
        return recenterSteering(cmd_vel);
    }

    double omega = align_kp_ * goal.yaw_error;
    omega = std::clamp(omega, -align_max_omega_, align_max_omega_);
    Eigen::VectorXd U = makeInPlaceRotationCommand(omega);
    if (!publishWheelCommands(U))
    {
        publishZeroCommands();
        return false;
    }

    align_recenter_steer_ << U(4), U(5), U(6), U(7);
    computeEquivalentTwist(U, cmd_vel);
    ROS_INFO_THROTTLE(1.0,
                      "MpcLocalPlanner: final yaw alignment, err=%.2f deg, omega=%.3f",
                      goal.yaw_error * 180.0 / M_PI, omega);
    return true;
}

bool MpcLocalPlanner::startGoalRecenter(
    const Eigen::Vector3d& current_state,
    geometry_msgs::Twist& cmd_vel)
{
    align_recenter_steer_ = has_last_steer_cmd_
                                ? last_steer_cmd_
                                : Eigen::Vector4d::Zero();
    const bool wheel_speeds_stopped =
        !has_last_control_command_ || last_control_command_.size() != NMPC_NU ||
        last_control_command_.head(4).cwiseAbs().maxCoeff() <= 1e-3;
    if (wheel_speeds_stopped &&
        align_recenter_steer_.cwiseAbs().maxCoeff() <= 1e-6)
    {
        align_recenter_steer_.setZero();
        return holdGoal(cmd_vel);
    }

    state_ = State::GOAL_RECENTERING;
    ROS_INFO("MpcLocalPlanner: goal pose reached; recenter steering before reporting success.");
    return recenterGoalSteering(current_state, cmd_vel);
}

bool MpcLocalPlanner::recenterGoalSteering(
    const Eigen::Vector3d& current_state,
    geometry_msgs::Twist& cmd_vel)
{
    const GoalEvaluation goal = evaluateCurrentGoal(current_state);
    if (!goal.valid || !goal.position_reached)
    {
        state_ = State::RECENTERING;
        if (goal.valid && goal.distance > goal_align_xy_abort_tolerance_)
        {
            ROS_WARN("MpcLocalPlanner: robot drifted %.3f m while recentering at goal; return to tracking.",
                     goal.distance);
        }
        return recenterSteering(cmd_vel);
    }
    if (!goal.yaw_reached)
    {
        state_ = State::GOAL_ALIGN_STEERING;
        ROS_WARN("MpcLocalPlanner: final yaw left tolerance while recentering; realign.");
        return prepareGoalAlignmentSteering(current_state, cmd_vel);
    }

    bool done = false;
    if (!applySteeringRecenterStep(cmd_vel, done))
    {
        return false;
    }

    if (!done)
    {
        ROS_INFO_THROTTLE(
            1.0,
            "MpcLocalPlanner: recentering steering at goal, "
            "steer_cmds=[%.3f, %.3f, %.3f, %.3f]",
            align_recenter_steer_(0), align_recenter_steer_(1),
            align_recenter_steer_(2), align_recenter_steer_(3));
        return true;
    }

    ROS_INFO("MpcLocalPlanner: steering recentered at goal.");
    return holdGoal(cmd_vel);
}

Eigen::VectorXd MpcLocalPlanner::makeInPlaceRotationCommand(double omega) const
{
    // In-place rotation using the same formula as four_wheeldrive.cpp.
    const double R = 0.0;
    const double W = param_.W;
    Eigen::VectorXd U(NMPC_NU);

    const double angle_fl = std::atan(L_front_ / (R - W));
    U(0) = omega * (L_front_ / std::sin(angle_fl));
    U(4) = angle_fl;

    const double angle_fr = std::atan(L_front_ / (R + W));
    U(3) = omega * (L_front_ / std::sin(angle_fr));
    U(7) = angle_fr;

    const double angle_rl = -std::atan(L_rear_ / (R - W));
    U(1) = -omega * (L_rear_ / std::sin(angle_rl));
    U(5) = angle_rl;

    const double angle_rr = -std::atan(L_rear_ / (R + W));
    U(2) = -omega * (L_rear_ / std::sin(angle_rr));
    U(6) = angle_rr;

    return U;
}

Eigen::Vector4d MpcLocalPlanner::makeInPlaceSteeringTarget() const
{
    const Eigen::VectorXd rotation = makeInPlaceRotationCommand(1.0);
    Eigen::Vector4d target(rotation(4), rotation(5), rotation(6), rotation(7));
    for (int i = 0; i < target.size(); ++i)
    {
        target(i) = std::clamp(target(i),
                               -param_.control_limits.max_steering_angle,
                               param_.control_limits.max_steering_angle);
    }
    return target;
}

bool MpcLocalPlanner::stepSteeringToward(const Eigen::Vector4d& target, Eigen::Vector4d& stepped_steer) const
{
    const double step = std::max(1e-4, align_recenter_max_step_);
    const double tolerance = std::max(0.0, align_recenter_tolerance_);
    Eigen::Vector4d current = has_last_steer_cmd_ ? last_steer_cmd_ : Eigen::Vector4d::Zero();
    bool done = true;

    for (int i = 0; i < 4; ++i)
    {
        const double error = target(i) - current(i);
        if (std::fabs(error) > step)
        {
            current(i) += (error > 0.0) ? step : -step;
        }
        else
        {
            current(i) = target(i);
        }

        if (std::fabs(target(i) - current(i)) > tolerance)
        {
            done = false;
        }
    }

    stepped_steer = current;
    return done;
}

void MpcLocalPlanner::resetControlToReference(int min_index, double min_e)
{
    if (min_e <= 1.0)
    {
        return;
    }

    const double v_r = speed_profile_[min_index];
    const double k_r = rcurvature_[min_index];
    const double omega_r = v_r * k_r;
    const double L = param_.L;
    const double W = param_.W;
    const double xw[4] = { L, -L, -L, L };
    const double yw[4] = { W,  W, -W, -W };

    for (int i = 0; i < 4; i++)
    {
        const double vx_i = v_r - yw[i] * omega_r;
        const double vy_i = xw[i] * omega_r;
        mpc_->U(i) = std::sqrt(vx_i * vx_i + vy_i * vy_i) * (vx_i >= 0 ? 1.0 : -1.0);
        mpc_->U(i + 4) = std::atan2(vy_i, vx_i);
    }

    ROS_WARN_THROTTLE(1.0, "MpcLocalPlanner: lateral error %.3f, resetting control to reference.", min_e);
}

bool MpcLocalPlanner::solveMpcCommand(const Eigen::Vector3d& state,
                                      int min_index,
                                      Eigen::VectorXd& U_solve)
{
    try
    {
        U_solve = mpc_->mpc_solve(r_x_, r_y_, ryaw_, speed_profile_,
                                  state, min_index, speed_profile_[min_index], param_,
                                  has_last_control_command_
                                      ? last_control_command_
                                      : Eigen::VectorXd::Zero(NMPC_NU));
    }
    catch (const std::exception&)
    {
        publishZeroCommands();
        return false;
    }

    return true;
}

bool MpcLocalPlanner::publishWheelCommands(Eigen::VectorXd& U)
{
    const ros::WallTime now = ros::WallTime::now();
    Eigen::VectorXd previous = Eigen::VectorXd::Zero(NMPC_NU);
    double elapsed = param_.dt;

    if (has_last_control_command_ && last_control_command_.size() == NMPC_NU)
    {
        previous = last_control_command_;
        const double measured_elapsed = (now - last_control_command_time_).toSec();
        if (std::isfinite(measured_elapsed) && measured_elapsed > 0.0)
        {
            // A missed cycle must not authorize an arbitrarily large command jump.
            elapsed = std::min(measured_elapsed, param_.dt);
        }
    }
    elapsed = std::max(elapsed, 1e-3);

    if (!applyControlLimits(U, previous, elapsed, param_.control_limits))
    {
        ROS_ERROR_THROTTLE(1.0, "MpcLocalPlanner rejected an invalid wheel command.");
        return false;
    }

    std_msgs::Float64 msg;

    // Wheel angular velocities (rad/s) = linear velocity / wheel radius
    msg.data = U(0) / wheel_radius_;
    pub_whell_front_L_.publish(msg);

    msg.data = U(1) / wheel_radius_;
    pub_whell_rear_L_.publish(msg);

    msg.data = U(2) / wheel_radius_;
    pub_whell_rear_R_.publish(msg);

    msg.data = U(3) / wheel_radius_;
    pub_whell_front_R_.publish(msg);

    // Steering angles (rad)
    msg.data = U(4);
    pub_steer_front_L_.publish(msg);

    msg.data = U(5);
    pub_steer_rear_L_.publish(msg);

    msg.data = U(6);
    pub_steer_rear_R_.publish(msg);

    msg.data = U(7);
    pub_steer_front_R_.publish(msg);

    rememberSteeringCommand(U);
    last_control_command_ = U;
    last_control_command_time_ = now;
    has_last_control_command_ = true;
    mpc_->U = U;
    return true;
}

void MpcLocalPlanner::publishZeroCommands() const
{
    std_msgs::Float64 msg;
    msg.data = 0.0;
    pub_whell_front_L_.publish(msg);
    pub_steer_front_L_.publish(msg);
    pub_whell_front_R_.publish(msg);
    pub_steer_front_R_.publish(msg);
    pub_whell_rear_L_.publish(msg);
    pub_steer_rear_L_.publish(msg);
    pub_whell_rear_R_.publish(msg);
    pub_steer_rear_R_.publish(msg);

    has_last_steer_cmd_ = true;
    last_steer_cmd_.setZero();
    last_control_command_ = Eigen::VectorXd::Zero(NMPC_NU);
    last_control_command_time_ = ros::WallTime::now();
    has_last_control_command_ = true;
    if (mpc_)
    {
        mpc_->U = last_control_command_;
    }
}

void MpcLocalPlanner::publishStoppedCommands() const
{
    std_msgs::Float64 msg;
    msg.data = 0.0;
    pub_whell_front_L_.publish(msg);
    pub_whell_rear_L_.publish(msg);
    pub_whell_rear_R_.publish(msg);
    pub_whell_front_R_.publish(msg);

    const Eigen::Vector4d steering = has_last_steer_cmd_
                                         ? last_steer_cmd_
                                         : Eigen::Vector4d::Zero();
    msg.data = steering(0);
    pub_steer_front_L_.publish(msg);
    msg.data = steering(1);
    pub_steer_rear_L_.publish(msg);
    msg.data = steering(2);
    pub_steer_rear_R_.publish(msg);
    msg.data = steering(3);
    pub_steer_front_R_.publish(msg);

    last_steer_cmd_ = steering;
    has_last_steer_cmd_ = true;
    last_control_command_ = Eigen::VectorXd::Zero(NMPC_NU);
    last_control_command_.tail(4) = steering;
    last_control_command_time_ = ros::WallTime::now();
    has_last_control_command_ = true;
    if (mpc_)
    {
        mpc_->U = last_control_command_;
    }
}

void MpcLocalPlanner::rememberSteeringCommand(const Eigen::VectorXd& U) const
{
    if (U.size() < 8)
    {
        return;
    }

    last_steer_cmd_ << U(4), U(5), U(6), U(7);
    has_last_steer_cmd_ = true;
}

bool MpcLocalPlanner::applySteeringRecenterStep(
    geometry_msgs::Twist& cmd_vel,
    bool& done)
{
    const double step = std::max(1e-4, align_recenter_max_step_);
    done = false;

    // Keep the current steering angles until the commanded wheel speeds have
    // ramped to zero. Changing steering while a rate-limited wheel command is
    // still active would move the chassis during recentering.
    if (has_last_control_command_ && last_control_command_.size() == NMPC_NU &&
        last_control_command_.head(4).cwiseAbs().maxCoeff() > 1e-3)
    {
        Eigen::VectorXd braking_command = last_control_command_;
        braking_command.head(4).setZero();
        if (!publishWheelCommands(braking_command))
        {
            publishZeroCommands();
            return false;
        }
        align_recenter_steer_ << braking_command(4), braking_command(5),
                                  braking_command(6), braking_command(7);
        computeEquivalentTwist(braking_command, cmd_vel);
        ROS_INFO_THROTTLE(
            1.0,
            "MpcLocalPlanner: braking before steering recenter, wheel_speed_max=%.3f.",
            braking_command.head(4).cwiseAbs().maxCoeff());
        return true;
    }

    Eigen::VectorXd U = Eigen::VectorXd::Zero(NMPC_NU);
    for (int i = 0; i < 4; ++i)
    {
        const double angle = stepTowardZero(align_recenter_steer_(i), step);
        align_recenter_steer_(i) = angle;
        U(i + 4) = angle;
    }

    if (!publishWheelCommands(U))
    {
        publishZeroCommands();
        return false;
    }
    align_recenter_steer_ << U(4), U(5), U(6), U(7);
    done = align_recenter_steer_.cwiseAbs().maxCoeff() <= 1e-6;
    cmd_vel = geometry_msgs::Twist();
    return true;
}

bool MpcLocalPlanner::recenterSteering(geometry_msgs::Twist& cmd_vel)
{
    bool done = false;
    if (!applySteeringRecenterStep(cmd_vel, done))
    {
        return false;
    }

    if (done)
    {
        align_recenter_steer_.setZero();
        state_ = State::TRACKING;
        ROS_INFO("MpcLocalPlanner: steering recentered, switching to tracking.");
    }
    else
    {
        ROS_INFO_THROTTLE(1.0,
                          "MpcLocalPlanner: recentering steering, steer_cmds=[%.3f, %.3f, %.3f, %.3f]",
                          align_recenter_steer_(0), align_recenter_steer_(1),
                          align_recenter_steer_(2), align_recenter_steer_(3));
    }

    return true;
}

void MpcLocalPlanner::computeEquivalentTwist(const Eigen::VectorXd& U, geometry_msgs::Twist& cmd_vel)
{
    // Approximate body-frame velocity from wheel commands
    // Wheel positions: FL(L, W), RL(-L, W), RR(-L, -W), FR(L, -W)
    double L = param_.L;
    double W = param_.W;

    double vx_sum = 0.0;
    double vy_sum = 0.0;
    double omega_sum = 0.0;

    // wheel linear velocities and steering angles
    double v[4] = { U(0), U(1), U(2), U(3) };
    double d[4] = { U(4), U(5), U(6), U(7) };

    for (int i = 0; i < 4; ++i)
    {
        vx_sum += v[i] * std::cos(d[i]);
        vy_sum += v[i] * std::sin(d[i]);
    }

    double vx_body = vx_sum / 4.0;
    double vy_body = vy_sum / 4.0;

    // Estimate omega from wheel contributions
    // v_i cos(d_i) = vx - yw_i * omega
    // v_i sin(d_i) = vy + xw_i * omega
    double xw[4] = { L, -L, -L, L };
    double yw[4] = { W,  W, -W, -W };

    for (int i = 0; i < 4; ++i)
    {
        double omega_from_x = (v[i] * std::cos(d[i]) - vx_body) / (-yw[i]);
        double omega_from_y = (v[i] * std::sin(d[i]) - vy_body) / xw[i];
        if (std::isfinite(omega_from_x))
        {
            omega_sum += omega_from_x;
        }
        if (std::isfinite(omega_from_y))
        {
            omega_sum += omega_from_y;
        }
    }

    cmd_vel.linear.x = vx_body;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = omega_sum / 8.0;
}

} // namespace mpc_local_planner
