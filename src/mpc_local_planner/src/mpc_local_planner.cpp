#include "mpc_local_planner/mpc_local_planner.hpp"
#include <pluginlib/class_list_macros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <angles/angles.h>

PLUGINLIB_EXPORT_CLASS(mpc_local_planner::MpcLocalPlanner, nav_core::BaseLocalPlanner)

namespace mpc_local_planner
{

MpcLocalPlanner::MpcLocalPlanner()
{
}

MpcLocalPlanner::~MpcLocalPlanner()
{
}

void MpcLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
    if (initialized_)
    {
        ROS_WARN("MpcLocalPlanner has already been initialized, doing nothing.");
        return;
    }

    name_ = name;
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

    // Load parameters
    private_nh.param("wheel_radius", wheel_radius_, 0.15);
    private_nh.param("max_speed", max_speed_, 0.5);
    private_nh.param("target_speed", target_speed_, 0.5);
    private_nh.param("forward_window", forward_window_, 80);
    private_nh.param("back_buffer", back_buffer_, 10);
    private_nh.param("goal_xy_tolerance", goal_xy_tolerance_, 0.10);
    private_nh.param("goal_yaw_tolerance", goal_yaw_tolerance_, 0.05);

    // MPC parameters from diffmpc.hpp defaults, allow override
    private_nh.param("L", param_.L, 0.4615);
    private_nh.param("W", param_.W, 0.4);
    param_.NX = NMPC_NX;
    param_.NU = NMPC_NU;
    param_.NP = NMPC_T;
    param_.NC = NMPC_T - 1;
    param_.dt = NMPC_DT;

    // Subscribe to odometry for velocity/backup state
    odom_sub_ = nh.subscribe("/odom", 1, &MpcLocalPlanner::odomCallback, this);

    // Advertise 8 wheel command publishers
    pub_whell_front_L_ = nh.advertise<std_msgs::Float64>("/smart/front_left_svelocity_controller/command", 1);
    pub_steer_front_L_ = nh.advertise<std_msgs::Float64>("/smart/front_left_str_controller/command", 1);
    pub_whell_front_R_ = nh.advertise<std_msgs::Float64>("/smart/front_right_velocity_controller/command", 1);
    pub_steer_front_R_ = nh.advertise<std_msgs::Float64>("/smart/front_right_str_controller/command", 1);
    pub_whell_rear_L_  = nh.advertise<std_msgs::Float64>("/smart/rear_left_velocity_controller/command", 1);
    pub_steer_rear_L_  = nh.advertise<std_msgs::Float64>("/smart/rear_left_str_controller/command", 1);
    pub_whell_rear_R_  = nh.advertise<std_msgs::Float64>("/smart/rear_right_velocity_controller/command", 1);
    pub_steer_rear_R_  = nh.advertise<std_msgs::Float64>("/smart/rear_right_str_controller/command", 1);

    // Initialize MPC
    mpc_.reset(new diffMpcController(param_.NX, param_.NU, param_.NP, param_.NC));
    agv_model_.reset(new KinematicModel_MPC(0.0, 0.0, 0.0, 0.0, param_.L, param_.W, param_.dt));

    initialized_ = true;
    ROS_INFO("MpcLocalPlanner initialized. wheel_radius=%.3f, max_speed=%.3f, L=%.3f, W=%.3f",
             wheel_radius_, max_speed_, param_.L, param_.W);
}

bool MpcLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
    if (!initialized_)      // 查看初始化是否成功
    {
        ROS_ERROR("MpcLocalPlanner has not been initialized, please call initialize() before using this planner.");
        return false;
    }

    if (plan.empty())
    {
        ROS_WARN("Received empty plan.");
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    convertPlanToReference(plan);
    last_min_index_ = 0;
    has_plan_ = true;

    // Reset MPC internal state for the new path
    mpc_->reset();

    ROS_INFO("MpcLocalPlanner received plan with %zu points.", plan.size());
    return true;
}

bool MpcLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_)
    {
        ROS_ERROR("MpcLocalPlanner has not been initialized.");
        return false;
    }

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

    // 计算当前位置在参考轨迹上的最近点索引和误差
    auto [min_index, min_e] = calcForwardNearestIndex(initial_x(0), initial_x(1));

    // Check goal reached by distance to the (preserved) final point.
    // We stop wheel commands here immediately, even if isGoalReached() hasn't
    // been checked by move_base in this control cycle yet.
    double dist_to_goal = 0.0, dyaw_to_goal = 0.0;
    bool goal_reached = checkGoalReached(initial_x, dist_to_goal, dyaw_to_goal);
    if (goal_reached)
    {
        ROS_INFO_ONCE("MpcLocalPlanner: reached goal (dist=%.3f, dyaw=%.3f), stopping.",
                      dist_to_goal, dyaw_to_goal);
        publishZeroCommands();
        cmd_vel = geometry_msgs::Twist();
        return true;
    }

    // If lateral error too large, reset control to reference
    if (min_e > 1.0)
    {
        double v_r = speed_profile_[min_index];
        double k_r = rcurvature_[min_index];
        double omega_r = v_r * k_r;
        double L = param_.L;
        double W = param_.W;
        double xw[4] = { L, -L, -L, L };
        double yw[4] = { W,  W, -W, -W };

        for (int i = 0; i < 4; i++)
        {
            double vx_i = v_r - yw[i] * omega_r;
            double vy_i = xw[i] * omega_r;
            mpc_->U(i) = std::sqrt(vx_i * vx_i + vy_i * vy_i) * (vx_i >= 0 ? 1.0 : -1.0);
            mpc_->U(i + 4) = std::atan2(vy_i, vx_i);
        }
        ROS_WARN_THROTTLE(1.0, "MpcLocalPlanner: lateral error %.3f, resetting control to reference.", min_e);
    }

    // Update kinematic model state
    agv_model_->x   = initial_x(0);
    agv_model_->y   = initial_x(1);
    agv_model_->yaw = initial_x(2);
    agv_model_->v   = speed_profile_[min_index];

    // Solve MPC
    Eigen::VectorXd U_solve;
    try
    {
        U_solve = mpc_->mpc_solve(r_x_, r_y_, ryaw_, rcurvature_, speed_profile_,
                                  initial_x, min_index, min_e, *agv_model_, param_);
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("MpcLocalPlanner MPC solve failed: %s", e.what());
        publishZeroCommands();
        return false;
    }

    // Publish wheel commands
    publishWheelCommands(U_solve);

    // 返回给 move_base 的 cmd_vel
    computeEquivalentTwist(U_solve, cmd_vel);

    // ROS_INFO_THROTTLE(1.0,
    //                   "MpcLocalPlanner: nearest_idx=%d, nearest_dist=%.3f, v=[%.2f, %.2f, %.2f, %.2f], steer=[%.3f, %.3f, %.3f, %.3f]",
    //                   min_index, min_e,
    //                   U_solve(0), U_solve(1), U_solve(2), U_solve(3),
    //                   U_solve(4), U_solve(5), U_solve(6), U_solve(7));

    return true;
}

bool MpcLocalPlanner::checkGoalReached(const Eigen::Vector3d& current_state,
                                       double& dist_to_goal, double& dyaw) const
{
    if (!initialized_ || !has_plan_ || r_x_.empty())
    {
        dist_to_goal = std::numeric_limits<double>::max();
        dyaw = std::numeric_limits<double>::max();
        return false;
    }

    double dx = current_state(0) - r_x_.back();
    double dy = current_state(1) - r_y_.back();
    dist_to_goal = std::sqrt(dx * dx + dy * dy);
    dyaw = angles::shortest_angular_distance(current_state(2), ryaw_.back());
    cout << "检查结果：" << (dist_to_goal <= goal_xy_tolerance_ && std::fabs(dyaw) <= goal_yaw_tolerance_) << endl;

    if (dist_to_goal <= goal_xy_tolerance_ && std::fabs(dyaw) <= goal_yaw_tolerance_)
    {
        ROS_INFO_ONCE("MpcLocalPlanner: reached goal");
        publishZeroCommands();
        // cmd_vel = geometry_msgs::Twist();
        return true;
    }else
    {
        return false;
    }
    // return dist_to_goal <= goal_xy_tolerance_ && std::fabs(dyaw) <= goal_yaw_tolerance_;
}

bool MpcLocalPlanner::isGoalReached()
{
    std::lock_guard<std::mutex> lock(mutex_);

    Eigen::Vector3d current_state;
    if (!getRobotPose(current_state))
    {
        return false;
    }

    double dist_to_goal = 0.0, dyaw = 0.0;
    return checkGoalReached(current_state, dist_to_goal, dyaw);
}

void MpcLocalPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    latest_odom_ = *msg;
    has_odom_ = true;
}

double MpcLocalPlanner::getYaw(const geometry_msgs::Quaternion& q) const
{
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(quat);
    double r, p, y;
    m.getRPY(r, p, y);
    return y;
}

bool MpcLocalPlanner::getRobotPose(Eigen::Vector3d& state) const
{
    geometry_msgs::PoseStamped robot_pose;
    if (!costmap_ros_->getRobotPose(robot_pose))
    {
        // Fallback to odometry if costmap pose is unavailable
        if (!has_odom_)
        {
            return false;
        }
        state << latest_odom_.pose.pose.position.x,
                 latest_odom_.pose.pose.position.y,
                 getYaw(latest_odom_.pose.pose.orientation);
        return true;
    }

    state << robot_pose.pose.position.x,
             robot_pose.pose.position.y,
             getYaw(robot_pose.pose.orientation);
    return true;
}

// 滑动平均平滑一个一维序列
std::vector<double> MpcLocalPlanner::smoothVector(const std::vector<double>& data, int window) const
{
    std::vector<double> smoothed = data;
    if (window <= 1 || data.empty()) return smoothed;

    int half = window / 2;
    for (size_t i = 0; i < data.size(); ++i)
    {
        double sum = 0.0;
        int count = 0;
        for (int j = -half; j <= half; ++j)
        {
            int idx = static_cast<int>(i) + j;
            if (idx >= 0 && idx < static_cast<int>(data.size()))
            {
                sum += data[idx];
                count++;
            }
        }
        if (count > 0) smoothed[i] = sum / count;
    }
    return smoothed;
}

// 将接收到的全局路径转换为参考轨迹，包括位置、航向、曲率和速度
void MpcLocalPlanner::convertPlanToReference(const std::vector<geometry_msgs::PoseStamped>& plan)
{
    r_x_.clear();
    r_y_.clear();
    ryaw_.clear();
    rcurvature_.clear();
    speed_profile_.clear();

    const int path_smooth_window = 9;
    const int speed_smooth_window = 5;
    const int curvature_smooth_window = 5;

    for (const auto& pose : plan)
    {
        r_x_.push_back(pose.pose.position.x);
        r_y_.push_back(pose.pose.position.y);
    }

    // 先对路径坐标做滑动平均平滑，降低全局规划器稀疏路点造成的抖动
    // 但保持首尾点不变，避免终点被拖离原始目标位置
    {
        std::vector<double> sx = smoothVector(r_x_, path_smooth_window);
        std::vector<double> sy = smoothVector(r_y_, path_smooth_window);
        if (!r_x_.empty()) {
            sx.front() = r_x_.front();
            sx.back()  = r_x_.back();
        }
        if (!r_y_.empty()) {
            sy.front() = r_y_.front();
            sy.back()  = r_y_.back();
        }
        r_x_ = std::move(sx);
        r_y_ = std::move(sy);
    }

    // 从平滑后的坐标重新计算航向角（不依赖 pose.orientation，兼容性更好）
    ryaw_.resize(plan.size(), 0.0);
    for (size_t i = 0; i < plan.size(); ++i)
    {
        size_t ip1 = std::min(i + 1, plan.size() - 1);
        size_t im1 = (i == 0) ? 0 : i - 1;
        double dx = r_x_[ip1] - r_x_[im1];
        double dy = r_y_[ip1] - r_y_[im1];
        ryaw_[i] = std::atan2(dy, dx);
    }

    // 平滑航向角，避免曲率计算时出现不连续
    mpc_->smooth_yaw(ryaw_);

    // 计算曲率
    rcurvature_.resize(plan.size(), 0.0);
    for (size_t i = 0; i < plan.size(); ++i)
    {
        if (i == 0 || i + 1 >= plan.size())
        {
            rcurvature_[i] = 0.0;
            continue;
        }

        size_t ip1 = i + 1;
        size_t im1 = i - 1;
        double dx1 = r_x_[ip1] - r_x_[i];
        double dy1 = r_y_[ip1] - r_y_[i];
        double dx0 = r_x_[i] - r_x_[im1];
        double dy0 = r_y_[i] - r_y_[im1];
        double ds1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
        double ds0 = std::sqrt(dx0 * dx0 + dy0 * dy0);
        double ds = 0.5 * (ds1 + ds0);

        double dyaw = ryaw_[ip1] - ryaw_[im1];
        dyaw = angles::normalize_angle(dyaw);

        rcurvature_[i] = (ds > 1e-6) ? (dyaw / ds) : 0.0;
    }

    // 平滑曲率，避免速度曲线剧烈抖动
    rcurvature_ = smoothVector(rcurvature_, curvature_smooth_window);

    // 计算参考速度，考虑曲率和最大速度约束
    speed_profile_ = mpc_->calculateReferenceSpeeds(rcurvature_, target_speed_);

    // 把速度钳制在 [min_speed, max_speed] 范围内
    const double min_speed = 0.15;
    for (auto& v : speed_profile_)
    {
        v = std::max(min_speed, std::min(max_speed_, v));
    }

    // 对速度曲线做滑动平均平滑，避免急弯处速度骤降导致 MPC 急刹停住
    speed_profile_ = smoothVector(speed_profile_, speed_smooth_window);

    // 终点前减速到 0，避免到了终点还在以 min_speed 硬冲
    const double stop_distance = 0.4;
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
        return mpc_->calc_ref_trajectory(current_x, current_y, r_x_, r_y_, ryaw_);
    }

    last_min_index_ = min_index;
    return std::make_tuple(min_index, mind);
}

void MpcLocalPlanner::publishWheelCommands(const Eigen::VectorXd& U)
{
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
