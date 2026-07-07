#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64MultiArray.h>
#include "reference_path_publisher/cubic_spline.hpp"
#include <vector>
#include <cmath>

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ref_path_publisher");
    ros::NodeHandle nh;

    // 发布参考路径 /reference_path 和原始路点 /reference_waypoints
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/reference_path", 1, true);
    ros::Publisher wp_pub   = nh.advertise<visualization_msgs::Marker>("/reference_waypoints", 1, true);
    ros::Publisher curvature_pub = nh.advertise<std_msgs::Float64MultiArray>("/reference_curvature", 1, true);
    ros::Publisher speed_pub     = nh.advertise<std_msgs::Float64MultiArray>("/reference_speeds", 1, true);

    // 生成参考路线（与你 main.cpp 中的路点一致）
    vector<double> wx({10.0, 60.0, 125.0, 50.0, 60.0, 35.0, -10.0});
    vector<double> wy({0.0,  0.0,  50.0, 65.0,  45.0, 50.0, -20.0});

    Spline2D csp_obj(wx, wy);
    vector<double> r_x, r_y, ryaw, rcurvature, rs;
    for(double i = 0; i < csp_obj.s.back(); i += 0.5)
    {
        vector<double> point_ = csp_obj.calc_postion(i);
        r_x.push_back(point_[0]);
        r_y.push_back(point_[1]);
        ryaw.push_back(csp_obj.calc_yaw(i));
        rcurvature.push_back(csp_obj.calc_curvature(i));
        rs.push_back(i);
    }

    // 对路径做滑动平均平滑，降低 cubic spline 在稀疏路点间产生的曲率尖峰
    int path_smooth_window = 9;
    nh.param("path_smooth_window", path_smooth_window, path_smooth_window);
    if(path_smooth_window > 1 && !r_x.empty())
    {
        vector<double> sx = r_x, sy = r_y;
        int half = path_smooth_window / 2;
        for(size_t i = 0; i < r_x.size(); ++i)
        {
            double sum_x = 0.0, sum_y = 0.0;
            int count = 0;
            for(int j = -half; j <= half; ++j)
            {
                int idx = static_cast<int>(i) + j;
                if(idx >= 0 && idx < static_cast<int>(r_x.size()))
                {
                    sum_x += r_x[idx];
                    sum_y += r_y[idx];
                    count++;
                }
            }
            sx[i] = sum_x / count;
            sy[i] = sum_y / count;
        }
        r_x = sx;
        r_y = sy;

        // 根据平滑后的坐标重新计算 yaw
        for(size_t i = 0; i < r_x.size(); ++i)
        {
            size_t ip1 = std::min(i + 1, r_x.size() - 1);
            size_t im1 = (i == 0) ? 0 : i - 1;
            double dx = r_x[ip1] - r_x[im1];
            double dy = r_y[ip1] - r_y[im1];
            ryaw[i] = std::atan2(dy, dx);
        }

        // 根据平滑后的 yaw 重新计算曲率
        for(size_t i = 0; i < r_x.size(); ++i)
        {
            if(i == 0 || i + 1 >= r_x.size())
            {
                rcurvature[i] = 0.0;
                continue;
            }
            size_t ip1 = i + 1;
            size_t im1 = i - 1;
            double dx1 = r_x[ip1] - r_x[i];
            double dy1 = r_y[ip1] - r_y[i];
            double dx0 = r_x[i] - r_x[im1];
            double dy0 = r_y[i] - r_y[im1];
            double ds1 = std::sqrt(dx1*dx1 + dy1*dy1);
            double ds0 = std::sqrt(dx0*dx0 + dy0*dy0);
            double ds = 0.5 * (ds1 + ds0);
            double dyaw = ryaw[ip1] - ryaw[im1];
            while(dyaw > M_PI) dyaw -= 2.0 * M_PI;
            while(dyaw < -M_PI) dyaw += 2.0 * M_PI;
            rcurvature[i] = (ds > 1e-6) ? (dyaw / ds) : 0.0;
        }
    }

    // 计算参考速度曲线
    double target_speed = 0.5;
    // 优先读取私有参数 /ref_path_publisher/target_speed，读不到则保持默认值
    if (!ros::param::get("~target_speed", target_speed)) {
        nh.param("target_speed", target_speed, target_speed);
    }

    vector<double> speed_profile;
    double k_max = 0.0;
    for (double k : rcurvature) {
        k_max = std::max(k_max, std::fabs(k));
        // 用倒数曲线根据曲率限速：曲率越大速度越低，避免急弯冲出路径
        double speed = target_speed / (1.0 + 3.0 * std::fabs(k));
        // 保留极低下限，保证已运动的小车仍能蠕动通过急弯
        speed = std::max(0.05, std::min(target_speed, speed));
        speed_profile.push_back(speed);
    }
    ROS_INFO("target_speed=%.3f, max_curvature=%.3f, speed_range=[%.3f, %.3f]",
             target_speed, k_max,
             *std::min_element(speed_profile.begin(), speed_profile.end()),
             *std::max_element(speed_profile.begin(), speed_profile.end()));

    // 对速度曲线做滑动平均平滑，避免急弯处速度骤降导致 MPC 急刹停住
    int smooth_window = 5;
    nh.param("speed_smooth_window", smooth_window, smooth_window);
    if(smooth_window > 1 && !speed_profile.empty())
    {
        vector<double> smoothed = speed_profile;
        for(size_t i = 0; i < speed_profile.size(); ++i)
        {
            double sum = 0.0;
            int count = 0;
            int half = smooth_window / 2;
            for(int j = -half; j <= half; ++j)
            {
                int idx = static_cast<int>(i) + j;
                if(idx >= 0 && idx < static_cast<int>(speed_profile.size()))
                {
                    sum += speed_profile[idx];
                    count++;
                }
            }
            smoothed[i] = sum / count;
        }
        speed_profile = smoothed;
    }

    // 构建 nav_msgs/Path
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "map";
    for(size_t i = 0; i < r_x.size(); ++i)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = r_x[i];
        pose.pose.position.y = r_y[i];
        pose.pose.position.z = 0.0;

        // yaw 转四元数（绕 Z 轴）
        double yaw = ryaw[i];
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = sin(yaw / 2.0);
        pose.pose.orientation.w = cos(yaw / 2.0);

        path_msg.poses.push_back(pose);
    }

    // 构建曲率和速度消息
    std_msgs::Float64MultiArray curvature_msg;
    curvature_msg.data = rcurvature;

    std_msgs::Float64MultiArray speed_msg;
    speed_msg.data = speed_profile;

    // 构建原始路点可视化 Marker（红色球）
    visualization_msgs::Marker wp_marker;
    wp_marker.header.frame_id = "map";
    wp_marker.ns = "waypoints";
    wp_marker.id = 0;
    wp_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    wp_marker.action = visualization_msgs::Marker::ADD;
    wp_marker.scale.x = 1.5;
    wp_marker.scale.y = 1.5;
    wp_marker.scale.z = 1.5;
    wp_marker.color.r = 1.0f;
    wp_marker.color.g = 0.0f;
    wp_marker.color.b = 0.0f;
    wp_marker.color.a = 1.0f;
    for(size_t i = 0; i < wx.size(); ++i)
    {
        geometry_msgs::Point p;
        p.x = wx[i];
        p.y = wy[i];
        p.z = 0.0;
        wp_marker.points.push_back(p);
    }

    // 先发布一次，方便 latched 订阅者获取
    path_pub.publish(path_msg);
    wp_pub.publish(wp_marker);
    curvature_pub.publish(curvature_msg);
    speed_pub.publish(speed_msg);
    ROS_INFO("Published reference path with %zu points, curvature and speeds.", r_x.size());

    // 持续发布（rviz 需要不断更新的 Path）
    ros::Rate rate(10);
    while(ros::ok())
    {
        path_msg.header.stamp = ros::Time::now();
        for(auto& pose : path_msg.poses)
        {
            pose.header.stamp = ros::Time::now();
        }
        path_pub.publish(path_msg);

        curvature_msg.layout.data_offset = 0;
        speed_msg.layout.data_offset = 0;
        curvature_pub.publish(curvature_msg);
        speed_pub.publish(speed_msg);

        wp_marker.header.stamp = ros::Time::now();
        wp_pub.publish(wp_marker);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
