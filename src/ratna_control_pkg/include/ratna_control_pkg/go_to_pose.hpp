#pragma once
#include "behaviortree_cpp_v3/action_node.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ratna_control_pkg/pid_ai.hpp>
#include <cmath>
float max = 1;
float min = -1;
float kp = 1;
float ki = 0;
float kd = 0;
PID pid_dist_{1.2, 0.0, 0.1, -1.0, 1.0};
PID pid_yaw_{3.0, 0.0, 0.2, -2.0, 2.0};
rclcpp::Time last_time_;

class GoToPose : public BT::StatefulActionNode
{
public:
    GoToPose(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config)
    {
        node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

        cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/omni_cont/cmd_vel", 10);
        odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&GoToPose::odom_callback, this, std::placeholders::_1));
    }

    BT::NodeStatus onStart() override
    {
        getInput("x_pose", goal_x_);
        getInput("y_pose", goal_y_);
        getInput("yaw_angle", goal_yaw_);

        pid_dist_.reset();
        pid_yaw_.reset();
        last_time_ = node_->now();
        RCLCPP_INFO(node_->get_logger(),
                    "🎯 GoToPose TARGET -> x: %.3f | y: %.3f | yaw: %.3f",
                    goal_x_, goal_y_, goal_yaw_);
        return BT::NodeStatus::RUNNING;
    }
    BT::NodeStatus onRunning() override
    {
        auto now = node_->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;
        double dx = goal_x_ - curr_x_;
        double dy = goal_y_ - curr_y_;
        double dist = hypot(dx, dy);

        double yaw_err = normalize(goal_yaw_ - curr_yaw_);

        double v = pid_dist_.update(dist, dt);
        double w = pid_yaw_.update(yaw_err, dt);

        double theta = atan2(dy, dx);

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = v * cos(theta);
        cmd.linear.y = v * sin(theta);
        cmd.angular.z = w;
        cmd_pub_->publish(cmd);

        RCLCPP_INFO(node_->get_logger(),
                    "\nGOAL  -> x: %.2f y: %.2f yaw: %.2f"
                    "\nPOSE  -> x: %.2f y: %.2f yaw: %.2f"
                    "\nERROR -> dx: %.2f dy: %.2f yaw: %.2f dist: %.2f",
                    goal_x_, goal_y_, goal_yaw_,
                    curr_x_, curr_y_, curr_yaw_,
                    dx, dy, yaw_err, dist);

        if (dist < 0.05 && fabs(yaw_err) < 0.05)
        {
            stop_robot();
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        stop_robot();
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<double>("x_pose"),
                BT::InputPort<double>("y_pose"),
                BT::InputPort<double>("yaw_angle")};
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    double curr_x_ = 0, curr_y_ = 0, curr_yaw_ = 0;
    double goal_x_, goal_y_, goal_yaw_;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        curr_x_ = msg->pose.pose.position.x;
        curr_y_ = msg->pose.pose.position.y;
        curr_yaw_ = atan2(
            2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z +
                 msg->pose.pose.orientation.x * msg->pose.pose.orientation.y),
            1 - 2 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y +
                     msg->pose.pose.orientation.z * msg->pose.pose.orientation.z));
    }

    double normalize(double a)
    {
        while (a > M_PI)
            a -= 2 * M_PI;
        while (a < -M_PI)
            a += 2 * M_PI;
        return a;
    }

    void stop_robot()
    {
        geometry_msgs::msg::Twist stop;
        cmd_pub_->publish(stop);
    }
};
