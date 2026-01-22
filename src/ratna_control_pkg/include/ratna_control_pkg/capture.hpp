#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <filesystem>

class Capture : public BT::StatefulActionNode
{
public:
    Capture(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config)
    {
        node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

        image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", 10,
            std::bind(&Capture::imageCallback, this, std::placeholders::_1));
    }

    BT::NodeStatus onStart() override
    {
        if (already_done_)
        {
            RCLCPP_INFO(node_->get_logger(), "📸 Capture skipped (already done)");
            return BT::NodeStatus::SUCCESS;
        }

        got_image_ = false;
        RCLCPP_INFO(node_->get_logger(), "📸 Waiting for camera frame...");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (!got_image_) return BT::NodeStatus::RUNNING;

        std::filesystem::create_directories("/tmp/capture");

        std::string filename = "/tmp/capture/image_" + std::to_string(counter_) + ".jpg";
        cv::imwrite(filename, last_image_);

        RCLCPP_INFO(node_->get_logger(), "📸 Saved %s", filename.c_str());

        counter_++;
        already_done_ = true;   // latch
        return BT::NodeStatus::SUCCESS;
    }

    void onHalted() override {}

    static BT::PortsList providedPorts() { return {}; }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    cv::Mat last_image_;
    bool got_image_ = false;
    bool already_done_ = false;
    int counter_ = 0;

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (already_done_) return;

        try
        {
            last_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
            got_image_ = true;
        }
        catch (...)
        {
            RCLCPP_ERROR(node_->get_logger(), "cv_bridge failed");
        }
    }
};
