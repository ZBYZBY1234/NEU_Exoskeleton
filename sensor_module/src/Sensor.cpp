#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include <cinttypes>
#include <cstdio>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

std::string string_thread_id()
{
    auto hashed = std::hash<std::thread::id>()(std::this_thread::get_id());
    return std::to_string(hashed);
}

double convertFromString(std::string str)
{
    std::istringstream iss(str);
    double x;
    if(iss >> x)
        return x;
    return 0.0;
}
class DualThreadedNode : public rclcpp::Node
{
public:
    DualThreadedNode()
    : Node("DualThreadedNode")
    {
        callback_group_subscriber1_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive
        );
        callback_group_subscriber2_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive
        );

        auto sub1_opt = rclcpp::SubscriptionOptions();
        sub1_opt.callback_group = callback_group_subscriber1_;
        auto sub2_opt = rclcpp::SubscriptionOptions();
        sub2_opt.callback_group = callback_group_subscriber2_;

        subscription1_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "Piezoelectric",
            rclcpp::QoS(10),
            std::bind(
                &DualThreadedNode::subscriber1_cb,
                this,
                std::placeholders::_1
            ),
            sub1_opt
        );

        subscription2_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "MPU6050",
            rclcpp::QoS(10),
            std::bind(
                &DualThreadedNode::subscriber2_cb,
                this,
                std::placeholders::_1
            ),
            sub2_opt
        );

        start = true;
    }

private:
    std::string timing_string()
    {
        if(start)
        {
            rclcpp::Time time = this->now();
            start_time = convertFromString(std::to_string(time.seconds()));
            return std::to_string(time.seconds());
            start = false;
        }
        else{
            rclcpp::Time time = this->now();
            return std::to_string(time.seconds());
        }
    }

    void subscriber1_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        auto message_received_at = timing_string();
        RCLCPP_INFO(
            this->get_logger(),"THREAD %s => Heard %f %f %f at %f",
            string_thread_id().c_str(),msg->data[0],msg->data[1],msg->data[2],convertFromString(message_received_at)-start_time
        );
    }

    void subscriber2_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        auto message_received_at = timing_string();

        RCLCPP_INFO(
            this->get_logger(),"THREAD %s => Heard '%f %f %f' at %f",
            string_thread_id().c_str(),msg->data[0],msg->data[1],msg->data[2],convertFromString(message_received_at)-start_time
        );
    }
    rclcpp::CallbackGroup::SharedPtr                                    callback_group_subscriber1_;
    rclcpp::CallbackGroup::SharedPtr                                    callback_group_subscriber2_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr   subscription1_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr   subscription2_;

    bool                                                                start;
    double                                                              start_time;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;
    auto subnode = std::make_shared<DualThreadedNode>();

    executor.add_node(subnode);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}