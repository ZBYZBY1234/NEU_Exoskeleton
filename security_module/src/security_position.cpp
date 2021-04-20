#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using std::placeholders::_1;

class MinimalSubscriber :
    public rclcpp::Node
{
public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1)
        );

        //TODO: Change the Topic of Joint States
        Joint_Subscription = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&MinimalSubscriber::callback1, this, _1)
        );
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    void callback1(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        auto position = msg->position;
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr Joint_Subscription;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}