#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "MPU6050", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) const
    {
        auto message = msg->data;
        RCLCPP_INFO(this->get_logger(), "I heard: '%f'", message[0]);
        auto message2 = std_msgs::msg::Float64MultiArray();
        message2.data = {  0.0,  -message[2], 0.0, 0.0,
                            0.0,  -message[2], 0.0, 0.0};

        Joint_Publisher->publish(message2);
    }
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr Joint_Publisher;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}