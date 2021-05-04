#include "sensor_module/Piezoelectric.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <cinttypes>
#include <cstdio>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"

#define     USB_DEVICE  "/dev/ttyUSB0"
#define     topic       "Interaction_Force"

using namespace std::chrono_literals;
/*
 * @Name: MPU6050
 * @Description: Using the MPU6050 Class to let Serial Port be initialized.
*/
Piezoelectric piezoelectric = Piezoelectric (USB_DEVICE, B115200);

class Force : public rclcpp::Node
{
public:
    Force()
    : Node("Force")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(topic, 10);
        timer_ = this->create_wall_timer(
            0.000000868s,
            std::bind(
                &Force::timer_callback,
                this)
        );
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::Float64MultiArray();
        Eigen::Matrix<float,8,1> data;
        data = piezoelectric.Read();

        message.data = {    data(0,0),data(1,0),data(2,0),data(3,0),
                            data(4,0),data(5,0),data(6,0),data(7,0)
                        };
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    size_t count_;
};

int  main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Force>());
    rclcpp::shutdown();
}