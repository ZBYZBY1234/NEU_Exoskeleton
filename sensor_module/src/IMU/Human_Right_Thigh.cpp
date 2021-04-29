#include "sensor_module/MPU6050.hpp"
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

#define     USB_DEVICE  "/dev/ttyUSB6"
#define     topic       "Human_Right_Thigh"
using namespace std::chrono_literals;
/*
 * @Name: MPU6050
 * @Description: Using the MPU6050 Class to let Serial Port be initialized.
*/
MPU6050 mpu6050 = MPU6050 (USB_DEVICE,B115200,"Right_Thigh");


class Human_Right_Thigh : public rclcpp::Node
{
public:
    Human_Right_Thigh()
    : Node("Human_Right_Thigh")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(topic, 10);
        timer_ = this->create_wall_timer(
            0.000000868s,
            std::bind(
                &Human_Right_Thigh::timer_callback,
                this)
        );
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::Float64MultiArray();
        float * Angle;
        Angle = mpu6050.Read_Data();
        // std::cout << mpu6050.Ang[0] << ","
        // << mpu6050.Vel[0] << ","
        // << mpu6050.Acc[0] << std::endl;
        message.data = {mpu6050.Ang[0],mpu6050.Vel[0],mpu6050.Acc[0]};
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    size_t count_;
};

int  main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Human_Right_Thigh>());
    rclcpp::shutdown();
}
