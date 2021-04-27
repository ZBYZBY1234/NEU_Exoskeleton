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

#define     USB_DEVICE  "/dev/ttyUSB3"
#define     topic       "Exoskeleton_Left_Calf"
using namespace std::chrono_literals;
/*
 * @Name: MPU6050
 * @Description: Using the MPU6050 Class to let Serial Port be initialized.
*/
MPU6050 mpu6050 = MPU6050 (USB_DEVICE,B115200,"Left_Calf");


struct Producer : public rclcpp::Node
{
    Producer(const std::string & name, const std::string & output)
    : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
    {
        // Create a publisher on the output topic.
        /*
         * @Name: create_publisher
         * @Description: Create a Publisher based on the std_msgs::Msg::Float64MultiArray message and the topic is output.
        */
        pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(output, 10);
        std::weak_ptr<std::remove_pointer<decltype(pub_.get())>::type> captured_pub = pub_;
        /*
         * @Name: callback
         * @Descripton: Call back function for Reading data.
        */
        auto callback = [captured_pub]() -> void {
            auto pub_ptr = captured_pub.lock();
            if (!pub_ptr) {
            return;
            }
            static int32_t count = 0;
            std_msgs::msg::Float64MultiArray::UniquePtr msg(new std_msgs::msg::Float64MultiArray());

            //Eigen::Matrix<float,3,1> Angle;
            float * Angle;
            Angle = mpu6050.Read_Data();

            msg->data = {mpu6050.angle_x,mpu6050.angle_y,mpu6050.angle_z};
            pub_ptr->publish(std::move(msg));
        };

        /*
         * @Name: crate_wall_timer
         * @Description: Create a timer, the input need initialize the time and call back function.
         *              The time need to match with the MPU6050 sensor.
         * @Input: time, callback
        */
        timer_ = this->create_wall_timer(0.000000868s, callback);
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int  main(int argc, char * argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;

    auto producer = std::make_shared<Producer>("producer", topic);
    executor.add_node(producer);
    executor.spin();

    rclcpp::shutdown();

}
