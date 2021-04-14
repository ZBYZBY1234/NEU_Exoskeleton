/* * @Author: Beal.MS
   * @Date: 2021-04-14 12:16:11
 * @Last Modified by: Beal.MS
 * @Last Modified time: 2021-04-14 12:48:33
   * @Description: Sensor module colaboration
*/
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

#define Piezoelectric_topic                         "Piezoelectric"
#define MPU6050_Thigh_topic                         "MPU6050_Thigh"
#define MPU6050_Calf_topic                          "MPU6050_Calf"
#define Sensor_topic                                "Sensor"

float Angle_Thigh[3]={0,0,0};
float Angle_Calf[3]={0,0,0};
float Pressor[3]={0,0,0};

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

/*
    * @Name: class SingleThreadedNode
    * @Description: create a class for Piezoelectric Sensor
*/
class SingleThreadedNode : public rclcpp::Node
{
public:
    SingleThreadedNode()
    : Node("SingleThreadedNode")
    {
        callback_group_subscriber1_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive
        );

        auto sub1_opt = rclcpp::SubscriptionOptions();
        sub1_opt.callback_group = callback_group_subscriber1_;

        subscription1_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            Piezoelectric_topic,
            rclcpp::QoS(10),
            std::bind(
                &SingleThreadedNode::subscriber1_cb,
                this,
                std::placeholders::_1
            ),
            sub1_opt
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
            start = false;
            return std::to_string(time.seconds());
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
            this->get_logger(),"Piezoelectric: THREAD %s => Heard %f %f %f at %f",
            string_thread_id().c_str(),msg->data[0],msg->data[1],msg->data[2],convertFromString(message_received_at)-start_time
        );
        //Pressor = {msg->data[0], msg->data[1], msg->data[2]};
        Pressor[0] = msg->data[0];
        Pressor[1] = msg->data[1];
        Pressor[2] = msg->data[2];
    }

    rclcpp::CallbackGroup::SharedPtr                                    callback_group_subscriber1_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr   subscription1_;

    bool                                                                start;
    double                                                              start_time;
};

/*
    * @Name: class DualThreadedNode
    * @Description: This class for the two MPU6050 sensors which will be used.
*/
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
            MPU6050_Thigh_topic,
            rclcpp::QoS(10),
            std::bind(
                &DualThreadedNode::subscriber1_cb,
                this,
                std::placeholders::_1
            ),
            sub1_opt
        );

        subscription2_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            MPU6050_Calf_topic,
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
            start = false;
            return std::to_string(time.seconds());
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
            this->get_logger(),"MPU6050_Thigh: THREAD %s => Heard %f %f %f at %f",
            string_thread_id().c_str(),msg->data[0],msg->data[1],msg->data[2],convertFromString(message_received_at)-start_time
        );
        Angle_Thigh[0] = msg->data[0];
        Angle_Thigh[1] = msg->data[1];
        Angle_Thigh[2] = msg->data[2];
    }

    void subscriber2_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        auto message_received_at = timing_string();

        RCLCPP_INFO(
            this->get_logger(),"MPU6050_Calf: THREAD %s => Heard '%f %f %f' at %f",
            string_thread_id().c_str(),msg->data[0],msg->data[1],msg->data[2],convertFromString(message_received_at)-start_time
        );
        Angle_Calf[0] = msg->data[0];
        Angle_Calf[1] = msg->data[1];
        Angle_Calf[2] = msg->data[2];
    }
    rclcpp::CallbackGroup::SharedPtr                                    callback_group_subscriber1_;
    rclcpp::CallbackGroup::SharedPtr                                    callback_group_subscriber2_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr   subscription1_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr   subscription2_;

    bool                                                                start;
    double                                                              start_time;
};

class PublisherNode : public rclcpp::Node
{
public:
    PublisherNode()
    : Node ("PublisherNode"), count_(0)
    {
        start = true;
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(Sensor_topic, 10);
        auto timer_callback =
        [this]()->void {
            auto message_received_at = timing_string();
            double time = convertFromString(message_received_at) - start_time;
            auto message = std_msgs::msg::Float64MultiArray();
            message.data = {Pressor[0], Pressor[1], Pressor[2],
                            Angle_Thigh[0], Angle_Thigh[1], Angle_Thigh[2],
                            Angle_Calf[0], Angle_Calf[1], Angle_Calf[2],
                            time};
            this->publisher_->publish(message);
        };
        timer_ = this->create_wall_timer(10ms, timer_callback);
    }

private:
    std::string timing_string()
    {
        if(start)
        {
            rclcpp::Time time = this->now();
            start_time = convertFromString(std::to_string(time.seconds()));
            start = false;
            return std::to_string(time.seconds());
        }
        else{
            rclcpp::Time time = this->now();
            return std::to_string(time.seconds());
        }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    size_t count_;

    bool                                                                start;
    double                                                              start_time;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;
    auto piezoelectric_sensor = std::make_shared<SingleThreadedNode>();
    auto mpu6050_sensors = std::make_shared<DualThreadedNode>();
    auto pubnode = std::make_shared<PublisherNode>();

    executor.add_node(piezoelectric_sensor);
    executor.add_node(mpu6050_sensors);
    executor.add_node(pubnode);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}