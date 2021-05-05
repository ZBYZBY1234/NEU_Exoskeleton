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

#define Piezoelectric_topic                            "Piezoelectric"
#define Exoskeleton_Left_Thigh_topic                   "Exoskeleton_Left_Thigh"
#define Exoskeleton_Left_Calf_topic                    "Exoskeleton_Left_Calf"
#define Exoskeleton_Right_Thigh_topic                  "Exoskeleton_Right_Thigh"
#define Exoskeleton_Right_Calf_topic                   "Exoskeleton_Right_Calf"
#define Sensor_topic                                   "Sensor_Exoskeleton"

float Left_Thigh_Exoskeleton[3]={0,0,0};
float Left_Calf_Exoskeleton[3]={0,0,0};
float Right_Thigh_Exoskeleton[3]={0,0,0};
float Right_Calf_Exoskeleton[3]={0,0,0};

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
    * @Name: class IMU_Node
    * @Description: This class for the two MPU6050 sensors which will be used.
*/
class IMU_Node : public rclcpp::Node
{
public:
    IMU_Node()
    : Node("Exoskeleton_IMU_Node")
    {
        callback_group_subscriber1_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive
        );
        callback_group_subscriber2_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive
        );
        callback_group_subscriber3_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive
        );
        callback_group_subscriber4_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive
        );

        auto sub1_opt = rclcpp::SubscriptionOptions();
        sub1_opt.callback_group = callback_group_subscriber1_;
        auto sub2_opt = rclcpp::SubscriptionOptions();
        sub2_opt.callback_group = callback_group_subscriber2_;
        auto sub3_opt = rclcpp::SubscriptionOptions();
        sub3_opt.callback_group = callback_group_subscriber3_;
        auto sub4_opt = rclcpp::SubscriptionOptions();
        sub4_opt.callback_group = callback_group_subscriber4_;

        subscription1_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            Exoskeleton_Left_Thigh_topic,
            rclcpp::QoS(1),
            std::bind(
                &IMU_Node::subscriber1_cb,
                this,
                std::placeholders::_1
            ),
            sub1_opt
        );

        subscription2_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            Exoskeleton_Left_Calf_topic,
            rclcpp::QoS(1),
            std::bind(
                &IMU_Node::subscriber2_cb,
                this,
                std::placeholders::_1
            ),
            sub2_opt
        );

        subscription3_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            Exoskeleton_Right_Thigh_topic,
            rclcpp::QoS(1),
            std::bind(
                &IMU_Node::subscriber3_cb,
                this,
                std::placeholders::_1
            ),
            sub3_opt
        );

        subscription4_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            Exoskeleton_Right_Calf_topic,
            rclcpp::QoS(1),
            std::bind(
                &IMU_Node::subscriber4_cb,
                this,
                std::placeholders::_1
            ),
            sub4_opt
        );

        start = true;
        /* It means that there is the first time to get the data. */
        flag1 = true;
        flag2 = true;
        flag3 = true;
        flag4 = true;
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
        if(flag1)
        {
            offset1 = msg->data[0];
            flag1 = false;
        }
        Left_Thigh_Exoskeleton[0] = msg->data[0] - offset1;
        Left_Thigh_Exoskeleton[1] = msg->data[1];
        Left_Thigh_Exoskeleton[2] = msg->data[2];
    }

    void subscriber2_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if(flag2)
        {
            offset2 = msg->data[0];
            flag2 = false;
        }
        Left_Calf_Exoskeleton[0] = msg->data[0] - offset2;
        Left_Calf_Exoskeleton[1] = msg->data[1];
        Left_Calf_Exoskeleton[2] = msg->data[2];
    }

    void subscriber3_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if(flag3)
        {
            offset3 = msg->data[0];
            flag3 = false;
        }
        Right_Thigh_Exoskeleton[0] = msg->data[0] - offset3;
        Right_Thigh_Exoskeleton[1] = msg->data[1];
        Right_Thigh_Exoskeleton[2] = msg->data[2];
    }

    void subscriber4_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if(flag4)
        {
            offset4 = msg->data[0];
            flag4 = false;
        }
        Right_Calf_Exoskeleton[0] = msg->data[0] - offset4;
        Right_Calf_Exoskeleton[1] = msg->data[1];
        Right_Calf_Exoskeleton[2] = msg->data[2];
    }

    rclcpp::CallbackGroup::SharedPtr                                    callback_group_subscriber1_;
    rclcpp::CallbackGroup::SharedPtr                                    callback_group_subscriber2_;
    rclcpp::CallbackGroup::SharedPtr                                    callback_group_subscriber3_;
    rclcpp::CallbackGroup::SharedPtr                                    callback_group_subscriber4_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr   subscription1_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr   subscription2_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr   subscription3_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr   subscription4_;

    bool                                                                start;
    double                                                              start_time;

    bool                                                                flag1,flag2,flag3,flag4;
    float                                                               offset1,offset2,offset3,offset4;
};

class PublisherNode : public rclcpp::Node
{
public:
    PublisherNode()
    : Node("Exoskeleton_Sensor_Publisher"), count_(0)
    {
        start = true;
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(Sensor_topic, 10);
        auto timer_callback =
        [this]()->void {
            auto message_received_at = timing_string();
            double time = convertFromString(message_received_at) - start_time;

            auto message = std_msgs::msg::Float64MultiArray();
            message.data = {
                            /* Angle */
                            Left_Thigh_Exoskeleton[0],
                            Left_Calf_Exoskeleton[0],
                            Right_Thigh_Exoskeleton[0],
                            Right_Calf_Exoskeleton[0],

                            /* Velocity */
                            Left_Thigh_Exoskeleton[1],
                            Left_Calf_Exoskeleton[1],
                            Right_Thigh_Exoskeleton[1],
                            Right_Calf_Exoskeleton[1],

                            /* Acceleration */
                            Left_Thigh_Exoskeleton[2],
                            Left_Calf_Exoskeleton[2],
                            Right_Thigh_Exoskeleton[2],
                            Right_Calf_Exoskeleton[2],

                            /* Time */
                            time};
            this->publisher_->publish(message);
        };
        timer_ = this->create_wall_timer(1ms, timer_callback);
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
    auto mpu6050_sensors = std::make_shared<IMU_Node>();
    auto pubnode = std::make_shared<PublisherNode>();

    executor.add_node(mpu6050_sensors);
    executor.add_node(pubnode);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}