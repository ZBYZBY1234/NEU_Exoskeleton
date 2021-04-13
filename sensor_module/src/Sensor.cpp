#include "sensor_module/Piezoelectric.hpp"
#include "sensor_module/MPU6050.hpp"

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

Piezoelectric piezoelectric = Piezoelectric ("/dev/ttyUSB0",B115200);
MPU6050 mpu6050_thigh = MPU6050 ("/dev/ttyUSB1",B115200,"Left_Knee_Thigh");
MPU6050 mpu6050_calf  = MPU6050 ("/dev/ttyUSB2",B115200,"Left_Knee_Calf");

Eigen::Matrix<float,3,1> Piezoelectric_Data;
Eigen::Matrix<float,3,1> MPU6050_Thigh_Angle;
Eigen::Matrix<float,3,1> MPU6050_Calf_Angle;

// class PublisherNode: public rclcpp::Node
// {
// public:
//     PublisherNode()
//     : Node("PublisherNode"),count_(0)
//     {
//         publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("Piezoelectric",10);
//         auto timer_callback =
//             [this]() -> void{
//                 //Extract current thread
//                 auto curr_thread = string_thread_id();
//                 // Prep display message
//                 // RCLCPP_INFO(
//                 //     this->get_logger(), "\n<<THREAD %s>> Publishing",
//                 //     curr_thread.c_str());
//                 int * Data;
//                 Data = piezoelectric.Read_Data();
//                 // for (int i = 0; i < 3; i++)
//                 // {
//                 //     printf("Data: %d ",*(Data+i));
//                 // }
//                 // printf("\n");
//             };
//         timer_ = this->create_wall_timer(0.000000868s,timer_callback);
//     }
// private:
//     rclcpp::TimerBase::SharedPtr timer_;
//     rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
//     size_t count_;
// };

// class PublisherNode2: public rclcpp::Node
// {
// public:
//     PublisherNode2()
//     : Node("PublisherNode2"),count_(0)
//     {
//         publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("mpu6050",10);
//         auto timer_callback2 =
//             [this]() -> void{
//                 //Extract current thread
//                 // auto curr_thread = string_thread_id();
//                 //Prep display message
//                 // RCLCPP_INFO(
//                 //     this->get_logger(), "\n<<THREAD %s>> Publishing",
//                 //     curr_thread.c_str());
//                 Eigen::Matrix<float,3,1> Data;
//                 Data = mpu6050_thigh.Read();
//                 // for (int i = 0; i < 3; i++)
//                 // {
//                 //     printf("Data: %d ",*(Data+i));
//                 // }
//                 // printf("\n");
//             };
//         timer_2 = this->create_wall_timer(0.000000868s,timer_callback2);
//     }
// private:
//     rclcpp::TimerBase::SharedPtr timer_2;
//     rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
//     size_t count_;
// };

struct Producer_Piezoelectric : public rclcpp::Node
{
    Producer_Piezoelectric(const std::string & name, const std::string & output)
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
         * @Descripton: Call back function for Reading Piezoelectric_Data.
        */
        auto callback = [captured_pub]() -> void {
            auto pub_ptr = captured_pub.lock();
            if (!pub_ptr) {
            return;
            }
            static int32_t count = 0;
            std_msgs::msg::Float64MultiArray::UniquePtr msg(new std_msgs::msg::Float64MultiArray());

            // Piezoelectric_Data = piezoelectric.Read();
            int * Data;
            Data = piezoelectric.Read_Data();

            // msg->data = {Piezoelectric_Data(0,0),Piezoelectric_Data(1,0),Piezoelectric_Data(2,0)};
            // for (int i = 0; i < 3; i++)
            // {
            //     printf("Data: %d ",*(Data+i));
            // }
            // printf("\n");
            // pub_ptr->publish(std::move(msg));
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

MPU6050 mpu6050 = MPU6050 ("/dev/ttyUSB1",B115200,"Left_Knee");

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

            Eigen::Matrix<float,3,1> Angle;
            Angle = mpu6050.Read();

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
// struct Producer_MPU6050_Calf : public rclcpp::Node
// {
//     Producer_MPU6050_Calf(const std::string & name, const std::string & output)
//     : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
//     {
//         // Create a publisher on the output topic.
//         /*
//          * @Name: create_publisher
//          * @Description: Create a Publisher based on the std_msgs::Msg::Float64MultiArray message and the topic is output.
//         */
//         pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(output, 10);
//         std::weak_ptr<std::remove_pointer<decltype(pub_.get())>::type> captured_pub = pub_;
//         /*
//          * @Name: callback
//          * @Descripton: Call back function for Reading Piezoelectric_Data.
//         */
//         auto callback = [captured_pub]() -> void {
//             auto pub_ptr = captured_pub.lock();
//             if (!pub_ptr) {
//             return;
//             }
//             static int32_t count = 0;
//             std_msgs::msg::Float64MultiArray::UniquePtr msg(new std_msgs::msg::Float64MultiArray());

//             MPU6050_Calf_Angle = mpu6050_calf.Read();

//             msg->data = {MPU6050_Calf_Angle(0,0),MPU6050_Calf_Angle(1,0),MPU6050_Calf_Angle(2,0)};
//             pub_ptr->publish(std::move(msg));
//         };

//         /*
//          * @Name: crate_wall_timer
//          * @Description: Create a timer, the input need initialize the time and call back function.
//          *              The time need to match with the MPU6050 sensor.
//          * @Input: time, callback
//          */
//         timer_ = this->create_wall_timer(0.000000868s, callback);
//     }

//     rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
//     rclcpp::TimerBase::SharedPtr timer_;
// };


// int  main(int argc, char * argv[])
// {
//     setvbuf(stdout, NULL, _IONBF, BUFSIZ);
//     rclcpp::init(argc, argv);
//     // rclcpp::executors::SingleThreadedExecutor executor_Piezoelectric;
//     // rclcpp::executors::SingleThreadedExecutor executor_MPU6050_Thigh;
//     // rclcpp::executors::SingleThreadedExecutor executor_MPU6050_Calf;
//     rclcpp::executors::MultiThreadedExecutor executor;
//     auto producer_piezoelectirc = std::make_shared<Producer_Piezoelectric>("producer_piezoelectric", "Piezoelectric");
//     // auto producer_mpu6050_Thigh = std::make_shared<Producer_MPU6050_Thigh>("producer_mpu6050_thigh", "MPU6050_Thigh");
//     // auto producer_mpu6050_calf  = std::make_shared<Producer_Piezoelectric>("producer_mpu6050_calf", "MPU6050_Calf");

//     // executor_Piezoelectric.add_node(producer_piezoelectirc);
//     // executor_Piezoelectric.spin();

//     // executor_MPU6050_Thigh.add_node(producer_mpu6050_Thigh);
//     // executor_MPU6050_Thigh.spin();

//     // executor_MPU6050_Calf.add_node(producer_mpu6050_calf);
//     // executor_MPU6050_Calf.spin();

//     // auto pubnode = std::make_shared<PublisherNode>();
//     // auto pubnode2 = std::make_shared<PublisherNode2>();
//     // executor.add_node(pubnode);

//     auto producer = std::make_shared<Producer>("producer", "MPU6050");

//     executor.add_node(producer);
//     executor.add_node(producer_piezoelectirc);
//     executor.spin();
//     rclcpp::shutdown();
//     return 0;
// }

int main(int argc, char * argv [])
{
    rclcpp::init(argc, argv);
    while (rclcpp::ok())
    {
        // int * Data;
        // Data = piezoelectric.Read_Data();
        // for (int i = 0; i < 3; i++)
        // {
        //     std::cout<<"Data: "<<*(Data+i);
        // }
        // std::cout<<std::endl;
        float * Data2;
        Data2 = mpu6050_thigh.Read_Data();
        // for (int i = 0; i < 3; i++)
        // {
        //     std::cout<<"Data: "<<*(Data2+i);
        // }
        // std::cout<<std::endl;
    }

}