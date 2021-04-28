#include <memory>
#include <fstream>
#include <string>
#include <iostream>
#include <streambuf>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using std::placeholders::_1;
using namespace std;

#define Joint_Error_Topic   "Joint_Error"

class Joint_Error :
    public rclcpp::Node
{
public:
    Joint_Error()
    : Node("Joint_Error")
    {
        //TODO: Change the Topic of Joint States
        Joint_Error_Subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            Joint_Error_Topic, 10, std::bind(&Joint_Error::callback, this, _1)
        );
    }
    ~Joint_Error()
    {

    }

private:
    void callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        auto position = msg->data;
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr   Joint_Error_Subscription;
    ofstream                                                            oFile;

    
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Joint_Error>());
    rclcpp::shutdown();
    return 0;
}