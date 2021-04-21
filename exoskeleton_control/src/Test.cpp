#include<stdio.h>
#include<unistd.h>
#include<sys/types.h>
#include<string.h>
#include<sys/stat.h>
#include<fcntl.h>
#define _PATH_NAME_ "/home/hemingshan/exo_ws/src/exoskeleton_control/Pipe_File/file.tmp"
#define _SIZE_ 100

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */




int fd=open(_PATH_NAME_,O_WRONLY);

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
        ret=mkfifo(_PATH_NAME_,S_IFIFO|0666);
        if(ret==-1){
            printf("make fifo error\n");
        }
        memset(buf,'\0',sizeof(buf));

        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
        fgets(buf,sizeof(buf)-1,stdin);
        int ret=write(fd,buf,strlen(buf)+1);
        if(ret<0){
            printf("write error");
        }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
    int ret;
    char buf[_SIZE_];
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    close(fd);
    return 0;
}