/*
 * @Author: MingshanHe
 * @Email: hemingshan_1999@163.com
 * @Date: 2021-01-15 20:09:00
 * @Last Modified by: MingshanHe
 * @Last Modified time: 2021-01-15 20:11:14
 * @Description: Calculate the center of gravity of the robot roughly(approximately)
 */
#include<iostream>
#include<ros/ros.h>
#include<tf/tf.h>
#include<tf/transform_listener.h>
#include<geometry_msgs/TransformStamped.h>
#include<std_msgs/Float64.h>

using namespace std;

class TF_Listener
{
public:
    TF_Listener(){
        base_m  = 7.1666344893724000;
        hip_m   = 0.6272795491670899;
        thigh_m = 1.9901518674612200;
        calf_m  = 1.6727541009136200;
        ankle_m = 1.1903645092283100;

        center_point_pub = nh.advertise<std_msgs::Float64>("center_point",10);
    };
    ~TF_Listener(){};
public:
    void Transform_Link()
    {
        try
        {
            // Base_Link To Hip_Link
            tf_listener.lookupTransform(Base_Link, Child_Link1[0], ros::Time(0), transform);
            ROS_INFO("%s To %s",Base_Link,Child_Link1[0]);
            ROS_INFO("x,y,z = %f, %f, %f", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
            ROS_INFO("w,y = %f, %f", transform.getRotation().getW(), transform.getRotation().getY());

            //Hip_Link To Thigh_Link
            tf_listener.lookupTransform(Base_Link, Child_Link2[0], ros::Time(0), transform);
            ROS_INFO("%s To %s",Base_Link,Child_Link2[0]);
            ROS_INFO("x,y,z = %f, %f, %f", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
            ROS_INFO("w,y = %f, %f", transform.getRotation().getW(), transform.getRotation().getY());

            //Thigh_Link To Calf_Link
            tf_listener.lookupTransform(Base_Link, Child_Link3[0], ros::Time(0), transform);
            ROS_INFO("%s To %s",Base_Link,Child_Link3[0]);
            ROS_INFO("x,y,z = %f, %f, %f", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
            ROS_INFO("w,y = %f, %f", transform.getRotation().getW(), transform.getRotation().getY());

            //Calf_Link To Ankle_Link
            tf_listener.lookupTransform(Base_Link, Child_Link4[0], ros::Time(0), transform);
            ROS_INFO("%s To %s",Base_Link,Child_Link4[0]);
            ROS_INFO("x,y,z = %f, %f, %f", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
            ROS_INFO("w,y = %f, %f", transform.getRotation().getW(), transform.getRotation().getY());
        }
        catch(const std::exception& e)
        {
            ROS_ERROR("%s", e.what());
        }
    }

    void Center_Gravity_Link()
    {
        try
        {
            // Center of Gravity of Back_Base_Link
            Back_Base_Link_x = 0;
            Back_Base_Link_y = 0;
            Back_Base_Link_z = 0;
            
            // Center of Gravity of Hip_Link
            tf_listener.lookupTransform(Base_Link, Child_Link1[0], ros::Time(0), transform);
            Left_Hip_Link_x = transform.getOrigin().x();
            Left_Hip_Link_y = transform.getOrigin().y();
            Left_Hip_Link_z = transform.getOrigin().z();

            tf_listener.lookupTransform(Base_Link, Child_Link1[1], ros::Time(0), transform);
            Right_Hip_Link_x = transform.getOrigin().x();
            Right_Hip_Link_y = transform.getOrigin().y();
            Right_Hip_Link_z = transform.getOrigin().z();

            //Center of Gravity of Thigh_Link
            tf_listener.lookupTransform(Base_Link, Child_Link2[0], ros::Time(0), transform);
            tf_listener.lookupTransform(Base_Link, Child_Link3[0], ros::Time(0), transform_);
            Left_Thigh_Link_x = (transform.getOrigin().x()+transform_.getOrigin().x())/2;
            Left_Thigh_Link_y = (transform.getOrigin().y()+transform_.getOrigin().y())/2;
            Left_Thigh_Link_z = (transform.getOrigin().z()+transform_.getOrigin().x())/2;
            
            tf_listener.lookupTransform(Base_Link, Child_Link2[1], ros::Time(0), transform);
            tf_listener.lookupTransform(Base_Link, Child_Link3[1], ros::Time(0), transform_);
            Right_Thigh_Link_x = (transform.getOrigin().x()+transform_.getOrigin().x())/2;
            Right_Thigh_Link_y = (transform.getOrigin().y()+transform_.getOrigin().y())/2;
            Right_Thigh_Link_z = (transform.getOrigin().z()+transform_.getOrigin().x())/2;

            //Center of Gravity of Calf_Link
            tf_listener.lookupTransform(Base_Link, Child_Link3[0], ros::Time(0), transform);
            tf_listener.lookupTransform(Base_Link, Child_Link4[0], ros::Time(0), transform_);
            Left_Calf_Link_x = (transform.getOrigin().x()+transform_.getOrigin().x())/2;
            Left_Calf_Link_y = (transform.getOrigin().y()+transform_.getOrigin().y())/2;
            Left_Calf_Link_z = (transform.getOrigin().z()+transform_.getOrigin().x())/2;
            
            tf_listener.lookupTransform(Base_Link, Child_Link3[1], ros::Time(0), transform);
            tf_listener.lookupTransform(Base_Link, Child_Link4[1], ros::Time(0), transform_);
            Right_Calf_Link_x = (transform.getOrigin().x()+transform_.getOrigin().x())/2;
            Right_Calf_Link_y = (transform.getOrigin().y()+transform_.getOrigin().y())/2;
            Right_Calf_Link_z = (transform.getOrigin().z()+transform_.getOrigin().x())/2;

            //Center of Gravity of Ankle_Link
            tf_listener.lookupTransform(Base_Link, Child_Link4[0], ros::Time(0), transform);
            Left_Ankle_Link_x = transform.getOrigin().x();
            Left_Ankle_Link_y = transform.getOrigin().y();
            Left_Ankle_Link_z = transform.getOrigin().z();

            tf_listener.lookupTransform(Base_Link, Child_Link4[1], ros::Time(0), transform);
            Right_Ankle_Link_x = transform.getOrigin().x();
            Right_Ankle_Link_y = transform.getOrigin().y();
            Right_Ankle_Link_z = transform.getOrigin().z();
            ROS_INFO("Thigh: %f",Left_Thigh_Link_x);
            ROS_INFO("Calf: %f",Left_Calf_Link_x);
        }
        catch(const std::exception& e)
        {
            ROS_ERROR("%s", e.what());
        }
    }

    void Center_Gravity_Robot()
    {
        float a = hip_m * (Left_Hip_Link_x + Right_Hip_Link_x);
        float b = thigh_m * (Left_Thigh_Link_x + Right_Thigh_Link_x);
        float c = calf_m * (Left_Calf_Link_x + Right_Calf_Link_x);
        float d = ankle_m * (Left_Ankle_Link_x + Right_Ankle_Link_x);
        Robot_x = (a+b+c+d)/(base_m + hip_m*2 + thigh_m*2 + calf_m*2 + ankle_m*2);
        ROS_INFO("The center of gravity of robot: x: %f", Robot_x);
        std_msgs::Float64 center_point;
        center_point.data = Robot_x;
        center_point_pub.publish(center_point);
        ROS_INFO("The center of gravity of ankle: x: %f", Left_Ankle_Link_x);
    }

private:
tf::TransformListener       tf_listener;
tf::StampedTransform        transform;
tf::StampedTransform        transform_;

char*                       Base_Link = {"Back_Base_Link"};        //Base_Back_Link
char*                       Child_Link1[2]= {"Left_Hip_Link","Right_Hip_Link"};   //Hip_Link
char*                       Child_Link2[2]= {"Left_Thigh_Link","Right_Thigh_Link"};  //Thigh_Link
char*                       Child_Link3[2]= {"Left_Calf_Link","Right_Calf_Link"};   //Calf_Link
char*                       Child_Link4[2]= {"Left_Ankle_Link","Right_Ankle_Link"};   //Ankle_Link

float                       Back_Base_Link_x, Back_Base_Link_y, Back_Base_Link_z;
float                       Left_Hip_Link_x, Left_Hip_Link_y, Left_Hip_Link_z;
float                       Left_Thigh_Link_x, Left_Thigh_Link_y, Left_Thigh_Link_z;
float                       Left_Calf_Link_x, Left_Calf_Link_y, Left_Calf_Link_z;
float                       Left_Ankle_Link_x, Left_Ankle_Link_y, Left_Ankle_Link_z;
float                       Right_Hip_Link_x, Right_Hip_Link_y, Right_Hip_Link_z;
float                       Right_Thigh_Link_x, Right_Thigh_Link_y, Right_Thigh_Link_z;
float                       Right_Calf_Link_x, Right_Calf_Link_y, Right_Calf_Link_z;
float                       Right_Ankle_Link_x, Right_Ankle_Link_y, Right_Ankle_Link_z;

float                       Robot_x, Robot_y, Robot_z;

float                       base_m;
float                       hip_m;
float                       thigh_m;
float                       calf_m;
float                       ankle_m;

ros::NodeHandle             nh;
ros::Publisher              center_point_pub;
};
int main(int argc, char** argv){
    ros::init(argc, argv, "tf_sub_test");
    
    TF_Listener tf_listener;
    ros::Rate rate(50);
    while(ros::ok()){
        tf_listener.Center_Gravity_Link();
        tf_listener.Center_Gravity_Robot();
        
        rate.sleep();
    }
    return 0;
}
