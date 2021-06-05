#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <exoskeleton_param/JointState_self.h>

class Joint_State
{
public:
    Joint_State(){
        joint_state_subscriber = nh_.subscribe("lower_limb/joint_states", 1, &Joint_State::CallBack, this);
        Left_Hip_Joint_position_publisher  = nh_.advertise<std_msgs::Float64>("Left_Hip_Joint_Position", 1);
        Left_Hip_Joint_velocity_publisher  = nh_.advertise<std_msgs::Float64>("Left_Hip_Joint_Velocity", 1);
        Left_Hip_Joint_effort_publisher    = nh_.advertise<std_msgs::Float64>("Left_Hip_Joint_Effort", 1);
        Left_Thigh_Joint_position_publisher  = nh_.advertise<std_msgs::Float64>("Left_Thigh_Joint_Position", 1);
        Left_Thigh_Joint_velocity_publisher  = nh_.advertise<std_msgs::Float64>("Left_Thigh_Joint_Velocity", 1);
        Left_Thigh_Joint_effort_publisher    = nh_.advertise<std_msgs::Float64>("Left_Thigh_Joint_Effort", 1);
        Left_Calf_Joint_position_publisher  = nh_.advertise<std_msgs::Float64>("Left_Calf_Joint_Position", 1);
        Left_Calf_Joint_velocity_publisher  = nh_.advertise<std_msgs::Float64>("Left_Calf_Joint_Velocity", 1);
        Left_Calf_Joint_effort_publisher    = nh_.advertise<std_msgs::Float64>("Left_Calf_Joint_Effort", 1);
        Left_Ankle_Joint_position_publisher  = nh_.advertise<std_msgs::Float64>("Left_Ankle_Joint_Position", 1);
        Left_Ankle_Joint_velocity_publisher  = nh_.advertise<std_msgs::Float64>("Left_Ankle_Joint_Velocity", 1);
        Left_Ankle_Joint_effort_publisher    = nh_.advertise<std_msgs::Float64>("Left_Ankle_Joint_Effort", 1);

        Right_Hip_Joint_position_publisher  = nh_.advertise<std_msgs::Float64>("Right_Hip_Joint_Position", 1);
        Right_Hip_Joint_velocity_publisher  = nh_.advertise<std_msgs::Float64>("Right_Hip_Joint_Velocity", 1);
        Right_Hip_Joint_effort_publisher    = nh_.advertise<std_msgs::Float64>("Right_Hip_Joint_Effort", 1);
        Right_Thigh_Joint_position_publisher  = nh_.advertise<std_msgs::Float64>("Right_Thigh_Joint_Position", 1);
        Right_Thigh_Joint_velocity_publisher  = nh_.advertise<std_msgs::Float64>("Right_Thigh_Joint_Velocity", 1);
        Right_Thigh_Joint_effort_publisher    = nh_.advertise<std_msgs::Float64>("Right_Thigh_Joint_Effort", 1);
        Right_Calf_Joint_position_publisher  = nh_.advertise<std_msgs::Float64>("Right_Calf_Joint_Position", 1);
        Right_Calf_Joint_velocity_publisher  = nh_.advertise<std_msgs::Float64>("Right_Calf_Joint_Velocity", 1);
        Right_Calf_Joint_effort_publisher    = nh_.advertise<std_msgs::Float64>("Right_Calf_Joint_Effort", 1);
        Right_Ankle_Joint_position_publisher  = nh_.advertise<std_msgs::Float64>("Right_Ankle_Joint_Position", 1);
        Right_Ankle_Joint_velocity_publisher  = nh_.advertise<std_msgs::Float64>("Right_Ankle_Joint_Velocity", 1);
        Right_Ankle_Joint_effort_publisher    = nh_.advertise<std_msgs::Float64>("Right_Ankle_Joint_Effort", 1);
    };
    ~Joint_State(){};
public:
    void CallBack(const sensor_msgs::JointState::ConstPtr& input)
    {
        float a,b,c;
        std_msgs::Float64 L_A_P, L_A_V, L_A_E;
        a = input->position[0];
        b = input->velocity[0];
        c = input->effort[0];
        L_A_P.data = a;
        L_A_V.data = b;
        L_A_E.data = c;

        std_msgs::Float64 L_C_P, L_C_V, L_C_E;
        a = input->position[1];
        b = input->velocity[1];
        c = input->effort[1];
        L_C_P.data = a;
        L_C_V.data = b;
        L_C_E.data = c;

        std_msgs::Float64 L_T_P, L_T_V, L_T_E;
        a = input->position[2];
        b = input->velocity[2];
        c = input->effort[2];
        L_T_P.data = a;
        L_T_V.data = b;
        L_T_E.data = c;

        std_msgs::Float64 L_H_P, L_H_V, L_H_E;
        a = input->position[3];
        b = input->velocity[3];
        c = input->effort[3];
        L_H_P.data = a;
        L_H_V.data = b;
        L_H_E.data = c;

        std_msgs::Float64 R_A_P, R_A_V, R_A_E;
        a = input->position[4];
        b = input->velocity[4];
        c = input->effort[4];
        R_A_P.data = a;
        R_A_V.data = b;
        R_A_E.data = c;

        std_msgs::Float64 R_C_P, R_C_V, R_C_E;
        a = input->position[5];
        b = input->velocity[5];
        c = input->effort[5];
        R_C_P.data = a;
        R_C_V.data = b;
        R_C_E.data = c;

        std_msgs::Float64 R_T_P, R_T_V, R_T_E;
        a = input->position[6];
        b = input->velocity[6];
        c = input->effort[6];
        R_T_P.data = a;
        R_T_V.data = b;
        R_T_E.data = c;

        std_msgs::Float64 R_H_P, R_H_V, R_H_E;
        a = input->position[7];
        b = input->velocity[7];
        c = input->effort[7];
        R_H_P.data = a;
        R_H_V.data = b;
        R_H_E.data = c;

        Left_Ankle_Joint_position_publisher.publish(L_A_P);
        Left_Ankle_Joint_velocity_publisher.publish(L_A_V);
        Left_Ankle_Joint_effort_publisher.publish(L_A_E);
        Left_Calf_Joint_position_publisher.publish(L_C_P);
        Left_Calf_Joint_velocity_publisher.publish(L_C_V);
        Left_Calf_Joint_effort_publisher.publish(L_C_E);  
        Left_Thigh_Joint_position_publisher.publish(L_T_P);
        Left_Thigh_Joint_velocity_publisher.publish(L_T_V);
        Left_Thigh_Joint_effort_publisher.publish(L_T_E);  
        Left_Hip_Joint_position_publisher.publish(L_H_P);
        Left_Hip_Joint_velocity_publisher.publish(L_H_V);
        Left_Hip_Joint_effort_publisher.publish(L_H_E);  

        Right_Ankle_Joint_position_publisher.publish(R_A_P);
        Right_Ankle_Joint_velocity_publisher.publish(R_A_V);
        Right_Ankle_Joint_effort_publisher.publish(R_A_E);
        Right_Calf_Joint_position_publisher.publish(R_C_P);
        Right_Calf_Joint_velocity_publisher.publish(R_C_V);
        Right_Calf_Joint_effort_publisher.publish(R_C_E);  
        Right_Thigh_Joint_position_publisher.publish(R_T_P);
        Right_Thigh_Joint_velocity_publisher.publish(R_T_V);
        Right_Thigh_Joint_effort_publisher.publish(R_T_E);  
        Right_Hip_Joint_position_publisher.publish(R_H_P);
        Right_Hip_Joint_velocity_publisher.publish(R_H_V);
        Right_Hip_Joint_effort_publisher.publish(R_H_E);    
          
    }
private:
ros::NodeHandle         nh_;
ros::Subscriber         joint_state_subscriber;

ros::Publisher          Left_Hip_Joint_position_publisher;
ros::Publisher          Left_Hip_Joint_velocity_publisher;
ros::Publisher          Left_Hip_Joint_effort_publisher;
ros::Publisher          Left_Thigh_Joint_position_publisher;
ros::Publisher          Left_Thigh_Joint_velocity_publisher;
ros::Publisher          Left_Thigh_Joint_effort_publisher;
ros::Publisher          Left_Calf_Joint_position_publisher;
ros::Publisher          Left_Calf_Joint_velocity_publisher;
ros::Publisher          Left_Calf_Joint_effort_publisher;
ros::Publisher          Left_Ankle_Joint_position_publisher;
ros::Publisher          Left_Ankle_Joint_velocity_publisher;
ros::Publisher          Left_Ankle_Joint_effort_publisher;

ros::Publisher          Right_Hip_Joint_position_publisher;
ros::Publisher          Right_Hip_Joint_velocity_publisher;
ros::Publisher          Right_Hip_Joint_effort_publisher;
ros::Publisher          Right_Thigh_Joint_position_publisher;
ros::Publisher          Right_Thigh_Joint_velocity_publisher;
ros::Publisher          Right_Thigh_Joint_effort_publisher;
ros::Publisher          Right_Calf_Joint_position_publisher;
ros::Publisher          Right_Calf_Joint_velocity_publisher;
ros::Publisher          Right_Calf_Joint_effort_publisher;
ros::Publisher          Right_Ankle_Joint_position_publisher;
ros::Publisher          Right_Ankle_Joint_velocity_publisher;
ros::Publisher          Right_Ankle_Joint_effort_publisher;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Joint_State");

    Joint_State joint_state;
    ros::spin();
    return 0;
}