/*
 * @Author: MingshanHe 
 * @Date: 2021-12-05 04:09:41 
 * @Last Modified by: MingshanHe
 * @Last Modified time: 2021-12-05 04:18:32
 * @Licence: MIT Licence
 */
#include <pluginlib/class_list_macros.h>
#include "/home/zby/exo_ws/src/ros_controllers/pid_controller/include/joint_position_controller/pid_controller.hpp"


namespace pid_controller
{
/** \brief Initialize the kinematic chain for kinematics-based computation.
 *
 */
bool PID_Controller::init(
    hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n) {

    // get publishing period
    if (!n.getParam("publish_rate", publish_rate_)){
        ROS_ERROR("Parameter 'publish_rate' not set");
        return false;
    }

    if (!n.getParam("joints", joints))
    {
        ROS_ERROR_STREAM("hardware can't get robot joints!");
        return false;
    }

    for (size_t i = 0; i < joints.size(); i++)
    {
        joint_handles_.push_back(robot->getHandle(joints[i]));
    }

    sub_command_ = n.subscribe("command_joint_pos", 5,
        &PID_Controller::command_joint_pos, this,
        ros::TransportHints().reliable().tcpNoDelay());

    joint_position_command.resize(4);
    joint_position_state.resize(4);
    joint_velocity_state.resize(4);
    joint_effort_state.resize(4);
    return true;
}


void PID_Controller::starting(const ros::Time& time){
    for(std::size_t i=0; i < 4; i++) {
        joint_position_state[i] = 0.0;
        joint_velocity_state[i] = 0.0;
        joint_effort_state[i] = 0.0;
        plus1[i] = 0.0;
        plus2[i] = 0.0;
        plus3[i] = 0.0;
        error1[i] = 0.0;
        error2[i] = 0.0;
        error3[i] = 0.0;
        next_error1[i] = 0.0;
        next_error2[i] = 0.0;
        next_error3[i] = 0.0;
        last_error1[i] = 0.0;
        last_error2[i] = 0.0;
        last_error3[i] = 0.0;
        //casual parameters, we need to try to get a better set of PID parameters
        Kp = 0.1;
        Kd = 0.1;
        Ki = 0.1;
    }
// //   End_Vel_Cmd_ = KDL::Twist::Zero();
    last_publish_time_ = time;
}

void PID_Controller::update(const ros::Time& time, const ros::Duration& period) {
    for(std::size_t i=0; i < this->joint_handles_.size(); i++)
    {
        error1[i] = joint_handles_[i].getPosition()-joint_position_state[i];
        plus1[i] = Kp*(error1[i]-next_error1[i])+Ki*error1[i]+Kd*(error1[i]-2*next_error1[i]+last_error1[i]);
        joint_position_state[i+1] += plus1[i];
        last_error1[i+1]=next_error1[i];
        next_error1[i+1]=error1[i]; 

        error2[i] = joint_handles_[i].getVelocity()-joint_velocity_state[i];
        plus2[i] = Kp*(error2[i]-next_error2[i])+Ki*error2[i]+Kd*(error2[i]-2*next_error2[i]+last_error2[i]);
        joint_velocity_state[i+1] += plus2[i];
        last_error2[i+1]=next_error2[i];
        next_error2[i+1]=error2[i]; 

        error3[i] = joint_handles_[i].getEffort()-joint_effort_state[i];
        plus3[i] = Kp*(error3[i]-next_error3[i])+Ki*error3[i]+Kd*(error3[i]-2*next_error3[i]+last_error3[i]);
        joint_effort_state[i+1] += plus3[i];
        last_error3[i+1]=next_error3[i];
        next_error3[i+1]=error3[i]; 

        // joint_position_state[i] = joint_handles_[i].getPosition();
        // joint_velocity_state[i] = joint_handles_[i].getVelocity();
        // joint_effort_state[i] = joint_handles_[i].getEffort();
    }

    writePositionCommands(period);


    // Limit rate of publishing
    if (publish_rate_ > 0.0 && last_publish_time_
        + ros::Duration(1.0/publish_rate_) < time) {

        // try to publish
        // if (realtime_pub_->trylock()) {
        // last_publish_time_ = last_publish_time_
        //                     + ros::Duration(1.0/publish_rate_);
        // // populate message
        // // realtime_pub_->msg_.header.stamp = time;
        // // tf::poseKDLToMsg(End_Pos_, realtime_pub_->msg_.pose);
        // // tf::twistKDLToMsg(End_Vel_.GetTwist(), realtime_pub_->msg_.twist);

        // // realtime_pub_->unlockAndPublish();
        // }
    }
}

/*!
 * \brief Subscriber's callback: copies twist commands
 */
void PID_Controller::command_joint_pos(const geometry_msgs::Quaternion &msg) {
    joint_position_command[0] = msg.x;
    joint_position_command[1] = msg.y;
    joint_position_command[2] = msg.z;
    joint_position_command[3] = msg.w;

}


/********************************************/
/**FUNCTIONS OF INSTANCES OF THE BASE CLASS**/
/********************************************/

/** \brief write the desired velocity command in the hardware interface input
 * for a VelocityJointInterface
 * \param period The duration of an update cycle
 */
void PID_Controller::writePositionCommands(
                                    const ros::Duration& period) {
    for(std::size_t i=0; i < this->joint_handles_.size(); i++) {
      this->joint_handles_[i].setCommand(joint_position_command[i]);
    }
}

} // controller_interface namespace

// Register controllers with the PLUGINLIB_EXPORT_CLASS macro to enable dynamic
// loading with the controller manager
PLUGINLIB_EXPORT_CLASS(pid_controller::PID_Controller, controller_interface::ControllerBase)
