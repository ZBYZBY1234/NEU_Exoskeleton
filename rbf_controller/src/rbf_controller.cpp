#include <pluginlib/class_list_macros.h>
#include "/home/zby/exo_ws/src/ros_controllers/rbf_controller/include/rbf_controller/rbf_controller.hpp"
#include "yaml-cpp/yaml.h"
#include <cmath>


namespace rbf_controller
{
/** \brief Initialize the kinematic chain for kinematics-based computation.
 *
 */
bool RBF_Controller::init(
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
        &RBF_Controller::command_joint_pos, this,
        ros::TransportHints().reliable().tcpNoDelay());

    joint_position_command.resize(4);
    joint_position_state.resize(4);
    joint_velocity_state.resize(4);
    joint_effort_state.resize(4);
    return true;
}


void RBF_Controller::starting(const ros::Time& time){
    YAML::Node rbf_param = YAML::LoadFile("/home/zby/exo_ws/src/ros_controllers/rbf_controller/config/rbf_param.yaml");
    for(std::size_t i=0; i < 4; i++) {
        joint_position_state[i] = 0.0;
        // joint_velocity_state[i] = 0.0;
        // joint_effort_state[i] = 0.0;
        plus1[i] = 0.0;
        // plus2[i] = 0.0;
        // plus3[i] = 0.0;
        error1[i] = 0.0;
        // error2[i] = 0.0;
        // error3[i] = 0.0;
        next_error1[i] = 0.0;
        // next_error2[i] = 0.0;
        // next_error3[i] = 0.0;
        last_error1[i] = 0.0;
        // last_error2[i] = 0.0;
        // last_error3[i] = 0.0;
        for(std::size_t j=0;j<7;j++)
        {
            h[j][i]=0.0;
            w1[j][i]=0.0;
            w2[j][i]=0.0;
            c1[j][i]=0.0;
            c2[j][i]=0.0;
            b1[j][i]=0.0;
            b1[j][i]=0.0;
            c[j][i]=rbf_param["c"].as<double>();
            b[j][i]=rbf_param["b"].as<double>();
        }

        
        Kp_init = rbf_param["Kp"].as<double>();
        Ki_init = rbf_param["Ki"].as<double>();
        Kd_init = rbf_param["Kd"].as<double>();
        eta = rbf_param["eta"].as<double>();
        alpha = rbf_param["alpha"].as<double>();
        etaP = rbf_param["etaP"].as<double>();
        etaI = rbf_param["etaI"].as<double>();
        etaD = rbf_param["etaD"].as<double>();
        

    }
// //   End_Vel_Cmd_ = KDL::Twist::Zero();
    last_publish_time_ = time;
}

void RBF_Controller::update(const ros::Time& time, const ros::Duration& period) {
    for(std::size_t i=0; i < this->joint_handles_.size(); i++)
    {
        for(std::size_t j=0;j<7;j++)
        {
        h[j][i]=exp(-pow(joint_position_state[i]-c[j][i],2)/(2*pow(b[j][i],2)));
        y[i] += h[j][i]*w[j][i];
        
        d_w[j][i]=eta*(joint_handles_[i].getPosition()-y[i])*h[j][i];
        w[j][i]=w1[j][i]+d_w[j][i]+alpha*(w1[j][i]-w2[j][i]);
        
        d_b[j][i]=eta*(joint_handles_[i].getPosition()-y[i])*w[j][i]*h[j][i]*pow(b[j][i],-3)*pow(joint_position_state[i]-c[j][i],2);
        b[j][i]=b1[j][i]+eta*d_b[j][i]+alpha*(b1[j][i]-b2[j][i]);
        
        d_c[j][i]=eta*(joint_handles_[i].getPosition()-y[i])*w[j][i]*h[j][i]*(joint_position_state[i]-c[j][i])*pow(b[j][i],-2);
        c[j][i]=c1[j][i]+d_c[j][i]+alpha*(c1[j][i]-c2[j][i]);

        w2[j][i]=w1[j][i];
        w1[j][i]=w[j][i];
        b2[j][i]=b1[j][i];
        b1[j][i]=b[j][i];
        c2[j][i]=c1[j][i];
        c1[j][i]=c[j][i];

        yu[i] += w[j][i]*h[j][i]*(-joint_position_state[i]+c[j][i])/pow(b[j][i],2);
        error1[i] = joint_handles_[i].getPosition()-joint_position_state[i];
        Kp=Kp_init+etaP*error1[i]*yu[i]*(error1[i]-next_error1[i]);
        Ki=Ki_init+etaI*error1[i]*yu[i]*error1[i];
        Kd=Kd_init+etaD*error1[i]*yu[i]*(error1[i]-2*next_error1[i]+last_error1[i]);
        
        plus1[i] = Kp*(error1[i]-next_error1[i])+Ki*error1[i]+Kd*(error1[i]-2*next_error1[i]+last_error1[i]);
        joint_position_state[i] += plus1[i];
        last_error1[i]=next_error1[i];
        next_error1[i]=error1[i]; 
        
        Kp_init=Kp;
        Ki_init=Ki;
        Kd_init=Kd;

        }
        // error2[i] = joint_handles_[i].getVelocity()-joint_velocity_state[i];
        // plus2[i] = Kp*(error2[i]-next_error2[i])+Ki*error2[i]+Kd*(error2[i]-2*next_error2[i]+last_error2[i]);
        // joint_velocity_state[i] += plus2[i];
        // last_error2[i]=next_error2[i];
        // next_error2[i]=error2[i]; 

        // error3[i] = joint_handles_[i].getEffort()-joint_effort_state[i];
        // plus3[i] = Kp*(error3[i]-next_error3[i])+Ki*error3[i]+Kd*(error3[i]-2*next_error3[i]+last_error3[i]);
        // joint_effort_state[i] += plus3[i];
        // last_error3[i]=next_error3[i];
        // next_error3[i]=error3[i]; 

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
void RBF_Controller::command_joint_pos(const geometry_msgs::Quaternion &msg) {
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
void RBF_Controller::writePositionCommands(
                                    const ros::Duration& period) {
    for(std::size_t i=0; i < this->joint_handles_.size(); i++) {
      this->joint_handles_[i].setCommand(joint_position_command[i]);
    }
}

} // controller_interface namespace

// Register controllers with the PLUGINLIB_EXPORT_CLASS macro to enable dynamic
// loading with the controller manager
PLUGINLIB_EXPORT_CLASS(rbf_controller::RBF_Controller, controller_interface::ControllerBase)
