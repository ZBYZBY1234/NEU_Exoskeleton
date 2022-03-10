#ifndef RBF_CONTROLLER_H
#define RBF_CONTROLLER_H

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <vector>
#include "geometry_msgs/Quaternion.h"
namespace rbf_controller
{

/** \brief This class implements a ROS control cartesian velocity
 * controller. Its base class implements the core
 * of the controller.
 */
class RBF_Controller: public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
public:
    RBF_Controller() {}
    ~RBF_Controller() {}

    bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n);

    void starting(const ros::Time& time);

    void update(const ros::Time& time, const ros::Duration& period);

    void command_joint_pos(const geometry_msgs::Quaternion &msg);

    void caculateNorm2(std::vector<double*> ,int);

    void caculateHj(std::vector<double*> ,std::vector<double*> ,int ,double );

    void caculateYpY(std::vector<double*> ,std::vector<double*> ,int );

private:

    void writePositionCommands(const ros::Duration& period);

protected:
    ros::Subscriber                                 sub_command_; // Interface to external commands

    ros::Time                                       last_publish_time_;
    double                                          publish_rate_;

    std::vector<hardware_interface::JointHandle>    joint_handles_;
    std::vector<std::string>                        joints;
    std::vector<double>                             joint_position_state;
    std::vector<double>                             joint_velocity_state;
    std::vector<double>                             joint_effort_state;
    std::vector<double>                             joint_position_command;
    
    std::vector<double>                             plus1;
    std::vector<double>                             plus2;
    std::vector<double>                             plus3;    

    std::vector<double>                             error1;
    std::vector<double>                             last_error1;
    std::vector<double>                             next_error1;
    std::vector<double>                             error2;
    std::vector<double>                             last_error2;
    std::vector<double>                             next_error2;
    std::vector<double>                             error3;
    std::vector<double>                             last_error3;
    std::vector<double>                             next_error3;

    double                                          Kp,Ki,Kd,Kp_init,Ki_init,Kd_init;
    
    double                                           eta,alpha;//学习速率，动量因子
    double                                           etaP,etaI,etaD;
    
    std::vector<double*>                             c;//中心矢量
    std::vector<double*>                             b;//基函数宽度
    std::vector<double*>                             w;//weight vector
    std::vector<double*>                             h;//hiden vector
    std::vector<double*>                             d_w;
    std::vector<double*>                             d_b;
    std::vector<double*>                             d_c;
    std::vector<double*>                             w1;
    std::vector<double*>                             w2;
    std::vector<double*>                             b1;
    std::vector<double*>                             b2;
    std::vector<double*>                             c1;
    std::vector<double*>                             c2;
                             
    std::vector<double>                              y;
    std::vector<double>                              yu;
    std::vector<double*>                             xc;
    std::vector<double>                              error11;
    std::vector<double>                              error12;
    std::vector<double>                              error21;
    std::vector<double>                              error22;
    std::vector<double>                              error31;
    std::vector<double>                              error32;


    // boost::shared_ptr<realtime_tools::RealtimePublisher<

    // boost::shared_ptr<realtime_tools::RealtimePublisher<
    // boost::shared_ptr<realtime_tools::RealtimePublisher<
    //     cartesian_state_msgs::PoseTwist> > realtime_pub_;
};
} // namespace controller_interface


#endif
