#include "device_hardware/device_hardware.h"

namespace device_hardware
{
    bool device_hardware::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
    {

        if (!robot_hw_nh.getParam("joint_trajectory_controller/joints", Robot.joints))
        {
            ROS_ERROR_STREAM("hardware can't get robot joints!");
            return false;
        }

        Robot.joints_num = Robot.joints.size();

        // Initialize raw data
        Robot.joint_position_state_.resize(Robot.joints_num);
        Robot.joint_velocity_state_.resize(Robot.joints_num);
        Robot.joint_effort_state_.resize(Robot.joints_num);

        Robot.joint_position_command_.resize(Robot.joints_num);
        Robot.joint_velocity_command_.resize(Robot.joints_num);
        Robot.joint_effort_command_.resize(Robot.joints_num);

        Robot.joint_name_.resize(Robot.joints_num);

        Robot.pj_handle_.resize(Robot.joints_num);
        Robot.vj_handle_.resize(Robot.joints_num);
        Robot.ej_handle_.resize(Robot.joints_num);

        // Robot.joint_limit_.resize(Robot.joints_num);
        // Robot.joint_solft_limit_.resize(Robot.joints_num);


        for (size_t i=0;i<Robot.joints_num;i++)
        {
            Robot.joint_name_[i] = Robot.joints[i];
            Robot.joint_position_state_[i]   = 0.0;
            Robot.joint_velocity_state_[i]   = 0.0;
            Robot.joint_effort_state_[i]     = 0.0;
            Robot.joint_position_command_[i] = Robot.joint_position_state_[i];
            Robot.joint_velocity_command_[i] = Robot.joint_velocity_state_[i];
            Robot.joint_effort_command_[i]   = Robot.joint_effort_state_[i];

            // Populate hardware interfaces
            js_interface_.registerHandle(
                hardware_interface::JointStateHandle(
                                Robot.joint_name_[i],
                                &(Robot.joint_position_state_[i]),
                                &(Robot.joint_velocity_state_[i]),
                                &(Robot.joint_effort_state_[i])));

            Robot.pj_handle_[i] = hardware_interface::JointHandle(
                js_interface_.getHandle(Robot.joints[i]),
                &(Robot.joint_position_command_[i]));

            Robot.vj_handle_[i] = hardware_interface::JointHandle(
                js_interface_.getHandle(Robot.joints[i]),
                &(Robot.joint_velocity_command_[i]));

            Robot.ej_handle_[i] = hardware_interface::JointHandle(
                js_interface_.getHandle(Robot.joints[i]),
                &(Robot.joint_effort_command_[i]));

            pj_interface_.registerHandle(Robot.pj_handle_[i]);
            vj_interface_.registerHandle(Robot.vj_handle_[i]);
            ej_interface_.registerHandle(Robot.ej_handle_[i]);
        }

        //initial

        init_hardware();

        std::vector<double> position_tmp;
        position_tmp.resize(4);
        position_tmp = read_position();
        write_position(position_tmp);
        for (size_t j = 0;j < Robot.joints_num;j++)
        {
            Robot.joint_position_command_[j] = position_tmp[j];
            Robot.joint_position_state_[j] = position_tmp[j];
        }


        registerInterface(&js_interface_);
        registerInterface(&pj_interface_);
        registerInterface(&vj_interface_);
        registerInterface(&ej_interface_);

        position_tmp.resize(Robot.joints_num);
        ROS_INFO("device initialize finish");
        return true;
    }

    void device_hardware::read(const ros::Time& time, const ros::Duration& period)
    {
        position_tmp = read_position();
        for(size_t i = 0; i < Robot.joints_num; i++)
        {
            Robot.joint_position_state_[i] = position_tmp[i];
        }

    }

    void device_hardware::write(const ros::Time& time, const ros::Duration& period)
    {
        for(size_t i = 0; i < Robot.joints_num; i++)
        {
            position_tmp[i] = Robot.joint_position_command_[i];
        }
    }


}

PLUGINLIB_EXPORT_CLASS( device_hardware::device_hardware, hardware_interface::RobotHW)