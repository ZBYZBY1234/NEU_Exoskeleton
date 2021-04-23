#include <iostream>
#include "exoskeleton_kinematics/exoskeleton_kinematics.hpp"


int main()
{
    Exoskeleton_kinetic exoskeleton_kinetic;
    Eigen::Matrix<float,4,4> T_end;
    Eigen::Matrix<float,1,6> KPS6;
    T_end = exoskeleton_kinetic.forward_kinematics(0.1,0.1,0.1,0.1);
    Eigen::Matrix<float,4,1> angle = exoskeleton_kinetic.inverse_kinematics(T_end);
    KPS6  = exoskeleton_kinetic.KPS44_KPS6(T_end);
}