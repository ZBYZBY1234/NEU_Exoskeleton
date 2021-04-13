#include <iostream>
#include <eigen3/Eigen/Dense>
#include <math.h>

#define PI 3.1415926
class Exoskeleton_dynamic
{
    public:
    Exoskeleton_dynamic();
    ~Exoskeleton_dynamic()
    {
        // std::cout<<"Clear Exoskeleton_dynamic class"<<std::endl;
    }
    public:
    Eigen::Matrix<float,6,1> Admittance_control(Eigen::Matrix<float,6,1> dxe_, Eigen::Matrix<float,6,1> xe_, Eigen::Matrix<float,6,1> Fe_);
    private:
    //params
    Eigen::Matrix<float,6,1> ddxe;
    Eigen::Matrix<float,6,1> dxe;
    Eigen::Matrix<float,6,1> xe;
    Eigen::Matrix<float,6,1> Fe;
    Eigen::Matrix<float,6,6> M;
    Eigen::Matrix<float,6,6> B;
    Eigen::Matrix<float,6,6> K;
};
Exoskeleton_dynamic::Exoskeleton_dynamic()
{
    // std::cout<<"You now create the Exoskeleton_dynamic class"<<std::endl;
    M <<    1,0,0,0,0,0,
            0,1,0,0,0,0,
            0,0,1,0,0,0,
            0,0,0,1,0,0,
            0,0,0,0,1,0,
            0,0,0,0,0,1;

    B <<    100,0,0,0,0,0,
            0,100,0,0,0,0,
            0,0,100,0,0,0,
            0,0,0,200,0,0,
            0,0,0,0,200,0,
            0,0,0,0,0,200;

    K <<    250,0,0,0,0,0,
            0,250,0,0,0,0,
            0,0,250,0,0,0,
            0,0,0,1000,0,0,
            0,0,0,0,1000,0,
            0,0,0,0,0,1000;
}
Eigen::Matrix<float,6,1> Exoskeleton_dynamic::Admittance_control(Eigen::Matrix<float,6,1> dxe_, Eigen::Matrix<float,6,1> xe_, Eigen::Matrix<float,6,1> Fe_)
{
    dxe = dxe_;
    xe  = xe_;
    Fe  = Fe_;

    Eigen::Matrix<float,6,1> tmp;
    tmp = (Fe-B*dxe-K*xe);
    //每加入判断范数：nrom(tmp)>1e-10
    ddxe = M.inverse()*(Fe-B*dxe-K*xe);
    return ddxe;
}