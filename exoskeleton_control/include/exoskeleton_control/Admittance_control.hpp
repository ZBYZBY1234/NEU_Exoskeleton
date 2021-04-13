/* * @Author: Beal.MS
   * @Date: 2021-03-20 12:01:47
 * @Last Modified by: Beal.MS
 * @Last Modified time: 2021-03-20 23:32:51
   * @Description: Admittance Control
   * @input: Expected_Angle, Expected_Velocity_DecareSpace, Force
   * @output: Position_Angle
*/
#include "exoskeleton_kinematics/exoskeleton_kinematics.hpp"
#include "exoskeleton_dynamics/exoskeleton_dynamics.hpp"
#include <cmath>

class Admittance_control: public Exoskeleton_kinetic, public Exoskeleton_dynamic
{
private:

    // Force Data
    Eigen::Matrix<float,6,1> Force_ext;
    Eigen::Matrix<float,6,1> Force_Base;
    Eigen::Matrix<float,4,1> T_ext;

    // Expected Data
    Eigen::Matrix<float,1,4> Expected_Angle;
    Eigen::Matrix<float,1,4> Expected_Velocity;
    Eigen::Matrix<float,1,4> Expected_Acceleration;

    Eigen::Matrix<float,4,4> Expected_T_end;
    Eigen::Matrix<float,1,6> Expected_KPS6;

    Eigen::Matrix<float,6,6> Expected_RR;

    // Feedback Data
    Eigen::Matrix<float,1,4> Feedback_Angle;
    Eigen::Matrix<float,1,4> Past_Feedback_Angle;
    Eigen::Matrix<float,1,4> Feedback_Velocity;
    Eigen::Matrix<float,1,4> Feedback_Acceleration;

    Eigen::Matrix<float,4,4> Feedback_T_end;
    Eigen::Matrix<float,1,6> Feedback_KPS6;

    Eigen::Matrix<float,6,6> Feedback_RR;

    // Jaccobi
    Eigen::Matrix<float,6,4> Jn_Base;
    Eigen::Matrix<float,6,4> Jn_Tool;

    //阻抗/导纳控制系数
    Eigen::Matrix<float,4,1> m_M_JointSpace;
    Eigen::Matrix<float,4,1> m_D_JointSpace;
    Eigen::Matrix<float,4,1> m_K_JointSpace;

    // 数值计算时间，用于计算微分积分
    float dt;
    // flag标志。
    // true: Dicare Space;
    // false: Joint Space;
    bool flag;
    bool isIdentity;
    // 导纳系数矩阵
    Eigen::Matrix<float,4,4> M,D,K;
public:

    Admittance_control();

    ~Admittance_control(){};

    /*
     * @Name: main
     * @Description: Thr main function of Admittance Control Algorithm
     * @Input: --Eigen::Matrix<float,1,4> Feedback_Angle_, Expected_Angle_, Feedback_Velocity_, Expected_Velocity_, Feedback_Acceleration_,Expected_Acceleration_
     *         --Eigen::Matrix<float,6,1> Force_
     * @Output: Eigen::Matrix<float,4,1> Angle
    */
    Eigen::Matrix<float,4,1> main(
        const Eigen::Matrix<float,1,4> Feedback_Angle_,
        const Eigen::Matrix<float,1,4> Expected_Angle_,
        const Eigen::Matrix<float,1,4> Expected_Velocity_,
        const Eigen::Matrix<float,1,4> Expected_Acceleration_,
        const Eigen::Matrix<float,6,1> Force_);

    //-----------------Control-Algorithm-Method-----------------//
    //-------------------------Function-------------------------//
    /*
     * @Name: Admittance Control Algorithm JointSpace
     * @Description: Using Admittance Control Algorithms to Calculate the Velocity and return it
     *               which will be used Integrated and added to Feedback Angle to make each joint
     *               get the expected position.
     * @Input: --Eigen::Matrix<float,4,1> T_ext_,Feedback_Angle_,Expected_Angle_,Feedback_Angle_,
     *                                    Expected_Acceleration_,Expected_Velocity_JointSpace
    */
    Eigen::Matrix<float,4,1> Admittance_Control_Algorithm_JointSpace (
            Eigen::Matrix<float,4,1> T_ext_,
            Eigen::Matrix<float,1,4> Feedback_Angle_,
            Eigen::Matrix<float,1,4> Expected_Angle_,
            Eigen::Matrix<float,1,4> Feedback_Velocity_,
            Eigen::Matrix<float,1,4> Expected_Acceleration_,
            Eigen::Matrix<float,1,4> Expected_Velocity_JointSpace
        );

    Eigen::Matrix<float,1,4> Feedback_Velocity_Fun();

};

// Admittance_control Algorithms Declareation
Admittance_control::Admittance_control()
{
    //系数设定

    m_M_JointSpace  << 100.0,10.0,10.0,100.0;
    m_D_JointSpace  << 100.0,100.0,100.0,100.0;
    m_K_JointSpace  << 100.0,1000.0,1000.0,100.0;

    //TODO:Check dt.
    dt = 0.01;

    isIdentity = false;
}


/* @Name: main
 * @Description: The main function in for controlling by Admittance Control Algorithm
 * @Input: Hip_Angle,Thigh_Angle,Calf_Angle
 * @Output: Angle
*/
Eigen::Matrix<float,4,1> Admittance_control::main(
    const Eigen::Matrix<float,1,4> Feedback_Angle_,
    const Eigen::Matrix<float,1,4> Expected_Angle_,
    const Eigen::Matrix<float,1,4> Expected_Velocity_,
    const Eigen::Matrix<float,1,4> Expected_Acceleration_,
    const Eigen::Matrix<float,6,1> Force_
    )
{
        //--------------------------关节空间(Joint Space)--------------------------//
        // Get the Feedback Angle of each Joint
        Feedback_Angle = Feedback_Angle_;
        Expected_Angle = Expected_Angle_;
        Expected_Velocity = Expected_Velocity_;
        Expected_Acceleration = Expected_Acceleration_;
        Force_ext = Force_;

        Expected_T_end = Exoskeleton_kinetic::forward_kinematics(Expected_Angle(0,0),Expected_Angle(0,1),Expected_Angle(0,2),Expected_Angle(0,3));
        Feedback_T_end = Exoskeleton_kinetic::forward_kinematics(Feedback_Angle(0,0),Feedback_Angle(0,1),Feedback_Angle(0,2),Feedback_Angle(0,3));

        // Get the Transform Matrix RR.
        //TODO:不确定是否需要对 Force_ext 变换至基坐标系下 进行描述。
        Expected_RR  << Expected_T_end(0,0), Expected_T_end(0,1), Expected_T_end(0,2), 0, 0, 0,
                        Expected_T_end(1,0), Expected_T_end(1,1), Expected_T_end(1,2), 0, 0, 0,
                        Expected_T_end(2,0), Expected_T_end(2,1), Expected_T_end(2,2), 0, 0, 0,
                        0, 0, 0, Expected_T_end(0,0), Expected_T_end(0,1), Expected_T_end(0,2),
                        0, 0, 0, Expected_T_end(1,0), Expected_T_end(1,1), Expected_T_end(1,2),
                        0, 0, 0, Expected_T_end(2,0), Expected_T_end(2,1), Expected_T_end(2,2);
        Feedback_RR <<  Feedback_T_end(0,0), Feedback_T_end(0,1), Feedback_T_end(0,2), 0, 0, 0,
                        Feedback_T_end(1,0), Feedback_T_end(1,1), Feedback_T_end(1,2), 0, 0, 0,
                        Feedback_T_end(2,0), Feedback_T_end(2,1), Feedback_T_end(2,2), 0, 0, 0,
                        0, 0, 0, Feedback_T_end(0,0), Feedback_T_end(0,1), Feedback_T_end(0,2),
                        0, 0, 0, Feedback_T_end(1,0), Feedback_T_end(1,1), Feedback_T_end(1,2),
                        0, 0, 0, Feedback_T_end(2,0), Feedback_T_end(2,1), Feedback_T_end(2,2);

        //TODO:
        //工具坐标系下描述
        Jn_Tool = Exoskeleton_kinetic::Jaccobi(
            Feedback_Angle(0,0),
            Feedback_Angle(0,1),
            Feedback_Angle(0,2),
            Feedback_Angle(0,3),
            Feedback_RR,
            false);

        //TODO:转换末端力至关节力
        T_ext = Jn_Tool.transpose()*Force_ext;

        Feedback_Velocity = Feedback_Velocity_Fun();

        Eigen::Matrix<float,4,1> Angle;

        Angle = Admittance_Control_Algorithm_JointSpace (
            T_ext,
            Feedback_Angle,
            Expected_Angle,
            Feedback_Velocity,
            Expected_Acceleration,
            Expected_Velocity);
        std::cout<<"dAngle: "<<Angle<<std::endl;
        std::cout<<"Feedback_Angle: "<<Feedback_Angle<<std::endl;
        Angle = Angle + Feedback_Angle.transpose();

        Past_Feedback_Angle(0,0) = Feedback_Angle(0,0);
        Past_Feedback_Angle(0,1) = Feedback_Angle(0,1);
        Past_Feedback_Angle(0,2) = Feedback_Angle(0,2);
        Past_Feedback_Angle(0,3) = Feedback_Angle(0,3);

        std::cout<<"Angle: "<<std::endl;
        std::cout<<Angle<<std::endl;
        return Angle;

}

//-----------------Control-Algorithm-Method-----------------//
//-------------------------Function-------------------------//

/* @Name: Admittance_Control_Algorithm_JointSpace
 * @Description: Using the Admittance Control Algorithms in Joint Space.
 * @Input: T_ext, Feedback_Angle, Expected_Angle, Feedback_Velocity, Expected_Acceleration, Expected_Velocity_JointSpace
 * @Output: Angle
*/

Eigen::Matrix<float,4,1> Admittance_control::Admittance_Control_Algorithm_JointSpace (
    Eigen::Matrix<float,4,1> T_ext_,
    Eigen::Matrix<float,1,4> Feedback_Angle_,
    Eigen::Matrix<float,1,4> Expected_Angle_,
    Eigen::Matrix<float,1,4> Feedback_Velocity_,
    Eigen::Matrix<float,1,4> Expected_Acceleration_,
    Eigen::Matrix<float,1,4> Expected_Velocity_JointSpace_
    )

{

    M  <<   m_M_JointSpace(0,0),0.0,0.0,0.0,
            0.0,m_M_JointSpace(1,0),0.0,0.0,
            0.0,0.0,m_M_JointSpace(2,0),0.0,
            0.0,0.0,0.0,m_M_JointSpace(3,0);

    D  <<   m_D_JointSpace(0,0),0.0,0.0,0.0,
            0.0,m_D_JointSpace(1,0),0.0,0.0,
            0.0,0.0,m_D_JointSpace(2,0),0.0,
            0.0,0.0,0.0,m_D_JointSpace(3,0);

    K  <<   m_K_JointSpace(0,0),0.0,0.0,0.0,
            0.0,m_K_JointSpace(1,0),0.0,0.0,
            0.0,0.0,m_K_JointSpace(2,0),0.0,
            0.0,0.0,0.0,m_K_JointSpace(3,0);

    Eigen::Matrix<float,4,1> tmp;

    /*  @Description: 为了求出导纳控制算法的输出
        @Input: T_ext, Expected_Angle, Expected_Velocity, Expected_Acceleration, Feedback_Angle,Feedback_Velocity
    */
    tmp = T_ext_ -
    K*(Feedback_Angle_-Expected_Angle_).transpose() +
    M*(Feedback_Velocity_/dt + Expected_Acceleration_).transpose() +
    D*Expected_Velocity_JointSpace_.transpose();

    Eigen::Matrix<float,4,1> dx;

    if (tmp.norm()>1e-10)
    {
        dx = (M/dt+D).inverse()*tmp;
    }
    else
    {
        dx << 0.0,0.0,0.0,0.0;
    }

    dx = dx*dt;
    return dx;
}

Eigen::Matrix<float,1,4> Admittance_control::Feedback_Velocity_Fun()
{
    Eigen::Matrix<float,1,4> Feedback_V;

    if (Past_Feedback_Angle.isIdentity() || isIdentity)
    {
        Feedback_V(0,0) = (Feedback_Angle(0,0) - Past_Feedback_Angle(0,0))/dt;
        Feedback_V(0,1) = (Feedback_Angle(0,1) - Past_Feedback_Angle(0,1))/dt;
        Feedback_V(0,2) = (Feedback_Angle(0,2) - Past_Feedback_Angle(0,2))/dt;
        Feedback_V(0,3) = (Feedback_Angle(0,3) - Past_Feedback_Angle(0,3))/dt;
    }
    else
    {
        //No value in Past_Feedback_Angle
        std::cout<<"There is no value in Past_Feedback_Angle"<<std::endl;
        Feedback_V << 0.0,0.0,0.0,0.0;
        isIdentity = true;
    }
    return Feedback_V;
}