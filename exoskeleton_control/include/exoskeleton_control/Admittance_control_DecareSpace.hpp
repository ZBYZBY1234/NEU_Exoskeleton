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
    Eigen::Matrix<float,6,1> Expected_Velocity_DecareSpace;
    Eigen::Matrix<float,1,4> Expected_Velocity_JointSpace;
    Eigen::Matrix<float,1,4> Expected_Acceleration_DecareSpace;
    Eigen::Matrix<float,1,4> Expected_Acceleration_JointSpace;

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
        //笛卡尔空间坐标的算法系数
    Eigen::Matrix<float,6,1> m_M_DicareSpace;
    Eigen::Matrix<float,6,1> m_D_DicareSpace;
    Eigen::Matrix<float,6,1> m_K_DicareSpace;

        //关节空间坐标的算法系数
    Eigen::Matrix<float,4,1> m_M_JointSpace;
    Eigen::Matrix<float,4,1> m_D_JointSpace;
    Eigen::Matrix<float,4,1> m_K_JointSpace;

    // 数值计算时间，用于计算微分积分
    float dt;
    // flag标志。
    // true: Dicare Space;
    // false: Joint Space;
    bool flag;
public:

    Admittance_control();

    ~Admittance_control(){};

    /*
     * @Name: main
     * @Description: Thr main function of Admittance Control Algorithm
     * @Input: --float: Hip_Angle, Thigh_Angle, Calf_Angle, Ankle_Angle
     *         --Eigen::Matrix<float,6,1> Force_
     * @Output: Eigen::Matrix<float,4,1> Angle
    */
    Eigen::Matrix<float,4,1> main(
        const Eigen::Matrix<float,1,4> Feedback_Angle_,
        const Eigen::Matrix<float,1,4> Expected_Angle_,
        const Eigen::Matrix<float,1,4> Feedback_Velocity_,
        const Eigen::Matrix<float,1,4> Expected_Velocity_,
        const Eigen::Matrix<float,1,4> Feedback_Acceleration_,
        const Eigen::Matrix<float,1,4> Expected_Acceleration_,
        const Eigen::Matrix<float,6,1> Force_);

    //-----------------Control-Algorithm-Method-----------------//
    //-------------------------Function-------------------------//
    //关节空间下的算法
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

    //笛卡尔空间下的算法
    /*
     * @Name: Admittance Control Algorithm DicareSpace
     * @Description: Using Admittance Control Algorithms to Calculate the Velocity and return it
     *               which will be used Integrated and added to Feedback Angle to make each joint
     *               get the expected position.
     * @Input: --Eigen::Matrix<float,4,1> Force_Base, dxe, xe
    */
    Eigen::Matrix<float,6,1> Admittance_Control_Aloorithm_DicareSpace(
        Eigen::Matrix<float,6,1> Force_Base,
        Eigen::Matrix<float,6,1> dxe,
        Eigen::Matrix<float,6,1> xe);

    //-------------------Dicare-Space-Method--------------------//
    //-------------------------Function-------------------------//

    /* @Name: Acc_Vel
    * @Description: From Acc Integrate it to Vel
    * @Input: ddxe_, dxe_
    * @Output: dxe_targ
    */
    Eigen::Matrix<float,6,1> Acc_Vel(
        Eigen::Matrix<float,6,1> ddxe,
        Eigen::Matrix<float,6,1> dxe);

    /* @Name: Vel_Pos
    * @Description: From Vel Integrate it to Pos
    * @Input: dxe_, xe_
    * @Output: xe_targ
    */
    Eigen::Matrix<float,6,1> Vel_Pos(
        Eigen::Matrix<float,6,1> dxe,
        Eigen::Matrix<float,6,1> xe);

    /*
    * @Name: xd_xe
    * @Description: xd - xe Using Transform Matrix to Calculate it.
    * @Input: xd_, xe_targ_
    * @Output: xyzkps
    */
    Eigen::Matrix<float,6,1> xd_xe(
        Eigen::Matrix<float,1,6> xd,
        Eigen::Matrix<float,6,1> xe_targ);

    /*
    * @Name: x_xd
    * @Description: x - xd Using Transform Matrix to Calculate it.
    * @Input: Expected_KPS6_, Feedback_KPS6_
    * @Output: xe
    */
    Eigen::Matrix<float,6,1> x_xd(
        Eigen::Matrix<float,1,6> Expected_KPS6_,
        Eigen::Matrix<float,1,6> Feedback_KPS6_);

    /*
    * @Name: Feedback_Velocity_Fun
    * @Description: Calculate Feedback Velocity using function which is used derivate time
    * @Input: None
    * @Output: Feedback_Velocity
    */
    Eigen::Matrix<float,1,4> Feedback_Velocity_Fun();

};

// Admittance_control Algorithms Declareation
Admittance_control::Admittance_control(
    Eigen::Matrix<float,1,4> Expected_Angle_,
    Eigen::Matrix<float,6,1> Expected_Velocity_DicareSpace_,
    Eigen::Matrix<float,1,4> Expected_Velocity_JointSpace_,
    Eigen::Matrix<float,1,4> Expected_Acceleration_,
    bool flag_)
{
    // std::cout<<"You now create the Admittance_control class"<<std::endl;
    flag = flag_;
    Expected_Angle = Expected_Angle_;
    if(flag_)
    {
        Expected_Velocity_DecareSpace = Expected_Velocity_DicareSpace_;
    }
    else
    {
        Expected_Velocity_JointSpace = Expected_Velocity_JointSpace_;
    }
    Expected_Acceleration = Expected_Acceleration_;

    Expected_T_end = Exoskeleton_kinetic::forward_kinematics(Expected_Angle(0,0),Expected_Angle(0,1),Expected_Angle(0,2),Expected_Angle(0,3));
    //TODO:Checked.
    // std::cout<<"Expected_T_end: "<<std::endl;
    // std::cout<<Expected_T_end<<std::endl;

    Expected_RR  << Expected_T_end(0,0), Expected_T_end(0,1), Expected_T_end(0,2), 0, 0, 0,
                    Expected_T_end(1,0), Expected_T_end(1,1), Expected_T_end(1,2), 0, 0, 0,
                    Expected_T_end(2,0), Expected_T_end(2,1), Expected_T_end(2,2), 0, 0, 0,
                    0, 0, 0, Expected_T_end(0,0), Expected_T_end(0,1), Expected_T_end(0,2),
                    0, 0, 0, Expected_T_end(1,0), Expected_T_end(1,1), Expected_T_end(1,2),
                    0, 0, 0, Expected_T_end(2,0), Expected_T_end(2,1), Expected_T_end(2,2);
    Expected_KPS6  = Exoskeleton_kinetic::KPS44_KPS6(Expected_T_end);

    //TODO:Checked.
    // std::cout<<"Expected_KPS6: "<<std::endl;
    // std::cout<<Expected_KPS6<<std::endl;

    //系数设定
    m_M_DicareSpace << 1.0,1.0,1.0,1.0,1.0,1.0;
    m_D_DicareSpace << 100.0,100.0,100.0,200.0,200.0,200.0;
    m_K_DicareSpace << 250.0,250.0,250.0,1000.0,1000.0,1000.0;

    m_M_JointSpace  << 1.0,1.0,1.0,1.0;
    m_D_JointSpace  << 1.0,1.0,1.0,1.0;
    m_K_JointSpace  << 1.0,1.0,1.0,1.0;

    //TODO:Check dt.
    dt = 0.001;
}


/* @Name: main
 * @Description: The main function in for controlling by Admittance Control Algorithm
 * @Input: Hip_Angle,Thigh_Angle,Calf_Angle
 * @Output: Angle
*/
Eigen::Matrix<float,4,1> Admittance_control::main(
    const Eigen::Matrix<float,1,4> Feedback_Angle_,
    const Eigen::Matrix<float,1,4> Expected_Angle_,
    const Eigen::Matrix<float,1,4> Feedback_Velocity_,
    const Eigen::Matrix<float,1,4> Expected_Velocity_,
    const Eigen::Matrix<float,1,4> Feedback_Acceleration_,
    const Eigen::Matrix<float,1,4> Expected_Acceleration_,
    const Eigen::Matrix<float,6,1> Force_
    )
{
    /*
    if(flag)
    // {

    //     //--------------------------笛卡尔空间(Dicare Space)--------------------------//

    //     // Get the Feedback Angle of each Joint
    //     Feedback_Angle << Hip_Angle, Thigh_Angle, Calf_Angle, Ankle_Angle;
    //     Feedback_T_end = Exoskeleton_kinetic::forward_kinematics(Feedback_Angle(0,0),Feedback_Angle(0,1),Feedback_Angle(0,2),Feedback_Angle(0,3));

    //     Force_ext = Force_;
    //     std::cout<<"Force_ext: "<<std::endl;
    //     std::cout<<Force_ext<<std::endl;
    //     // Get the Transform Matrix RR.

    //     Feedback_RR <<  Feedback_T_end(0,0), Feedback_T_end(0,1), Feedback_T_end(0,2), 0, 0, 0,
    //                     Feedback_T_end(1,0), Feedback_T_end(1,1), Feedback_T_end(1,2), 0, 0, 0,
    //                     Feedback_T_end(2,0), Feedback_T_end(2,1), Feedback_T_end(2,2), 0, 0, 0,
    //                     0, 0, 0, Feedback_T_end(0,0), Feedback_T_end(0,1), Feedback_T_end(0,2),
    //                     0, 0, 0, Feedback_T_end(1,0), Feedback_T_end(1,1), Feedback_T_end(1,2),
    //                     0, 0, 0, Feedback_T_end(2,0), Feedback_T_end(2,1), Feedback_T_end(2,2);

    //     //TODO:Checked.
    //     // std::cout<<"Feedback_RR: "<<std::endl;
    //     // std::cout<<Feedback_RR<<std::endl;

    //     Force_Base = Exoskeleton_kinetic::F_Tran_base(Feedback_RR,Force_ext);
    //     //TODO:Checked.
    //     // std::cout<<"Force_Base: "<<std::endl;
    //     // std::cout<<Force_Base<<std::endl;

    //     Jn_Base = Exoskeleton_kinetic::Jaccobi(
    //         Feedback_Angle(0,0),
    //         Feedback_Angle(0,1),
    //         Feedback_Angle(0,2),
    //         Feedback_Angle(0,3),
    //         Feedback_RR,
    //         true);
    //     //TODO:Checked.
    //     // std::cout<<"Jn_Base: "<<std::endl;
    //     // std::cout<<Jn_Base<<std::endl;
    //     //完成数值微分求取速度
    //     Feedback_Velocity = Feedback_Velocity_Fun();
    //     Eigen::Matrix<float,6,1> dx; //末端加速度
    //     dx = Jn_Base*Feedback_Velocity.transpose();
    //     Eigen::Matrix<float,6,1> dxe;
    //     dxe = dx - Expected_Velocity_DecareSpace;

    //     Feedback_KPS6 = Exoskeleton_kinetic::KPS44_KPS6(Feedback_T_end);
    //     //TODO:Checked.
    //     std::cout<<"Feedback_KPS6: "<<std::endl;
    //     std::cout<<Feedback_KPS6<<std::endl;

    //     //x-xd input: Feedback_KPS6, Expected_KPS6
    //     Eigen::Matrix<float,6,1> xe;
    //     xe = x_xd(Expected_KPS6, Feedback_KPS6);
    //     //TODO:Checked.
    //     // std::cout<<"Expected_KPS6: "<<std::endl;
    //     // std::cout<<Expected_KPS6<<std::endl;
    //     // std::cout<<"Feedback_KPS6: "<<std::endl;
    //     // std::cout<<Feedback_KPS6<<std::endl;
    //     // std::cout<<"xe: "<<std::endl;
    //     // std::cout<<xe<<std::endl;

    //     /* @Name: Admittance Control Algorithm
    //     * @Description: This is the Realization of this Algorithms which is most important
    //     * @Input: Force_Base, xe, dxe
    //     * @Output: ddxe
    //     */
    //     Eigen::Matrix<float,6,1> ddxe;
    //     //Using Function to get the ddxe(Acceleration) of the Admittance Control Algorithm.
    //     ddxe = Admittance_Control_Aloorithm_DicareSpace(Force_Base,dxe,xe);
    //     //TODO:Checked
    //     // std::cout<<"ddxe: "<<std::endl;
    //     // std::cout<<ddxe<<std::endl;

    //     /* @Name: Acc_Vel
    //     * @Description: This is using the ddxe and dxe to compute the dxe_target
    //     * @Input: ddxe, dxe
    //     * @Output: dxe_targ
    //     */
    //     Eigen::Matrix<float,6,1> dxe_targ;
    //     dxe_targ = Acc_Vel(ddxe,dxe);
    //     //TODO:Checked
    //     // std::cout<<"dxe_targ: "<<std::endl;
    //     // std::cout<<dxe_targ<<std::endl;

    //     /* @Name: Vel_Pos
    //     * @Description: This is using the dxe and xe to comput the xe_target
    //     * @Input: dxe, xe
    //     * @Output: xe_targ
    //     */
    //     Eigen::Matrix<float,6,1> xe_targ;
    //     xe_targ = Vel_Pos(dxe, xe);
    //     //TODO:Checked.
    //     // std::cout<<"xe_targ: "<<std::endl;
    //     // std::cout<<xe_targ<<std::endl;

    //     /* @Name: xd_xe
    //     * @Description: This is using the xd and xe_targ to compute the xyzkps
    //     * @Input: xd, xe_targ
    //     * @Output: xyzkps
    //     */
    //     Eigen::Matrix<float,6,1> xyzkps;
    //     xyzkps = xd_xe(Expected_KPS6, xe_targ);
    //     //TODO:Checked.
    //     // std::cout<<"xyzkps: "<<std::endl;
    //     // std::cout<<xyzkps<<std::endl;

    //     /* @Name: KPs6_KPS44
    //     * @Description: Using kinetic Class function
    //     * @Input: xyzkps(KPS6)
    //     * @Output: KPS44
    //     */
    //     Eigen::Matrix<float,4,4> KPS44;
    //     KPS44 = Exoskeleton_kinetic::KPS6_KPS44(xyzkps);
    //     //TODO:Checked.
    //     // std::cout<<"KPS44: "<<std::endl;
    //     // std::cout<<KPS44<<std::endl;

    //     /* @Name: inverse
    //     * @Description: Using kinetic Class function
    //     * @Input: KPS44
    //     * @Output: Angle
    //     */
    //     Eigen::Matrix<float,4,1> Angle;
    //     Angle = Exoskeleton_kinetic::inverse_kinematics(KPS44);
    //     //TODO:Checked.
    //     // std::cout<<"Angle: "<<std::endl;
    //     // std::cout<<Angle<<std::endl;

    //     Past_Feedback_Angle(0,0) = Feedback_Angle(0,0);
    //     Past_Feedback_Angle(0,1) = Feedback_Angle(0,1);
    //     Past_Feedback_Angle(0,2) = Feedback_Angle(0,2);
    //     Past_Feedback_Angle(0,3) = Feedback_Angle(0,3);

    //     return Angle;
    // }
    else{
        //--------------------------关节空间(Joint Space)--------------------------//
        // Get the Feedback Angle of each Joint
        Feedback_Angle << Hip_Angle, Thigh_Angle, Calf_Angle, Ankle_Angle;
        std::cout<<"Feedback_Angle: "<<std::endl;
        std::cout<<Feedback_Angle<<std::endl;

        Force_ext = Force_;
        std::cout<<"Force_ext: "<<std::endl;
        std::cout<<Force_ext<<std::endl;

        Feedback_T_end = Exoskeleton_kinetic::forward_kinematics(Feedback_Angle(0,0),Feedback_Angle(0,1),Feedback_Angle(0,2),Feedback_Angle(0,3));

        // Get the Transform Matrix RR.
        //TODO:不确定是否需要对 Force_ext 变换至基坐标系下 进行描述。
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
        std::cout<<"Jn_Tool: "<<std::endl;
        std::cout<<Jn_Tool.transpose()<<std::endl;
        std::cout<<"T_ext: "<<std::endl;
        std::cout<<T_ext<<std::endl;

        Feedback_Velocity = Feedback_Velocity_Fun();

        Eigen::Matrix<float,4,1> Angle;

        Angle = Admittance_Control_Algorithm_JointSpace (
            T_ext,
            Feedback_Angle,
            Expected_Angle,
            Feedback_Velocity,
            Expected_Acceleration,
            Expected_Velocity_JointSpace);
        Angle = Angle + Feedback_Angle.transpose();

        Past_Feedback_Angle(0,0) = Feedback_Angle(0,0);
        Past_Feedback_Angle(0,1) = Feedback_Angle(0,1);
        Past_Feedback_Angle(0,2) = Feedback_Angle(0,2);
        Past_Feedback_Angle(0,3) = Feedback_Angle(0,3);

        std::cout<<"Angle: "<<std::endl;
        std::cout<<Angle<<std::endl;
        return Angle;
    }
}

//-----------------Control-Algorithm-Method-----------------//
//-------------------------Function-------------------------//

/* @Name: Admittance_Control_Algorithm_DicareSpace
 * @Description: Using the Admittance Control Algorithms in Dicare Space.
 * @Input: Force_Base_, dxe_, xe_
 * @Output: ddxe_
*/

Eigen::Matrix<float,6,1> Admittance_control::Admittance_Control_Aloorithm_DicareSpace(
    Eigen::Matrix<float,6,1> Force_Base_,
    Eigen::Matrix<float,6,1> dxe_,
    Eigen::Matrix<float,6,1> xe_
    )
{
    Eigen::Matrix<float,6,6> M,D,K;
    M  <<   m_M_DicareSpace(0,0),0.0,0.0,0.0,0.0,0.0,
            0.0,m_M_DicareSpace(1,0),0.0,0.0,0.0,0.0,
            0.0,0.0,m_M_DicareSpace(2,0),0.0,0.0,0.0,
            0.0,0.0,0.0,m_M_DicareSpace(3,0),0.0,0.0,
            0.0,0.0,0.0,0.0,m_M_DicareSpace(4,0),0.0,
            0.0,0.0,0.0,0.0,0.0,m_M_DicareSpace(5,0);

    D  <<   m_D_DicareSpace(0,0),0.0,0.0,0.0,0.0,0.0,
            0.0,m_D_DicareSpace(1,0),0.0,0.0,0.0,0.0,
            0.0,0.0,m_D_DicareSpace(2,0),0.0,0.0,0.0,
            0.0,0.0,0.0,m_D_DicareSpace(3,0),0.0,0.0,
            0.0,0.0,0.0,0.0,m_D_DicareSpace(4,0),0.0,
            0.0,0.0,0.0,0.0,0.0,m_D_DicareSpace(5,0);

    K  <<   m_K_DicareSpace(0,0),0.0,0.0,0.0,0.0,0.0,
            0.0,m_K_DicareSpace(1,0),0.0,0.0,0.0,0.0,
            0.0,0.0,m_K_DicareSpace(2,0),0.0,0.0,0.0,
            0.0,0.0,0.0,m_K_DicareSpace(3,0),0.0,0.0,
            0.0,0.0,0.0,0.0,m_K_DicareSpace(4,0),0.0,
            0.0,0.0,0.0,0.0,0.0,m_K_DicareSpace(5,0);

    Eigen::Matrix<float,6,1> tmp;
    tmp = (Force_Base_ - D*dxe_ - K*xe_);

    Eigen::Matrix<float,6,1> ddxe;

    if (tmp.norm()>1e-10)
    {
        ddxe = M.inverse() * (Force_Base_ - D*dxe_ - K*xe_);
    }
    else
    {
        ddxe << 0.0,0.0,0.0,0.0,0.0,0.0;
    }

    return ddxe;
}

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
    Eigen::Matrix<float,4,4> M,D,K;

    M  <<   m_M_JointSpace(0,0),0.0,0.0,0.0,
            0.0,m_M_JointSpace(1,0),0.0,0.0,
            0.0,0.0,m_M_JointSpace(2,0),0.0,
            0.0,0.0,0.0,m_M_JointSpace(3,0);

    D  <<   m_D_JointSpace(0,0),0.0,0.0,0.0,
            0.0,m_D_JointSpace(1,0),0.0,0.0,
            0.0,0.0,m_D_JointSpace(2,0),0.0,
            0.0,0.0,0.0,m_D_JointSpace(3,0);

    K  <<   m_K_DicareSpace(0,0),0.0,0.0,0.0,
            0.0,m_K_DicareSpace(1,0),0.0,0.0,
            0.0,0.0,m_K_DicareSpace(2,0),0.0,
            0.0,0.0,0.0,m_K_DicareSpace(3,0);

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


//-------------------Dicare-Space-Method--------------------//
//-------------------------Function-------------------------//
/* @Name: Acc_Vel
 * @Description: From Acc Integrate it to Vel
 * @Input: ddxe_, dxe_
 * @Output: dxe_targ
*/

Eigen::Matrix<float,6,1> Admittance_control::Acc_Vel(Eigen::Matrix<float,6,1> ddxe_,Eigen::Matrix<float,6,1> dxe_)
{

    Eigen::Matrix<float,6,1> dxe_targ;
    Eigen::Matrix<float,6,1> dt_ddxe;

    dt_ddxe = ddxe_*dt;

    dxe_targ(0,0) = dxe_(0,0) + dt_ddxe(0,0);
    dxe_targ(1,0) = dxe_(1,0) + dt_ddxe(1,0);
    dxe_targ(2,0) = dxe_(2,0) + dt_ddxe(2,0);
    dxe_targ(3,0) = dxe_(3,0) + dt_ddxe(3,0);
    dxe_targ(4,0) = dxe_(4,0) + dt_ddxe(4,0);
    dxe_targ(5,0) = dxe_(5,0) + dt_ddxe(5,0);

    return dxe_targ;
}


/* @Name: Vel_Pos
 * @Description: From Vel Integrate it to Pos
 * @Input: dxe_, xe_
 * @Output: xe_targ
*/
Eigen::Matrix<float,6,1> Admittance_control::Vel_Pos(Eigen::Matrix<float,6,1> dxe_,Eigen::Matrix<float,6,1> xe_)
{
    Eigen::Matrix<float,6,1> xe_targ;
    Eigen::Matrix<float,6,1> dt_dxe;

    dt_dxe = dxe_*dt;
    xe_targ(0,0) = xe_(0,0) + dt_dxe(0,0);
    xe_targ(1,0) = xe_(1,0) + dt_dxe(1,0);
    xe_targ(2,0) = xe_(2,0) + dt_dxe(2,0);

    // 等效转轴转姿态矩阵 -- 1
    Eigen::Matrix<float,3,1> eqaxis;
    eqaxis(0,0) = dt_dxe(3,0);
    eqaxis(1,0) = dt_dxe(4,0);
    eqaxis(2,0) = dt_dxe(5,0);

    float A = eqaxis.norm();

    Eigen::Matrix<float,3,3> dR;

    if (A>1e-10)
    {
        Eigen::Matrix<float,3,1> axis;
        axis = eqaxis/A;

        float sx,cx,v;
        sx = sin(A);
        cx = cos(A);
        v  = 1-cx;
        dR  <<  axis(0,0)*axis(0,0)*v + cx, axis(0,0)*axis(1,0)*v - axis(2,0)*sx, axis(0,0)*axis(2,0)*v + axis(1,0)*sx,
                axis(0,0)*axis(1,0)*v + axis(2,0)*sx, axis(1,0)*axis(1,0)*v + cx, axis(1,0)*axis(2,0)*v - axis(0,0)*sx,
                axis(0,0)*axis(2,0)*v - axis(1,0)*sx, axis(1,0)*axis(2,0)*v + axis(0,0)*sx, axis(2,0)*axis(2,0)*v + cx;
    }
    else
    {
        dR << 1,0,0,0,1,0,0,0,1;
    }

    // 等效转轴转姿态矩阵 -- 2
    eqaxis(0,0) = xe_(3,0);
    eqaxis(1,0) = xe_(4,0);
    eqaxis(2,0) = xe_(5,0);

    A = eqaxis.norm();

    Eigen::Matrix<float,3,3> Re;

    if (A>1e-10)
    {
        Eigen::Matrix<float,3,1> axis;
        axis = eqaxis/A;

        float sx,cx,v;
        sx = sin(A);
        cx = cos(A);
        v  = 1-cx;
        Re  <<  axis(0,0)*axis(0,0)*v + cx, axis(0,0)*axis(1,0)*v - axis(2,0)*sx, axis(0,0)*axis(2,0)*v + axis(1,0)*sx,
                axis(0,0)*axis(1,0)*v + axis(2,0)*sx, axis(1,0)*axis(1,0)*v + cx, axis(1,0)*axis(2,0)*v - axis(0,0)*sx,
                axis(0,0)*axis(2,0)*v - axis(1,0)*sx, axis(1,0)*axis(2,0)*v + axis(0,0)*sx, axis(2,0)*axis(2,0)*v + cx;
    }
    else
    {
        Re << 1,0,0,0,1,0,0,0,1;
    }

    Eigen::Matrix<float,3,3> R;
    R = dR * Re;

    xe_targ(4,0) = atan2(-R(2,0),sqrt(R(0,0)*R(0,0)+R(1,0)*R(1,0)));

    if (fabs(fabs(xe_targ(4,0))-PI/ 2.0) < 1e-7)
    {
        if (xe_targ(4,0) > 0) // pi/2
        {
            xe_targ(4,0) = PI / 2.0;
            xe_targ(5,0) = 0.0;
            xe_targ(3,0) = atan2(R(0,1),R(1,1));
        }
        else // -pi/2
        {
            xe_targ(4,0) = -PI / 2.0;
            xe_targ(5,0) = 0.0;
            xe_targ(3,0) = -atan2(R(0,1),R(1,1));
        }
    }
    else
    {
        float cp;
        cp = cos(xe_targ(4,0));
        xe_targ(5,0) = atan2(R(1,0)/cp, R(0,0)/cp);
        xe_targ(3,0) = atan2(R(2,1)/cp, R(2,2)/cp);
    }

    return xe_targ;
}


/*
 * @Name: xd_xe
 * @Description: xd - xe Using Transform Matrix to Calculate it.
 * @Input: xd_, xe_targ_
 * @Output: xyzkps
*/
Eigen::Matrix<float,6,1> Admittance_control::xd_xe(
    Eigen::Matrix<float,1,6> xd_,
    Eigen::Matrix<float,6,1> xe_targ_
    )
{
    Eigen::Matrix<float,6,1> xyzkps;
    xyzkps(0,0) = xd_(0,0) + xe_targ_(0,0);
    xyzkps(1,0) = xd_(0,1) + xe_targ_(1,0);
    xyzkps(2,0) = xd_(0,2) + xe_targ_(2,0);

    Eigen::Matrix<float,3,1> rpy;
    float sx,cx,sy,cy,sz,cz;
    Eigen::Matrix<float,3,3> dR,R0;

    rpy(0,0) = xe_targ_(3,0);
    rpy(1,0) = xe_targ_(4,0);
    rpy(2,0) = xe_targ_(5,0);

    sx = sin(rpy(0,0));
    cx = cos(rpy(0,0));
    sy = sin(rpy(1,0));
    cy = cos(rpy(1,0));
    sz = sin(rpy(2,0));
    cz = cos(rpy(2,0));

    dR  <<  cy*cz, cz*sx*sy - cx*sz, sx*sz + cx*cz*sy,
            cy*sz, cx*cz + sx*sy*sz, cx*sy*sz - cz*sx,
            -sy, cy*sx, cx*cy;

    rpy(0,0) = xd_(0,3);
    rpy(1,0) = xd_(0,4);
    rpy(2,0) = xd_(0,5);

    R0  <<  cy*cz, cz*sx*sy - cx*sz, sx*sz + cx*cz*sy,
            cy*sz, cx*cz + sx*sy*sz, cx*sy*sz - cz*sx,
            -sy, cy*sx, cx*cy;
    Eigen::Matrix<float,3,3> R;
    R = dR * R0;

    xyzkps(4,0) = atan2(-R(2,0),sqrt(R(0,0)*R(0,0)+R(1,0)*R(1,0)));
    if (fabs(fabs(xyzkps(4,0))-PI/ 2.0) < 1e-7)
    {
        if (xyzkps(4,0) > 0) // pi/2
        {
            xyzkps(4,0) = PI / 2.0;
            xyzkps(5,0) = 0.0;
            xyzkps(3,0) = atan2(R(0,1),R(1,1));
        }
        else // -pi/2
        {
            xyzkps(4,0) = -PI / 2.0;
            xyzkps(5,0) = 0.0;
            xyzkps(3,0) = -atan2(R(0,1),R(1,1));
        }
    }
    else
    {
        float cp;
        cp = cos(xyzkps(4,0));
        xyzkps(5,0) = atan2(R(1,0)/cp, R(0,0)/cp);
        xyzkps(3,0) = atan2(R(2,1)/cp, R(2,2)/cp);
    }

    return xyzkps;
}


/*
 * @Name: x_xd
 * @Description: x - xd Using Transform Matrix to Calculate it.
 * @Input: Expected_KPS6_, Feedback_KPS6_
 * @Output: xe
*/
Eigen::Matrix<float,6,1> Admittance_control::x_xd(
    Eigen::Matrix<float,1,6> Expected_KPS6_,
    Eigen::Matrix<float,1,6> Feedback_KPS6_
    )
{
    Eigen::Matrix<float,6,1> xe;

    xe(0,0) = Feedback_KPS6_(0,0) - Expected_KPS6_(0,0);
    xe(1,0) = Feedback_KPS6_(0,1) - Expected_KPS6_(0,1);
    xe(2,0) = Feedback_KPS6_(0,2) - Expected_KPS6_(0,2);

    float p_f,q_f,r_f;
    p_f = Feedback_KPS6_(0,3);
    q_f = Feedback_KPS6_(0,4);
    r_f = Feedback_KPS6_(0,5);
    Eigen::Matrix<float,3,3> I_f;
    I_f << 1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0;
    float sx_f,cx_f,sy_f,cy_f,sz_f,cz_f;
    sx_f = sin(p_f);
    cx_f = cos(p_f);
    sy_f = sin(q_f);
    cy_f = cos(q_f);
    sz_f = sin(r_f);
    cz_f = cos(r_f);

    I_f(0,0) = cy_f * cz_f;
    I_f(0,1) = cz_f * sx_f * sy_f - cx_f * sz_f;
    I_f(0,2) = sx_f * sz_f + cx_f * cz_f * sy_f;
    I_f(1,0) = cy_f * sz_f;
    I_f(1,1) = cx_f * cz_f + sx_f * sy_f * sz_f;
    I_f(1,2) = cx_f * sy_f * sz_f - cz_f * sx_f;
    I_f(2,0) = -sy_f;
    I_f(2,1) = cy_f * sx_f;
    I_f(2,2) = cx_f * cy_f;

    float p_e,q_e,r_e;
    p_e = Expected_KPS6_(0,3);
    q_e = Expected_KPS6_(0,4);
    r_e = Expected_KPS6_(0,5);
    Eigen::Matrix<float,3,3> I_e;
    I_e << 1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0;
    float sx_e,cx_e,sy_e,cy_e,sz_e,cz_e;
    sx_e = sin(p_e);
    cx_e = cos(p_e);
    sy_e = sin(q_e);
    cy_e = cos(q_e);
    sz_e = sin(r_e);
    cz_e = cos(r_e);

    I_e(0,0) = cy_e * cz_e;
    I_e(0,1) = cz_e * sx_e * sy_e - cx_e * sz_e;
    I_e(0,2) = sx_e * sz_e + cx_e * cz_e * sy_e;
    I_e(1,0) = cy_e * sz_e;
    I_e(1,1) = cx_e * cz_e + sx_e * sy_e * sz_e;
    I_e(1,2) = cx_e * sy_e * sz_e - cz_e * sx_e;
    I_e(2,0) = -sy_e;
    I_e(2,1) = cy_e * sx_e;
    I_e(2,2) = cx_e * cy_e;

    Eigen::Matrix<float,3,3> DI;
    //矩阵的转置
    DI = I_f * I_e.transpose();
    float cp;
    cp = (DI(0,0)+DI(1,1)+DI(2,2)-1)/2.0;
    if (cp>1-(1e-10))
    {
        xe(3,0) = 0;
        xe(4,0) = 0;
        xe(5,0) = 0;
    }
    else
    {
        if (cp<-1+(1e-10))
        {
            xe(3,0) = sqrt((DI(0,0)+1.0)/2);
            xe(4,0) = sqrt((DI(1,1)+1.0)/2);
            xe(5,0) = sqrt((DI(2,2)+1.0)/2);
            if(DI(0,2) < 0){xe(0,3) = -xe(0,3);}
            if(DI(2,1) < 0){xe(0,4) = -xe(0,4);}
            if((xe(0,3)*xe(0,4)*DI(0,1))<0){xe(0,3) = -xe(0,3);}
            xe(3,0) *= PI;
            xe(4,0) *= PI;
            xe(5,0) *= PI;
        }
        else
        {
            float Q;
            Q = acos(cp);
            xe(3,0) = Q*0.5*(DI(2,1)-DI(1,2))/sin(Q);
            xe(4,0) = Q*0.5*(DI(0,2)-DI(2,0))/sin(Q);
            xe(5,0) = Q*0.5*(DI(1,0)-DI(0,1))/sin(Q);
        }
    }
    return xe;
}


/*
 * @Name: Feedback_Velocity_Fun
 * @Description: Calculate Feedback Velocity using function which is used derivate time
 * @Input: None
 * @Output: Feedback_Velocity
*/

Eigen::Matrix<float,1,4> Admittance_control::Feedback_Velocity_Fun()
{
    Eigen::Matrix<float,1,4> Feedback_V;

    if (Past_Feedback_Angle.isIdentity())
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
    }
    return Feedback_V;
}