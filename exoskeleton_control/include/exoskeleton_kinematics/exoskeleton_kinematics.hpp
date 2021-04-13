#include <iostream>
#include <eigen3/Eigen/Dense>
#include <math.h>

#define PI 3.1415926
class Exoskeleton_kinetic
{
    public:
        Exoskeleton_kinetic()
        {
            // std::cout<<"You now create the Exoskeleton_kinetic class"<<std::endl;
            // TODO: Checked.
            d1 = -0.097;
            a3 = -sqrt(pow(0.23966,2)+pow(0.28189,2));
            a4 = -sqrt(pow(-0.15989,2)+pow(0.31135,2));
            T_base <<   1,0,0,0,
                        0,1,0,0,
                        0,0,1,0,
                        0,0,0,1;
            T_tool <<   1,0,0,0,
                        0,1,0,0,
                        0,0,1,0,
                        0,0,0,1;

            T_tool_Feed <<  1,0,0,0,
                            0,1,0,0,
                            0,0,1,0,
                            0,0,0,1;
        }
        ~Exoskeleton_kinetic()
        {
            // std::cout<<"Clear Exoskeleton_kinetic class"<<std::endl;
        }
    public:

        /*
         * @Name: forward_kinematics
         * @Description: The forward kinematics for robot each angle which to get the end Altitude Matrix.
         * @Input: --float Hip_angle, Thigh_angle, Calf_angle, Ankle_angle
         * @Output: --Eigen::Matrix<float,4,4> T_end
        */

        Eigen::Matrix<float,4,4> forward_kinematics(float Hip_angle, float Thigh_angle, float Calf_angle, float Ankle_angle);

        /*
         * @Name: inverse_kinematics
         * @Description: The inverse kinematics for robot and used to calculate each joint state value by T_end.
         * @Input: --Eigen::Matrix<float,4,4> T_end
         * @Output: --Eigen::Matrix<float,4,1> Angle
        */

        Eigen::Matrix<float,4,1> inverse_kinematics(Eigen::Matrix<float,4,4> T_end);

        /*
         * @Name: Jaccobi
         * @Description: The function for Calculating the Jaccobi Matrix based on each joint state value and base coorinate.
         * @Input: --float: Hip_angle, Thigh_angle, Calf_angle, Ankle_angle
         *         --Eigen::Matrix<float,6,6>: RR_
         *         --bool: flag_
         * @Output: Eigen::Matrix<floag,6,4>: Jaccobi
        */

        Eigen::Matrix<float,6,4> Jaccobi(float Hip_angle, float Thigh_angle, float Calf_angle, float Ankle_angle, Eigen::Matrix<float,6,6> RR_, bool flag_);

        /*
         * @Name: KPS44_KPS6
         * @Description: Transform KPS44 Matrix to KPS6 Vector.
         * @Input: --Eigen::Matrix<float,4,4>: T_end
         * @Output: Eigen::Matrix<float,1,6> KPS6
        */

        Eigen::Matrix<float,1,6> KPS44_KPS6(Eigen::Matrix<float,4,4> T_end);

        /*
         * @Name: KPS6_KPS44
         * @Description: Transform KPS6 Vector to KPS44 Matrix.
         * @Input: --Eigen::Matrix<float,6,1> KPS6
         * @Output: Eigen::Matrix<float,4,4> KPS44
        */

        Eigen::Matrix<float,4,4> KPS6_KPS44(Eigen::Matrix<float,6,1> KPS6);

        /*
         * @Name: F_Tran_base
         * @Description: Transoform F_tool_ to F_Base by RR_ which is descripted based on Base Coordinate.
         * @Input: --Eigen::Matrix<float,6,6> RR_;
         *         --Eigen::Matrix<float,6,1> F_tool_;
         * @Output: --Eigen::Matrix<float,6,1> F_base
        */

        Eigen::Matrix<float,6,1> F_Tran_base(Eigen::Matrix<float,6,6> RR_,Eigen::Matrix<float,6,1> F_tool_);
    private:
        float d1;
        float a3;
        float a4;
        Eigen::Matrix<float,4,4> T_base;
        Eigen::Matrix<float,4,4> T1;
        Eigen::Matrix<float,4,4> T2;
        Eigen::Matrix<float,4,4> T3;
        Eigen::Matrix<float,4,4> T4;
        Eigen::Matrix<float,4,4> T_tool;

        Eigen::Matrix<float,4,4> T_base_Feed;
        Eigen::Matrix<float,4,4> T1_Feed;
        Eigen::Matrix<float,4,4> T2_Feed;
        Eigen::Matrix<float,4,4> T3_Feed;
        Eigen::Matrix<float,4,4> T4_Feed;
        Eigen::Matrix<float,4,4> T_tool_Feed;

        Eigen::Matrix<float,4,4> T_end;

        float angle[4];

        Eigen::Matrix<float,6,4> Jn_tool;
        Eigen::Matrix<float,6,4> Jn_base;

        Eigen::Matrix<float,6,6> RR;

        Eigen::Matrix<float,6,1> F_tool;
        Eigen::Matrix<float,6,1> F_base;
};

Eigen::Matrix<float,4,4> Exoskeleton_kinetic::forward_kinematics(float Hip_angle, float Thigh_angle, float Calf_angle, float Ankle_angle)
{
    T1 <<   cos(Hip_angle), -sin(Hip_angle), 0, 0,
            sin(Hip_angle),  cos(Hip_angle), 0, 0,
            0,0,1,d1,
            0,0,0,1;
    T2 <<   cos(Thigh_angle), -sin(Thigh_angle), 0, 0,
            0,0,-1,0,
            sin(Thigh_angle), cos(Thigh_angle),0,0,
            0,0,0,1;
    T3 <<   cos(Calf_angle), -sin(Calf_angle), 0, a3,
            sin(Calf_angle), cos(Calf_angle), 0, 0,
            0,0,1,0,
            0,0,0,1;
    T4 <<   cos(Ankle_angle), -sin(Ankle_angle), 0, a4,
            sin(Ankle_angle), cos(Ankle_angle), 0, 0,
            0,0,1,0,
            0,0,0,1;

    T_end = T_base*T1*T2*T3*T4;

    RR  <<  T_end(0,0), T_end(0,1), T_end(0,2), 0, 0, 0,
            T_end(1,0), T_end(1,1), T_end(1,2), 0, 0, 0,
            T_end(2,0), T_end(2,1), T_end(2,2), 0, 0, 0,
            0, 0, 0, T_end(0,0), T_end(0,1), T_end(0,2),
            0, 0, 0, T_end(1,0), T_end(1,1), T_end(1,2),
            0, 0, 0, T_end(2,0), T_end(2,1), T_end(2,2);

    return T_end;
}

Eigen::Matrix<float,4,1> Exoskeleton_kinetic::inverse_kinematics(Eigen::Matrix<float,4,4> T_end)
{
    Eigen::Matrix<float,4,1> angle;
    //Hip_Angle
    angle(0,0) = atan2(T_end(0,2),-T_end(1,2));

    //Calf_Angle
    float equ1 = pow(T_end(0,3),2)+pow(T_end(1,3),2);
    float equ2 = pow(T_end(2,3)-d1,2);
    float equ3 = equ1+equ2;
    float c3 = (equ3 - pow(a3,2) - pow(a4,2))/(2*a3*a4);
    float s3;
    if((1-pow(c3,2))<1e-20)
    {
        s3 = 0;
    }
    else
    {
        s3 = sqrt(1-pow(c3,2));
    }
    angle(2,0) = atan2(s3,c3);

    //Thigh_Angle
    float x = sqrt(pow(T_end(0,3),2)+pow(T_end(1,3),2));
    float y = T_end(2,3)-d1;
    if (T_end(0,3)<0)
    {
        x = -x;
    }
    float k1 = a4*cos(angle[2])+a3;
    float k2 = -sin(angle[2])*a4;
    angle(1,0) = atan2(y,x) + atan2(k2,k1);

    //Ankle_Angle
    float phi = atan2(T_end(2,0),T_end(2,1));
    angle(3,0)  = phi-angle[1]-angle[2];
    std::cout<<"Inverse Angle: ";
    for (int i = 0; i < 4; i++)
    {
        angle(i,0) = fmod(angle(i,0),(PI/2));
        std::cout<<angle(i,0)<<" ";
    }
    std::cout<<std::endl;
    return angle;
}

Eigen::Matrix<float,6,4> Exoskeleton_kinetic::Jaccobi(
    float Hip_angle,
    float Thigh_angle,
    float Calf_angle,
    float Ankle_angle,
    Eigen::Matrix<float,6,6> RR_,
    bool flag_)//flag_: 是否在基坐标系下描述
{
    T1_Feed <<  cos(Hip_angle), -sin(Hip_angle), 0, 0,
                sin(Hip_angle),  cos(Hip_angle), 0, 0,
                0,0,1,d1,
                0,0,0,1;
    T2_Feed <<  cos(Thigh_angle), -sin(Thigh_angle), 0, 0,
                0,0,-1,0,
                sin(Thigh_angle), cos(Thigh_angle),0,0,
                0,0,0,1;
    T3_Feed <<  cos(Calf_angle), -sin(Calf_angle), 0, a3,
                sin(Calf_angle), cos(Calf_angle), 0, 0,
                0,0,1,0,
                0,0,0,1;
    T4_Feed <<  cos(Ankle_angle), -sin(Ankle_angle), 0, a4,
                sin(Ankle_angle), cos(Ankle_angle), 0, 0,
                0,0,1,0,
                0,0,0,1;

    Eigen::Matrix<float,4,4> k1,k2,k3,k4;
    k4 = T_tool_Feed;
    k3 = T4_Feed*T_tool_Feed;
    k2 = T3_Feed*T4_Feed*T_tool_Feed;
    k1 = T2_Feed*T3_Feed*T4_Feed*T_tool_Feed;
    //TODO:Checked.
    // std::cout<<"k4: "<<std::endl;
    // std::cout<<k4<<std::endl;
    // std::cout<<"k3: "<<std::endl;
    // std::cout<<k3<<std::endl;
    // std::cout<<"k2: "<<std::endl;
    // std::cout<<k2<<std::endl;
    // std::cout<<"k1: "<<std::endl;
    // std::cout<<k1<<std::endl;

    Jn_tool(0,0) = -k1(0,0)*k1(1,3)+k1(1,0)*k1(0,3);
    Jn_tool(1,0) = -k1(0,1)*k1(1,3)+k1(1,1)*k1(0,3);
    Jn_tool(2,0) = -k1(0,2)*k1(1,3)+k1(1,2)*k1(0,3);
    Jn_tool(3,0) = k1(2,0);
    Jn_tool(4,0) = k1(2,1);
    Jn_tool(5,0) = k1(2,2);

    Jn_tool(0,1) = -k2(0,0)*k2(1,3)+k2(1,0)*k2(0,3);
    Jn_tool(1,1) = -k2(0,1)*k2(1,3)+k2(1,1)*k2(0,3);
    Jn_tool(2,1) = -k2(0,2)*k2(1,3)+k2(1,2)*k2(0,3);
    Jn_tool(3,1) = k2(2,0);
    Jn_tool(4,1) = k2(2,1);
    Jn_tool(5,1) = k2(2,2);

    Jn_tool(0,2) = -k3(0,0)*k3(1,3)+k3(1,0)*k3(0,3);
    Jn_tool(1,2) = -k3(0,1)*k3(1,3)+k3(1,1)*k3(0,3);
    Jn_tool(2,2) = -k3(0,2)*k3(1,3)+k3(1,2)*k3(0,3);
    Jn_tool(3,2) = k3(2,0);
    Jn_tool(4,2) = k3(2,1);
    Jn_tool(5,2) = k3(2,2);

    Jn_tool(0,3) = -k4(0,0)*k4(1,3)+k4(1,0)*k4(0,3);
    Jn_tool(1,3) = -k4(0,1)*k4(1,3)+k4(1,1)*k4(0,3);
    Jn_tool(2,3) = -k4(0,2)*k4(1,3)+k4(1,2)*k4(0,3);
    Jn_tool(3,3) = k4(2,0);
    Jn_tool(4,3) = k4(2,1);
    Jn_tool(5,3) = k4(2,2);

    if (flag_)
    {
        Jn_base = RR_*Jn_tool;
        return Jn_base;
    }
    else
    {
        return Jn_tool;
    }

}

Eigen::Matrix<float, 1,6> Exoskeleton_kinetic::KPS44_KPS6(Eigen::Matrix<float,4,4> T_end)
{
    Eigen::Matrix<float,1,6> KPS6;
    Eigen::Matrix<float,1,3> rpy;
    Eigen::Matrix<float,3,4> T;
    T  <<   1,0,0,0,
            0,1,0,0,
            0,0,1,0;
    Eigen::Matrix<float,4,1> P;
    P  <<   0,0,0,1;
    Eigen::Matrix<float,3,1> H  = T*(T_end*P);
    rpy(0,1) = atan2(-T_end(2,0),sqrt(T_end(0,0)*T_end(0,0)+T_end(1,0)*T_end(1,0)));
    if (fabs(fabs(rpy(0,1))-PI/2.0)<1e-6)
    {
        if (rpy(0,1)>0)
        {
            rpy(0,1) = PI/2.0;
            rpy(0,2) = 0.0;
            rpy(0,0) = atan2(T_end(0,1),T_end(1,1));
        }
        else
        {
            rpy(0,1) = -PI/2.0;
            rpy(0,2) = 0.0;
            rpy(0,3) = -atan2(T_end(0,1),T_end(1,1));
        }
    }
    else
    {
        float cp = cos(rpy(0,1));
        rpy(0,2) = atan2(T_end(1,0)/cp,T_end(0,0)/cp);
        rpy(0,0) = atan2(T_end(2,1)/cp,T_end(2,2)/cp);
    }
    KPS6 << H(0,0), H(1,0), H(2,0), rpy(0,0), rpy(0,1), rpy(0,2);

    return KPS6;
}

Eigen::Matrix<float,4,4> Exoskeleton_kinetic::KPS6_KPS44(Eigen::Matrix<float,6,1> KPS6_)
{
    float x,y,z,p,q,r;

    x = KPS6_(0,0);
    y = KPS6_(1,0);
    z = KPS6_(2,0);
    p = KPS6_(3,0);
    q = KPS6_(4,0);
    r = KPS6_(5,0);

    Eigen::Matrix<float,4,4> I;

    I  <<   1.0,0.0,0.0,0.0,
            0.0,1.0,0.0,0.0,
            0.0,0.0,1.0,0.0,
            0.0,0.0,0.0,1.0;

    float sx,cx,sy,cy,sz,cz;
    sx = sin(p);
    cx = cos(p);
    sy = sin(q);
    cy = cos(q);
    sz = sin(r);
    cz = cos(r);

    I(0,0) = cy*cz;
    I(0,1) = cz*sx*sy - cx*sz;
    I(0,2) = sx*sz + cx*cz*sy;
    I(1,0) = cy*sz;
    I(1,1) = cx*cz + sx*sy*sz;
    I(1,2) = cx*sy*sz - cz*sx;
    I(2,0) = -sy;
    I(2,1) = cy*sx;
    I(2,2) = cx*cy;
    I(0,3) = x;
    I(1,3) = y;
    I(2,3) = z;
    I(3,3) = 1;

    return I;
}

Eigen::Matrix<float,6,1> Exoskeleton_kinetic::F_Tran_base(Eigen::Matrix<float,6,6> RR_,Eigen::Matrix<float,6,1> F_tool_)
{
    F_tool = F_tool_;
    F_base = RR_*F_tool_;
    return F_base;
}