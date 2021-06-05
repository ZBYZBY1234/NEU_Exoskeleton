%%计算基坐标下的雅可比矩阵
clear all 
clc
%% define Q for robot
% syms s1 s2 s3 s4 s5 s6
% syms c1 c2 c3 c4 c5 c6
%% parameters

% syms d1 a1 a2 a3 d4 dt
%% transition matrix for robot
%% input 
Radian=[6,-14,-65,50,-87,-15];
Radian = Radian/180*pi;
%% input 
theta1 = Radian(1);
theta2 = Radian(2);
theta3 = Radian(3);
theta4 = Radian(4);
theta5 = Radian(5);
%% D-H parameters 
d1 = -0.097;
a3 = -sqrt(0.23966^2+0.28189^2);
a4 = -sqrt(-0.15989^2+0.31135^2);
Tbase = [1,0,0,0;0,1,0,0;0,0,1,d1;0,0,0,1];
T1=[1,0,0,0;0,1,0,0;0,0,1,0;0,0,0,1];
T2=[cos(theta1),-sin(theta1),0,0;sin(theta1),cos(theta1),0,0;0,0,1,0;0,0,0,1];
T3=[cos(theta2),-sin(theta2),0,0;0,0,-1,0;sin(theta2),cos(theta2),0,0;0,0,0,1];
T4=[cos(theta3),-sin(theta3),0,a3;sin(theta3),cos(theta3),0,0;0,0,1,0;0,0,0,1];
Ttool = [cos(theta4),-sin(theta4),0,a4;sin(theta4),cos(theta4),0,0;0,0,1,0;0,0,0,1];
%% Tbase T1~T6 Ttool
% KPS44=Tbase*T1*T2*T3*T4*T5*T6*Ttool;
%k7=Ttool;
k = zeros(4,4,4);
k(:,:,4)=Ttool;
k(:,:,3)=T4*Ttool;
k(:,:,2)=T3*T4*Ttool;
k(:,:,1)=T2*T3*T4*Ttool;
Jn=zeros(6,4);
for i=1:4
    Jn(1,i)=-k(1,1,i)*k(2,4,i)+k(2,1,i)*k(1,4,i);
    Jn(2,i)=-k(1,2,i)*k(2,4,i)+k(2,2,i)*k(1,4,i);
    Jn(3,i)=-k(1,3,i)*k(2,4,i)+k(2,3,i)*k(1,4,i);
    Jn(4,i)=k(3,1,i);
    Jn(5,i)=k(3,2,i);
    Jn(6,i)=k(3,3,i);
end
Jn
FK = Tbase * T1 * T2 * T3 * T4 * Ttool;
FK66 = FK(1:3,1:3);
RR = [ FK66 zeros(3,3);
       zeros(3,3) FK66];
Jn_Base = RR*Jn
inv(Jn_Base)
V=[-1,3,1,-1,1,1];
V= V*pi/180;

V=reshape(V,6,1);
Qv2 = (Jn_Base *V)'
%Qv=Jn_Base\V;
%Qv=(Qv * 180/pi)'

	



