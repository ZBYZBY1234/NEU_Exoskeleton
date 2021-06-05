function T = fcn(q)
theta1 = q(2);
theta2 = q(3);
theta3 = q(4);
theta4 = q(5);
d1 = -0.097;
a3 = -sqrt(0.23966^2+0.28189^2);
a4 = -sqrt(-0.15989^2+0.31135^2);
Tbase = [1 0 0 0;
         0 1 0 0;
         0 0 1 d1;
         0 0 0 1];
    
T1=[1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];

T2=[cos(theta1) -sin(theta1) 0 0;
    sin(theta1)  cos(theta1)  0 0;
    0   0   1 0;
    0   0   0 1];
T3=[cos(theta2) -sin(theta2) 0 0;
    0   0  -1 0;
    sin(theta2)  cos(theta2)  0 0;
    0   0   0 1];
T4=[cos(theta3) -sin(theta3) 0 a3;
    sin(theta3) cos(theta3)  0 0;
    0  0   1 0;
    0  0   0 1];
Ttool = [cos(theta4) -sin(theta4) 0 a4;
         sin(theta4)  cos(theta4) 0 0;
         0    0 1 0;
         0    0 0 1];

T = Tbase*T1*T2*T3*T4*Ttool