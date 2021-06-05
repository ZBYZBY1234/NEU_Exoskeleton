function q= fcn(T)
%theta-offset
% offset1 = 0;
% offset2 = pi/2-atan(0.23966/0.28189);
% offset3 = pi/2-atan(0.23966/0.28189)+atan(0.15989/0.31135);
% offset4 = -atan(0.15989/0.31135);
%%DH
d1 = -0.097;
a3 = -sqrt(0.23966^2+0.28189^2);
a4 = -sqrt(-0.15989^2+0.31135^2);
%%theta1
  theta1 = atan2(T(1,3),-T(2,3));
%%theta3
  equ1 = T(1,4)^2 + T(2,4)^2;
  equ2 = (T(3,4)-d1)^2;
  equ3 = equ1 + equ2;
  c3   = (equ3 - a3^2 - a4^2)/(2*a3*a4);
%   s3   = sqrt(1-c3^2);
  if 1-c3^2 < 1e-20
      s3 = 0;
  else
      s3   = sqrt(1-c3^2);
  end
  theta3 = atan2(s3,c3);
  
%%theta2
  
  x = (T(1,4)^2+T(2,4)^2)^0.5;
  y = T(3,4)-d1;
  if T(1,4) < 0
      x = -x;
  end
  k1 = (a4*cos(theta3)+a3);
  k2 = -sin(theta3)*a4;
  theta2 = atan2(y,x)+ atan2(k2,k1);
%   atan = atan2(k2,k1)
%   theta2 = -theta2;
  
%%theta4
  phi = atan2(T(3,1),T(3,2));
  theta4 = phi - theta2 - theta3;
  q = [0,theta1,theta2,theta3,theta4];
