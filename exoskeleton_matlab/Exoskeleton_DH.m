clear;
clc;

L(1)=Link('d', -0.097,'a', 0,                      'alpha', 0,    'offset', pi,                                              'qlim',[0 pi/2],'modified'); 
L(2)=Link('d', 0,'a', 0,                           'alpha', pi/2, 'offset', pi/2-atan(0.23966/0.28189),                      'qlim',[-atan(0.23966/0.28189) pi/2],'modified');
L(3)=Link('d', 0,'a', -sqrt(0.23966^2+0.28189^2),  'alpha', 0,    'offset', pi/2-atan(0.23966/0.28189)+atan(0.15989/0.31135),'qlim',[-pi/2+atan(0.23966/0.28189)-atan(0.15989/0.31135) pi/2],'modified');
L(4)=Link('d', 0,'a', -sqrt(-0.15989^2+0.31135^2), 'alpha', 0,    'offset', -atan(0.15989/0.31135),                          'qlim',[0 pi/2-atan(0.15989/0.31135)],'modified');

robot=SerialLink(L,'name','robot');

theta1 = unifrnd(0,pi/2,[1,10000]);
theta2 = unifrnd(-atan(0.23966/0.28189),pi/2,[1,10000]);
theta3 = unifrnd(-pi/2+atan(0.23966/0.28189)-atan(0.15989/0.31135),pi/2,[1,10000]);
theta4 = unifrnd(0,pi/2-atan(0.15989/0.31135),[1,10000]);

G= cell(10000, 3);%建立元胞数组
for n = 1:10000
    G{n} =[theta1(n) theta2(n) theta3(n) theta4(n)];
end                                         %产生3000组随机点
H1=cell2mat(G);                       %将元胞数组转化为矩阵
T=double(robot.fkine(H1));       %机械臂正解
figure(1)
scatter3(squeeze(T(1,4,:)),squeeze(T(2,4,:)),squeeze(T(3,4,:)),1,'.')%随机点图
robot.plot([0 0 0 0],'workspace',[-0.6001 0.6001 -0.6001 0.6001 -0.6001 0.6001])%机械臂图
view(180,0)%xz plot
view(-90,90)%xy plot
view(-90,0)%yz plot
% robot.teach([0 0 0 0])



