clear;clc;
DEG2RAD=pi/180;
SetupBipedRobot;
idxl=FindRoute(LLEG_J5);
idxr=FindRoute(RLEG_J5);
SetJointAngles(idxl,DEG2RAD*[15,-10,-30,60,-30,10]);
SetJointAngles(idxr,DEG2RAD*[-15,-10,-45,90,-45,10]);
uLINK(BODY).p = [0.0, 0.0, 0.7]';
uLINK(BODY).R = eye(3);
ForwardKinematics(1);  
clf               
DrawAllJoints(1);
view(38,14)
axis equal
zlim([0.1 1.3])
grid on

J = CalcJacobian(idxr)