function DrawLeg(whichleg,x,y,z,roll,pitch,yaw,theta1,theta2)
W=0.1;  % the x direction coordination the first joint of leg
L=0.4/2; 
l1 = 0.1;%change length
l2 = 0.3;
d=0.06;
R = Eul2R(roll,pitch,yaw,'ZYX');
if whichleg == 1
    T = FK_2DOF_Leg1_SD_e(theta1,theta2);
    T1 = R*[T(1);T(2);T(3)]+[x;y;z];
    plot3(T1(1),T1(2),T1(3),'o');hold on;
    DrawCylinder(R*[W;L;0]+[x;y;z], R*[1;0;0], 0.02,0.02, 0);hold on;
    DrawCylinder(R*[W;L-d;0]+[x;y;z], R*[1;0;0], 0.02,0.02, 0);hold on;
    Connect3D(R*[W;L;0]+[x;y;z],R*[W;L+l1*sin(theta2);-l1*cos(theta2)]+[x;y;z] ,'b',2);hold on;
    Connect3D(R*[W;L-d;0]+[x;y;z],R*[W;L-d-l1*sin(theta1);-l1*cos(theta1)]+[x;y;z] ,'b',2);hold on;
    Connect3D(R*[T(1);T(2);T(3)]+[x;y;z],R*[W;L+l1*sin(theta2);-l1*cos(theta2)]+[x;y;z] ,'b',2);hold on;
    Connect3D(R*[T(1);T(2);T(3)]+[x;y;z],R*[W;L-d-l1*sin(theta1);-l1*cos(theta1)]+[x;y;z] ,'b',2);hold on;
    
elseif whichleg == 2
    T = FK_2DOF_Leg1_SD_e(theta1,theta2);
    T1 = R*[-2*W+T(1);T(2);T(3)]+[x;y;z];
    plot3(T1(1),T1(2),T1(3),'o');hold on;
    DrawCylinder(R*[-W;L;0]+[x;y;z], R*[1;0;0], 0.02,0.02, 0);hold on;
    DrawCylinder(R*[-W;L-d;0]+[x;y;z], R*[1;0;0], 0.02,0.02, 0);hold on;
    Connect3D(R*[-W;L;0]+[x;y;z],R*[-W;L+l1*sin(theta2);-l1*cos(theta2)]+[x;y;z] ,'b',2);hold on;
    Connect3D(R*[-W;L-d;0]+[x;y;z],R*[-W;L-d-l1*sin(theta1);-l1*cos(theta1)]+[x;y;z] ,'b',2);hold on;
    Connect3D(R*[-2*W+T(1);T(2);T(3)]+[x;y;z],R*[-W;L+l1*sin(theta2);-l1*cos(theta2)]+[x;y;z] ,'b',2);hold on;
    Connect3D(R*[-2*W+T(1);T(2);T(3)]+[x;y;z],R*[-W;L-d-l1*sin(theta1);-l1*cos(theta1)]+[x;y;z] ,'b',2);hold on;
    
elseif whichleg == 3
    T = FK_2DOF_Leg1_SD_e(theta1,theta2);
    T1 = R*[-2*W+T(1);-2*L+d+T(2);T(3)]+[x;y;z];
    plot3(T1(1),T1(2),T1(3),'o');hold on;
    DrawCylinder(R*[-W;-L;0]+[x;y;z], R*[1;0;0], 0.02,0.02, 0);hold on;
    DrawCylinder(R*[-W;-L+d;0]+[x;y;z], R*[1;0;0], 0.02,0.02, 0);hold on;%
    Connect3D(R*[-W;-L;0]+[x;y;z],R*[-W;-L-l1*sin(theta1);-l1*cos(theta1)]+[x;y;z] ,'b',2);hold on;
    Connect3D(R*[-W;-L+d;0]+[x;y;z],R*[-W;-L+d+l1*sin(theta2);-l1*cos(theta2)]+[x;y;z] ,'b',2);hold on;
    Connect3D(R*[-2*W+T(1);-2*L+d+T(2);T(3)]+[x;y;z],R*[-W;-L-l1*sin(theta1);-l1*cos(theta1)]+[x;y;z] ,'b',2);hold on;
    Connect3D(R*[-2*W+T(1);-2*L+d+T(2);T(3)]+[x;y;z],R*[-W;-L+d+l1*sin(theta2);-l1*cos(theta2)]+[x;y;z] ,'b',2);hold on;

elseif whichleg == 4
    T = FK_2DOF_Leg1_SD_e(theta1,theta2);
    T1 = R*[T(1);-2*L+d+T(2);T(3)]+[x;y;z];
    plot3(T1(1),T1(2),T1(3),'o');hold on;
    DrawCylinder(R*[W;-L;0]+[x;y;z], R*[1;0;0], 0.02,0.02, 0);hold on;
    DrawCylinder(R*[W;-L+d;0]+[x;y;z], R*[1;0;0], 0.02,0.02, 0);hold on;
    Connect3D(R*[W;-L;0]+[x;y;z],R*[W;-L-l1*sin(theta1);-l1*cos(theta1)]+[x;y;z] ,'b',2);hold on;
    Connect3D(R*[W;-L+d;0]+[x;y;z],R*[W;-L+d+l1*sin(theta2);-l1*cos(theta2)]+[x;y;z] ,'b',2);hold on;
    Connect3D(R*[T(1);-2*L+d+T(2);T(3)]+[x;y;z],R*[W;-L-l1*sin(theta1);-l1*cos(theta1)]+[x;y;z] ,'b',2);hold on;
    Connect3D(R*[T(1);-2*L+d+T(2);T(3)]+[x;y;z],R*[W;-L+d+l1*sin(theta2);-l1*cos(theta2)]+[x;y;z] ,'b',2);hold on;
end
axis equal;
end
