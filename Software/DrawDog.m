function DrawDog(x,y,z,roll,pitch,yaw,thetalist1,thetalist2,thetalist3,thetalist4,fcla)
W=0.1;  % the x direction coordination the first joint of leg
L=0.4/2;  
R = Eul2R(roll,pitch,yaw,'ZYX');
plot3(x,y,0,'r+');hold on;grid on;
DrawLeg(1,x,y,z,roll,pitch,yaw,thetalist1(1),thetalist1(2))
DrawLeg(2,x,y,z,roll,pitch,yaw,thetalist2(1),thetalist2(2))
DrawLeg(3,x,y,z,roll,pitch,yaw,thetalist3(2),thetalist3(1))
DrawLeg(4,x,y,z,roll,pitch,yaw,thetalist4(2),thetalist4(1))

Connect3D(R*[+W;+L;0]+[x;y;z], R*[-W;+L;0]+[x;y;z],'b',2);hold on;
Connect3D(R*[-W;+L;0]+[x;y;z], R*[-W;-L;0]+[x;y;z],'b',2);hold on;
Connect3D(R*[-W;-L;0]+[x;y;z], R*[+W;-L;0]+[x;y;z],'b',2);hold on;
Connect3D(R*[+W;-L;0]+[x;y;z], R*[+W;+L;0]+[x;y;z],'b',2);hold on;
%axis([-0.4,0.4,-1,3,-0.5,0.1]);
% plotrode(z)
drawnow;
%view(134,30);
if(fcla)
    cla;
end