% The files is used to draw cube
% EXAMPLE
% 
% INPUT 
% x,y,z,roll,pitch,yaw are used to define the 6D pose of quadruped
% k,m,q are used to define the long,width,hight of cube
%
% OUTPUT
% graphics of cube

function plotcube(x,y,z,roll,pitch,yaw,k,m,q,color)
R = Eul2R(roll,pitch,yaw,'ZYX');
vert = [0 0 0; 0 0+m*1 0; 0+k*1 0+m*1 0; 0+k*1 0 0 ; ...
    0 0 0+q*1;0 0+m*1 0+q*1; 0+k*1 0+m*1 0+q*1;0+k*1 0 0+q*1];
for i = 1:8
    T=R*vert(i,:)'+[x,y,z]';
    vert(i,:)=T';
end
fac = [1 2 3 4; ...
    2 6 7 3; ...
    4 3 7 8; ...
    1 5 8 4; ...
    1 2 6 5; ...
    5 6 7 8];
patch('Faces',fac,'Vertices',vert,'FaceColor',color);  % patch function
end