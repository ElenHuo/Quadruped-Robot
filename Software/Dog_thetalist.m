% Using tracjectory of one leg to compute innverse thetalist
function thetalist = Dog_thetalist(footstep,z,H0,ini_Location,Ty,step)
% Input: tracjectory
% Output: inverse thetalist
W=0.1;  % the x direction coordination the first joint of leg
L=0.4/2; 
d=0.06;
[trajectory_y,trajectory_z] = Dog_tracjectory(footstep,H0,Ty,step);
n = length(trajectory_y);

thetalist = [];

for i=1:1:n
    T = [W; L-d/2+trajectory_y(i)-ini_Location; z+trajectory_z(i)];
    thetalist1=IK_2DOF_Leg1_SD(T);
    thetalist = [thetalist;thetalist1(1), thetalist1(2)];
end