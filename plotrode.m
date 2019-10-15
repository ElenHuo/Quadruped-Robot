% The files is used to draw rode in RC_19
% EXAMPLE
% 
% OUTPUT
% graphics of rode in RC_19

function plotrode(z)

plotcube(-0.935,3.5,-0.5018,0,0,pi/4,1.87,0.3,0.1,'y');hold on;% object

plotcube(-0.935,0,z,0,0,0,0.15,3.5,0.1,'r');hold on;% fence_left
plotcube(0.935,0,z,0,0,0,0.15,4,0.1,'r');hold on;% fence_right

plotcube(-0.935,3.5,z,0,0,pi/4,0.15,0.5*sqrt(2),0.1,'r');hold on;% fence_left
plotcube(0.935,4,z,0,0,pi/4,0.15,1.5*sqrt(2),0.1,'r');hold on;% fence_right

plotcube(-1.435,4,z,0,0,0,-1.5,0.15,0.1,'r');hold on;% fence_left
plotcube(-0.565,5.5,z,0,0,0,-1,0.15,0.1,'r');hold on;% fence_right

plotcube(-2.935,4,z,0,0,0,0.15,4,0.1,'r');hold on;% fence_left
plotcube(-1.565,5.5,z,0,0,0,0.15,1,0.1,'r');hold on;% fence_right

plotcube(-2.935,8,z,0,0,0,1.45,0.15,0.1,'r');hold on;% fence_left

plotcube(-1.485,8,z,0,atan(-4/16),0,1.6,0.15,0.1,'r');hold on;% fence_left
plotcube(-1.565,6.5,z,0,atan(-4/16),0,1.6,0.15,0.1,'r');hold on;% fence_right

plotcube(0.165,8,z+0.4,0,0,0,1.1,0.15,0.1,'r');hold on;% fence_left
plotcube(0.165,6.5,z+0.4,0,0,0,1.1,0.15,0.1,'r');hold on;% fence_left
end