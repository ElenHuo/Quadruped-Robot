% genernate a tracjectory between two points
function [trajectory_y,trajectory_z] = Dog_tracjectory(footstep,H0,Ty,step)
t = 0:Ty/step:Ty;    % please write 10 times

S0 = footstep;

n=length(t);
trajectory_y = S0*(t/Ty-1/(2*pi)*sin(2*pi*t/Ty));
trajectory_z = [2*H0*(t(1:(n-1)/2)/Ty-1/(4*pi)*sin(4*pi*t(1:(n-1)/2)/Ty))...
,-2*H0*(t((n+1)/2:n)/Ty-1/(4*pi)*sin(4*pi*t((n+1)/2:n)/Ty))+2*H0];
