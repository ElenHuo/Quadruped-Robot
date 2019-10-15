function Move_Dog_S(s,x_w_0,y_w_0,z_w_0,roll,pitch,yaw)
W=0.1;  % the x direction coordination the first joint of leg
L=0.4/2;  
footstep1  = 0.137627;
N = ceil(s/footstep1);
z1 = -0.3699;
H01 = 0.01;
H02 = 0;
step = 30;
Ty = 0.077221;
ini_Location = 0.10;
thetalist1 = Dog_thetalist(footstep1,z1,H01,ini_Location,Ty,step);
thetalist2 = Dog_thetalist(footstep1,z1,H02,ini_Location,Ty,step);
[trajectory_y1,trajectory_z1] = Dog_tracjectory(footstep1,H01,Ty,step);

trajectory_y = (trajectory_y1+trajectory_y1)/2; % go straight
footstpe = (footstep1+footstep1)/2;
%view(134,30);
for i = 1:N
    for j = 1:step+1
        R = Eul2R(roll,pitch,yaw,'ZYX');
        P = R*[0;trajectory_y(j)+(i-1)*footstpe;0];
        x_w = P(1)+x_w_0;y_w = P(2)+y_w_0;z_w = P(3)+z_w_0;
        if rem(i,2)
            if j==1 || j==step+1
                DrawDog(x_w,y_w,z_w,roll,pitch,yaw,...
                    thetalist1(j,:),...
                    thetalist2(end-j+1,:),...
                    thetalist1(end-j+1,:),...
                    thetalist2(j,:),0);
            else
                DrawDog(x_w,y_w,z_w,roll,pitch,yaw,...
                    thetalist1(j,:),...
                    thetalist2(end-j+1,:),...
                    thetalist1(end-j+1,:),...
                    thetalist2(j,:),1);
            end
        else 
            if j==1 || j==step+1
                DrawDog(x_w,y_w,z_w,roll,pitch,yaw,...
                    thetalist2(end-j+1,:),...
                    thetalist1(j,:),...
                    thetalist2(j,:),...
                    thetalist1(end-j+1,:),0);
            else
                DrawDog(x_w,y_w,z_w,roll,pitch,yaw,...
                    thetalist2(end-j+1,:),...
                    thetalist1(j,:),...
                    thetalist2(j,:),...
                    thetalist1(end-j+1,:),1);
            end
        end
    end
end