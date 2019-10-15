% genernate the best thetalist for dog running
clear;
fid = fopen('TG_result.txt','w+');
W=0.1;  % the x direction coordination the first joint of leg
L=0.4/2; 
l1 = 0.1;%change length
l2 = 0.3;
d=0.06;

smax = 0.03;
tmin = 1;
tmax = 0.3;
count = 0;
for theta1=0/180*pi:(120)/(180*100)*pi:120/180*pi  %15
    for theta2 = 0/180*pi:(120)/(180*100)*pi:120/180*pi
        T = FK_2DOF_Leg1_SD_e(theta1,theta2);
        T1 = [W,L-d/2-0.1,T(3)];
       thetalist = IK_2DOF_Leg1_SD(T1);
       I_theta1 = thetalist(1);
       I_theta2 = thetalist(2);
        if abs(theta2-I_theta2)> abs(theta1-I_theta1)
            t  = abs(theta2-I_theta2)/(2*pi);
        else 
            t  = abs(theta1-I_theta1)/(2*pi);
        end
        s = T(2)-L+d/2+0.1;
        count = count + 1;
        
%         if (t>0.1)%{theta2>I_theta2%}t<0.5 && t>0.01
            tmin = t;
            smax = s;
            fprintf(fid, '-------------------\r\n');
            fprintf(fid, 'The %d times\r\n', count);
            fprintf(fid, 'theta1=%f,theta2=%f\r\n', theta1,theta2);
            fprintf(fid, 'I_theta1=%f,I_theta2=%f\r\n', I_theta1,I_theta2);
            fprintf(fid, 'smax=%f,tmin=%f,v=%f\r\n', smax,tmin,smax/tmin);
            fprintf(fid, '-------------------\r\n');
%         end   
    end
end
fclose(fid);
% The 15417 times
% theta1=0.000000,theta2=-1.047198,theta3=1.601514	-60   91.76
% I_theta1=0.000000,I_theta2=-0.896924,I_theta3=1.565565  -51.39 89.7003
% smax=0.066528,tmin=0.028700   2.3180
