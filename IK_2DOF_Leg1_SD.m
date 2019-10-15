function thetalist = IK_2DOF_Leg1_SD(T)
W=0.1;  % the x direction coordination the first joint of leg
L=0.4/2; 

l1 = 0.1;%change length
l2 = 0.3;
d=0.06;
r = sqrt((T(2)-L+d/2)^2+T(3)^2);

theta = asin((T(2)-L+d/2)/r);

thetalist = [acos((r^2+l1^2-l2^2)/(2*r*l1))-theta;acos((r^2+l1^2-l2^2)/(2*r*l1))+theta];
end
