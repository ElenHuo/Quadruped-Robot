%Dynamic
function taulist = Dynamic_SD(theta1,theta2)
F_s = 2*10;

W=0;  % the x direction coordination the first joint of leg
L=0.4/2;  

l1 = 0.1;%change length
l2 = 0.3;
d  = 0.06;

l3 = sqrt( l1^2 + d^2 - 2 * l1 * d * cos(theta1+pi/2) );
l4 = sqrt( l1^2 + d^2 - 2 * l1 * d * cos(theta2+pi/2) );

%
afa1 = acos( (d^2 + l4^2 - l1^2) / (2 * d * l4) );
afa2 = acos( (d^2 + l3^2 - l1^2) / (2 * d * l3) );


if(cos(theta1)<0&&cos(theta2)<0)
    fai1 = theta1 + pi/2 + afa1;
    fai2 = theta2 + pi/2 + afa2;
else
    fai1 = theta1 + pi/2 - afa1;
    fai2 = theta2 + pi/2 - afa2;
end
%
D = sqrt( l1^2 + l4^2 - 2 * l1 * l4 * cos(fai1) );
w =  acos( D / (2 * l2) );
w1 = w;
w2 = w;

%
beta1 = acos( (l1^2 + D^2 -l4^2) / (2 * l1 * D) );
beta2 = acos( (l1^2 + D^2 -l3^2) / (2 * l1 * D) );

if(cos(theta1)<0&&cos(theta2)<0)
    kesi1 = w1 - beta1;
    kesi2 = w2 - beta2;
else
    kesi1 = beta1 + w1;
    kesi2 = beta2 + w2;
end

angle1 = pi/2-(pi-theta1-kesi1);
angle2 = pi-(pi/2-(pi-theta2-kesi2));
angle_a = angle2-pi/2;
angle_b = angle2-angle_a-angle1;
F1 = sin(angle_a)*F_s/sin(pi-angle_a-angle_b);
F2 = sin(angle_b)*F_s/sin(pi-angle_a-angle_b);
F3 = F1*sin(pi-kesi1);
F4 = F2*sin(pi-kesi2);
tau1 = F3*l1;
tau2 = F4*l1;
taulist = [tau1,tau2];
end
% taulist=[];
% for i=1.344052:0.1:1.4070682
% for j=0.499159:0.2:1.256637
% tau = Dynamic_SD(i,j);
% taulist = [taulist;tau];
% plot(taulist(:,1),'*');
% hold on;
% end
% end

