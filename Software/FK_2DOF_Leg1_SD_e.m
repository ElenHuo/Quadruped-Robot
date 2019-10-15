function T =   FK_2DOF_Leg1_SD_e( w1, w2 )
w1 = pi - w1;
w2 = pi - w2;

W=0.1;  % the x direction coordination the first joint of leg
L=0.4/2; 
l1 = 0.1;%change length
l2 = 0.3;
d=0.06;

r = sqrt(l2^2-l1^2*(sin(w1/2+w2/2))^2)-l1*cos(w1/2+w2/2);
theta = w1/2-w2/2;
y = r * sin(theta)+L-d/2;
z = r * cos(theta);
T=[W,y,-z];
end