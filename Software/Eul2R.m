% converts a set of Euler angles, eul, 
% to the corresponding rotation matrix, rotm.
function R = Eul2R(roll, pitch, yaw, s)

if s == 'ZYX'
    R_roll = [1,0,0;
              0,cos(roll),-sin(roll);
              0,sin(roll),cos(roll)];
    R_pitch = [cos(pitch),0,sin(pitch);
               0,1,0;
               -sin(pitch),0,cos(pitch)];
    R_yaw = [cos(yaw),-sin(yaw),0;
             sin(yaw),cos(yaw),0;
             0,0,1];
    R = R_roll*R_pitch*R_yaw;
else
    R = eye(3);
end
