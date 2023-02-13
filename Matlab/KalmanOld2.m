function [gravityQuat, magQuat, updateQuat, updateP1, updateP2] = KalmanOld2(Acc, Gyro, Mag, oldQuat, deltaT, oldP1, oldP2)
%KALMAN Summary of this function goes here
%   Detailed explanation goes here
updateP1 = 0;
updateP2 = 0;
updateQuat1 = [0 0 0 0];
updateQuat2 = [0 0 0 0];
oldQuat = oldQuat';
g = [0 0 1];
A = [0,       -Gyro(1), -Gyro(2), -Gyro(3); ...
     Gyro(1), 0       , Gyro(3) , -Gyro(2); ...
     Gyro(2), -Gyro(3), 0       , Gyro(1) ; ...
     Gyro(3), Gyro(2) , -Gyro(1), 0       ];
% H and V: Jacobean matrices of the partial derivatives with respect to 
% the quaternion and to the noise of the nonlinear equation
H = eye(4,4);
V = eye(4,4);
% R and Q: covariance matrices of the read and the state equation of the 
Q = eye(4,4) * 10e-4;
R = eye(4,4) * 10e-3;

% A priori attitude quaternion based on Gyro data
%ADD UNIT VECTOR AND REMOVE oldQuat
aPrioriQuat = oldQuat + 0.5*deltaT*(A*oldQuat);

if(norm(Acc) ~= 0)

% Set the forth element of the apriori quaternion to 0 as to not influence
% yaw
aPrioriQuat1 = aPrioriQuat;
aPrioriQuat1(4) = 0;
aPrioriQuat1 = aPrioriQuat1/norm(aPrioriQuat1);

% Measured quaternion based on gravity readings from the accelerometer
n = cross(Acc,g) / norm(cross(Acc,g));
omega = atan2( norm(cross(Acc,g)) , dot(Acc,g));
gravityQuat = [cos(omega/2) n*sin(omega/2)]';
gravityQuat(4) = 0;
gravityQuat = gravityQuat*(1/norm(gravityQuat));

% Calculate the Kalman gain
F = 0.5*deltaT*A + eye(4,4);
pP1 = F*oldP1*F' + Q; % the a priori error covariance matrix
K1 = pP1*H' / (H*pP1*H' + V*R*V');

% Update covariance
updateP1 = (eye(4,4)-K1*H)*pP1;

% Multiply the residual with the kalman gain (this is the typical state 
% estimate equation but written in a more intuitive way instead of the
% conventional way)

% ---------------------------
% ehhhhhm this has to be used
% ---------------------------
updateQuat1 = (eye(4,4) - K1*H)*aPrioriQuat1 + K1*(H*gravityQuat);
updateQuat1 = (updateQuat1*(1/norm(updateQuat1)))';
end

% Only continue to stage 2 if magnetometer data is available
if(norm(Mag) ~= 0)

% Set the second and third element of the apriori quaternion to 0 as to not 
% influence roll and pitch
aPrioriQuat2 = aPrioriQuat;
%aPrioriQuat2([2 3]) = 0;
aPrioriQuat2 = aPrioriQuat2/norm(aPrioriQuat2);

% Measured quaternion based on gravity readings from the accelerometer
Mag = Mag/norm(Mag);
m = [-0.5978    0.4256   -0.6794];
%n = quatrotate(updateQuat1,[0 0 1]);
n = cross(Mag,m) / norm(cross(Mag,m));
omega = atan2( norm(cross(Mag,m)) , dot(Mag,m));
magQuat = [cos(omega/2) n*sin(omega/2)]';
magQuat = magQuat*(1/norm(magQuat));

% Calculate the Kalman gain
F = 0.5*deltaT*A + eye(4,4);
pP2 = F*oldP2*F' + Q; % the a priori error covariance matrix
K2 = pP2*H' / (H*pP2*H' + V*R*V');

% Update covariance
updateP2 = (eye(4,4)-K2*H)*pP2;

% Multiply the residual with the kalman gain (this is the typical state 
% estimate equation but written in a more intuitive way instead of the
% conventional way)
updateQuat2 = (eye(4,4) - K2*H)*aPrioriQuat2 + K2*(H*magQuat);
updateQuat2 = (updateQuat2*(1/norm(updateQuat2)))';
end
q1 = quaternion(updateQuat1(1), updateQuat1(2), updateQuat1(3), updateQuat1(4));
q2 = quaternion(updateQuat2(1), updateQuat2(2), updateQuat2(3), updateQuat2(4));
eul1 = quat2eul(q1);
eul2 = quat2eul(q2);
%eul1(1) = 0;
eul2([1 2 3]) = 0;
eul = eul1 + eul2;
updateQuat = eul2quat(eul);
updateQuat = updateQuat*(1/norm(updateQuat)); 
end

% partially: https://github.com/danicomo/9dof-orientation-estimation/blob/master/KalmanFilter_V11/KalmanFilter_V14_GN.m