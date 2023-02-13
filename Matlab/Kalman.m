function [Q, aPoP] = Kalman(Acc, Gyro, Mag, oldQuat, deltaT, oldP)
%KALMAN Summary of this function goes here
%   Detailed explanation goes here
Acc = Acc/norm(Acc);
Mag = Mag/norm(Mag);

A = eye(4,4) + 0.5*deltaT* ...
    [0      , -Gyro(1), -Gyro(2), -Gyro(3); ...
     Gyro(1), 0       , Gyro(3) , -Gyro(2); ...
     Gyro(2), -Gyro(3), 0       , Gyro(1) ; ...
     Gyro(3), Gyro(2) , -Gyro(1), 0       ];
V1 = eye(3,3);
V2 = eye(3,3);
% R and Q: covariance matrices of the read and the state equation of the 
Q = eye(4,4) * 10e-5;
R1 = eye(3,3) * 1e-2;
R2 = eye(3,3) * 1e-1;

%aPrP = [0.1250, 0.0003, 0.0003, 0.0003; ... 
%      0.0003, 0.1250, 0.0003, 0.0003; ... 
%      0.0003, 0.0003, 0.1250, 0.0003; ... 
%      0.0003, 0.0003, 0.0003, 0.1250]; 

% A priori attitude quaternion based on Gyro data
aPrQ = A*oldQuat;
aPrQ = aPrQ * (1/norm(aPrQ));

% A priori noise covariance matrix

aPrP = A * oldP * A' + Q;

% ----------------------------------
%           Stage 1 start
% ----------------------------------

% Jacobian matrix H1
H1 = [-aPrQ(3)  , aPrQ(4)   ,-aPrQ(1)  , aPrQ(2); ...
       aPrQ(2)  , aPrQ(1)   , aPrQ(4)  , aPrQ(3); ...
       aPrQ(1)  ,-aPrQ(2)   ,-aPrQ(3)  , aPrQ(4)] ...
     * 2;
 
% Kalan gain stage 1
K1 = aPrP*H1' / (H1*aPrP*H1' + V1*R1*V1');

% Gravity vector estimation
h1 = [2*aPrQ(2)*aPrQ(4) - 2*aPrQ(1)*aPrQ(3); ...
      2*aPrQ(1)*aPrQ(2) + 2*aPrQ(3)*aPrQ(4); ...
      aPrQ(1)^2 - aPrQ(2)^2 - aPrQ(3)^2 + aPrQ(4)^2];
  
% Calculate the correction quaternion
cQ1 = K1*(Acc' - h1);

% Ensure yaw angle is unaffected
cQ1(4) = 0;

% Calculae the a posteriori state estimation
Q1 = aPrQ + cQ1;
Q1 = Q1 * (1/norm(Q1));

% Calculae the a posteriori error covariance matrix
aPoP1 = (eye(4,4) - K1 * H1) * aPrP;
 
% ----------------------------------
%           Stage 2 start
% ----------------------------------

% Jacobian matrix H2
H2 = [ aPrQ(4)  , aPrQ(3)   , aPrQ(2)  , aPrQ(1); ...
       aPrQ(1)  ,-aPrQ(2)   ,-aPrQ(3)  ,-aPrQ(4); ...
       aPrQ(2)  ,-aPrQ(1)   , aPrQ(4)  , aPrQ(3)] ...
     * 2;

% Kalan gain stage 2
K2 = aPrP*H2' / (H2*aPrP*H2' + V2*R2*V2');

% Magnetometer vector estimation
h2 = [2*aPrQ(2)*aPrQ(3) + 2*aPrQ(1)*aPrQ(4); ...
      aPrQ(1)^2 - aPrQ(2)^2 - aPrQ(3)^2 - aPrQ(4)^2; ...
      2*aPrQ(3)*aPrQ(4) - 2*aPrQ(1)*aPrQ(2)];
  
% Calculate the correction quaternion
cQ2 = K2*(Mag' - h2);

% Ensure pitch and roll angle is unaffected
cQ2([2 3]) = 0;

% Calculae the a posteriori state estimation
Q = Q1 + cQ2;
Q = Q * (1/norm(Q));

% Calculae the a posteriori error covariance matrix
aPoP = (eye(4,4) - K2 * H2) * aPoP1;
