function [updateQuat, updateP] = KalmanOld(Acc, Gyro, Mag, oldQuat, deltaT, oldP)
%KALMAN Summary of this function goes here
%   Detailed explanation goes here

oldQuat = oldQuat';
g = [0 0 1];
A = [0,       -Gyro(1), -Gyro(2), -Gyro(3); ...
     Gyro(1), 0       , Gyro(3) , -Gyro(2); ...
     Gyro(2), -Gyro(3), 0       , Gyro(1) ; ...
     Gyro(3), Gyro(2) , -Gyro(1), 0       ];
H = eye(4,4);
V = eye(4,4);
Q = eye(4,4) * 10e-4;
R = eye(4,4) * 10e-3;

%var=[10e-4 10e-4 10e-4]';
%Q=[var(1)+var(2)+var(3) -var(1)+var(2)-var(3) -var(1)-var(2)+var(3) var(1)-var(2)-var(3); ...
%	-var(1)+var(2)-var(3) var(1)+var(2)+var(3) var(1)-var(2)-var(3) -var(1)-var(2)+var(3); ...
%	-var(1)-var(2)+var(3) var(1)-var(2)-var(3) var(1)+var(2)+var(3) -var(1)+var(2)-var(3); ...
%	var(1)-var(2)-var(3) -var(1)+var(2)-var(3) -var(1)+var(2)-var(3) var(1)+var(2)+var(3)];
%sigmaR=[0.01 0.01 0.01 0.01]';
%R=[sigmaR(1,1) 0 0 0;0 sigmaR(2,1) 0 0;0 0 sigmaR(3,1) 0;0 0 0 sigmaR(4,1)];

% A priori attitude quaternion based on Gyro data
aPrioriQuat = oldQuat + 0.5*deltaT*(A*oldQuat);
aPrioriQuat = aPrioriQuat/norm(aPrioriQuat);

% A priori gravity vector
% F = [2*aPrioriQuat(2)*aPrioriQuat(4) - 2*aPrioriQuat(1)*aPrioriQuat(3); ...
     %2*aPrioriQuat(1)*aPrioriQuat(2) + 2*aPrioriQuat(3)*aPrioriQuat(4); ...
     %aPrioriQuat(1)^2 - aPrioriQuat(2)^2 - aPrioriQuat(3)^2 + aPrioriQuat(4)^2]; ...
% pG = g * F;

% measured quaternion based on gravity
%euld2 = [0, ...
      %rad2deg(atan2( Acc(1), sqrt(Acc(2)^2 + Acc(3)^2) )), ...
      %rad2deg(atan2( Acc(2), sign(Acc(3))*sqrt(Acc(1)^2 + Acc(3)^2) )) ];
%gravityQuad = compact(quaternion(euld2, 'eulerd', 'ZYX', 'frame'))';
n = cross(Acc,g) / norm(cross(Acc,g));
omega = atan2( norm(cross(Acc,g)) , dot(Acc,g));
gravityQuad = [cos(omega/2) n*sin(omega/2)]';
gravityQuad = gravityQuad*(1/norm(gravityQuad));

% Calculate the Kalman gain
%
% pP: the a priori error covariance matrix
% R and Q: covariance matrices of the read and the state equation of the 
% filter
% H and V: Jacobean matrices of the partial derivatives with respect to 
% the quaternion and to the noise of the nonlinear equation
const = deltaT/2;
F1=[1 -const*Gyro(1) -const*Gyro(2) -const*Gyro(3)];
F2=[const*Gyro(1) 1 const*Gyro(3) -const*Gyro(2)];
F3=[const*Gyro(2) -const*Gyro(3) 1 const*Gyro(1)];
F4=[-const*Gyro(3) const*Gyro(2) -const*Gyro(1) 1];
F=[F1;F2;F3;F4];

pP = F*oldP*F' + Q;
K1 = pP*H' / (H*pP*H' + V*R*V');

% Compairing a priory gravity vector with measured gravity resulting in a
% correction quanternion (residual)
% y = H*gravityQuad - H*aPrioriQuat;
%q1 = quaternion(gravityQuad(1), gravityQuad(2), gravityQuad(3), gravityQuad(4))
%q2 = quaternion(aPrioriQuat(1), aPrioriQuat(2), aPrioriQuat(3), aPrioriQuat(4))
%qBase =  quaternion(1, 0, 0, 0);
%y = compact(conj(q2)*q1*qBase*conj(q1)*q2)'
%y(4) = 0;
%y = y/norm(y);

% Multiply the residual with the kalman gain
%updateQuat = oldQuat + K1*y;
updateQuat = (eye(4,4) - K1*H)*aPrioriQuat + K1*(H*gravityQuad);
updateQuat = (updateQuat*(1/norm(updateQuat)))';

% Setting Q1(4) to 0 so acceleromiter data does not influence the yaw angle


% update covariance
updateP = (eye(4,4)-K1*H)*pP;



        
end

% partially: https://github.com/danicomo/9dof-orientation-estimation/blob/master/KalmanFilter_V11/KalmanFilter_V14_GN.m