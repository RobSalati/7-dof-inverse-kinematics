%% MEEG671 Final Project - Robert Salati
% If you'd like an animation and plots of trajectories, uncomment line 100
% and below.
function main()
%% Forward Kinematics
% Given constants:
Xca = 0.169432340063091;  % X coordinate for camera to aruco
Yca = -0.084399496991377;  % Y coordinate for camera to aruco
Zca = 0.760080832922170;  % Z coordinate for camera to aruco
q0 = [58.2686; 75.3224; 11.7968; 45.9029; -22.1081; -31.2831; -42.3712]*pi/180;  % Initial configuration
qc = [78.74; 18.46; 0.35; -65.23; 1.43; 7.11; -1.19]*pi/180;  % Configuration when picture was taken
jointTypes = ['r','r','r','r','r','r','r'];  % Joint type (all revolute)
jointLimits = [170; 120; 170; 120; 170; 120; 175]*pi/180;  % Mechanical joint limits
velocityLimits = [98 98 100 130 140 180 180]*pi/180;  % Velocity joint limits

% camera - aruco transformation: 
XYZ = [-177.1661574999052, -21.5174231910357, -12.2087112074803].*pi./180;  % Put values in array
Rca = eul2rotm(XYZ,'xyz');  % Create rotation matrix from given XYZ angles
Tca = zeros(4);  % Allocate space
Tca(1:4,1:3) = [Rca; 0 0 0];  % Rotation portion
Tca(:,4) = [Xca;Yca;Zca;1];  % Translation portion

% base - end effector transformation:
d = [0.340 0 0.400 0 0.400 0 .126]';  % DH Parameters
a = [0 0 0 0 0 0 0]';
alpha = [-pi/2 pi/2 pi/2 -pi/2 -pi/2 pi/2 0]';
dh = [d qc a alpha]; 
A = dhmat(dh);  % Create transformation matrices
T0e = A{1}*A{2}*A{3}*A{4}*A{5}*A{6}*A{7};  % Chain for T0e

% end effector - camera transformation
Tec = zeros(4);  % Allocate for T from end effector to camera
Tec1 = zeros(4);  % Allocate for transformation to calculate translations
Rec = eul2rotm([-68 0 -90]*pi/180, 'zyx');  % Rotation matrix 
Tec(1:4,1:3) = [Rec; 0 0 0];
Tec(:,4) = [0;0;0;1];  % Translation
Tec1(1:4,1:3) = [eye(3); 0 0 0];
Pec = [2.1; -1.72; 1.55]*0.0254;  % Calculate translations
Tec1(:,4) = [Pec; 1];
Tec = Tec*Tec1;  % Calculate Tec

% end effector - object transformation
Teobj = zeros(4);  % Allocate for T from end effector to object
Teobj1 = zeros(4);  % Allocate for transformation to calculate translations
Reobj = rotmat(22*pi/180,'z');  % Rotation matrix
Teobj(1:4,1:3) = [Reobj; 0 0 0];  
Teobj(:,4) = [0;0;0;1];
Teobj1(1:4,1:3) = [eye(3);0 0 0];  % Calculate translations
Peobj = [0.149; 0.1855; 1.403]*0.0254;
Teobj1(:,4) = [Peobj; 1]; 
Teobj = Teobj*Teobj1;  % Calculate Teobj

% A-T transformation
Tat = [
    0 -1 0 2.3605*0.0254; 
    -1 0 0 -4.124*0.0254; 
    0 0 -1 0; 
    0 0 0 1];  % Calculated by hand

% Final Transformation Matrix
T0t = T0e*Tec*Tca*Tat;  % Solve for base to target
T0ef = T0t*Teobj^-1;  % Solve for desired base to end effector

%% Inverse Kinematics
ZYZ = tform2eul(T0ef,'ZYZ');
xDesired = [T0ef(1:3,4);ZYZ'];
qf = inverseKinematics( ...
    [d qc a alpha], xDesired, ...
    jointTypes, jointLimits);

trajectoryTime = 6;
[angles, velocities, accelerations] = designTrajectory(q0, qf, trajectoryTime);

%% Results
reportInverseKinematicsResults(q0, qf, jointLimits);
animateRobotMotion(dh, angles, 5, T0t)
plotRobotJointAngles(angles, velocities, accelerations, ...
    jointLimits, velocityLimits, 0:0.005:trajectoryTime);
end