function q = inverseKinematics(DH, xd, joint_types, joint_limits)
d = DH(:,1);  % Extract DH parameters
q = DH(:,2);
a = DH(:,3);
alpha = DH(:,4);

K = diag(ones(1,6)*0.6);  % Standard gain matrix
K0 = diag(ones(7,1)*2.1);  % Gain matrix for joint limit avoidance

w_old = 0;  % Previous value of optimization function (used for derivative)
q_dot = ones(7,1)*1;  % Initial q_dot
% Algorithm loop:
q_mod = zeros(7,1);  % Array to store q values after modulo
for i=1:10000
    [Jg, Transforms] = geometricJacobian([d q a alpha], joint_types);  % Geometric Jacobian and transformation matrices
    T0e = Transforms{7};  % End effector transformation
    ZYZ = tform2eul(T0e,'ZYZ');  % Euler XYZ angles from T07
    phi = ZYZ(1);  % Phi Theta angles for calculating T matrix
    theta = ZYZ(2);
    xe = [T0e(1:3,4); ZYZ'];  % calculated pose
    e = xd-xe;  % Error
    if max(abs(e)) < 0.0001  % check error and break if the max error is less than 0.0001
        break;
    end
    T = [0 -sin(phi) cos(phi)*sin(theta); 0 cos(phi) sin(phi)*sin(theta); 1 0 cos(theta)];  % T matrix from ZYZ angles
    Ta = [eye(3), zeros(3); zeros(3) T];  % Ta matrix to find analytical jacobian
    Ja = Ta^-1*Jg;  % Analytical jacobian
    Ja_inv = pinv(Ja);  % Pseudo inverse of analytical jacobian

    for k=1:7  % Modulo angles with 2pi to stay within a range of -2pi to 2pi
        if q(k)<0
            q_mod(k) = mod(q(k),-2*pi);
        else
            q_mod(k) = mod(q(k),2*pi);
        end
    end
    % Joint limit avoidance
    w_new = -1/14*sum((q_mod./(joint_limits*2)).^2);  % Optimization function
    q_dot0 = K0*((w_new-w_old)./q_dot);  % Velocities from limit avoidance

    q_dot = Ja_inv*K*e + (eye(7)-Ja_inv*Ja)*q_dot0;  % joint angular velocity
    q = q + q_dot;  % solve joint angles
    w_old = w_new;  % Set old optimization value to current value
end
% Mod final angles with 2pi
for k=1:7
    if q(k)<0
        q(k) = mod(q(k),-2*pi);
    else
        q(k) = mod(q(k),2*pi);
    end
end
q(7) = q(7)+2*pi;
end