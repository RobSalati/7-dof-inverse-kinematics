function [J,T] = geometricJacobian(DH, joints)
A = dhmat(DH);
size = length(A);  % Number of joints
J = zeros(6,size);
z = cell(size,1);  % Z cell array
T = cell(size,1);  % Homogenous transform cell array
p = cell(size+1,1);  % Homogenous position cell array

T{1} = A{1};  % T01
z{1} = [0;0;1];  % z0
p{1} = T{1}(1:3,4);  % p0

for i=2:size
    T{i} = T{i-1}*A{i};  % Creates T02, T03, ..., T0N
    z{i} = T{i-1}(1:3,3);  % Creates z1, z2, ..., zN
    p{i} = T{i-1}(1:3,4);  % Creates p1, p2, ..., pN
end
p{size+1} = T{size}(1:3,4);  % Creates pe

for i = 1:size  % Writes Jacobian
    if joints(i) == 'r' || joints(i) == 'R'
        J(1:3,i) = cross(z{i},p{size+1}-p{i});  % Translations
        J(4:6,i) = z{i};  % Rotations
    end
    if joints(i) == 'p' || joints(i) == 'P'
        J(1:3,i) = z{i};
        J(4:6,i) = [0;0;0];

    end
end
