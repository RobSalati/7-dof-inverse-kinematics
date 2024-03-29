function A = dhmat(DH)
    d = DH(:,1);
    q = DH(:,2);
    a = DH(:,3);
    alpha = DH(:,4);
    size = length(d);
    A = cell(length(d),1);
    for i = 1:size
        A{i} = [cos(q(i)),     -sin(q(i))*cos(alpha(i)), sin(q(i))*sin(alpha(i)),  a(i)*cos(q(i))
                sin(q(i)),     cos(q(i))*cos(alpha(i)),  -cos(q(i))*sin(alpha(i)), a(i)*sin(q(i))
                0                  sin(alpha(i)),                cos(alpha(i)),                d(i)
                0                  0                             0                             1];
    end
end  % function