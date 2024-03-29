function animateRobotMotion(dh, angles, speed, targetOrientation)
figure()
for i=1:size(angles, 2)/speed
    dh(:, 2) = [
        angles(1,i*speed);
        angles(2,i*speed);
        angles(3,i*speed);
        angles(4,i*speed);
        angles(5,i*speed);
        angles(6,i*speed);
        angles(7,i*speed)];
    Transforms = dhmat(dh);  % Transformation Matrices
    T1 = Transforms{1};
    P1 = T1(1:3,4);
    T2 = T1*Transforms{2};
    P2 = T2(1:3,4);
    T3 = T2*Transforms{3};
    P3 = T3(1:3,4);
    T4 = T3*Transforms{4};
    P4 = T4(1:3,4);
    T5 = T4*Transforms{5};
    P5 = T5(1:3,4);
    T6 = T5*Transforms{6};
    P6 = T6(1:3,4);
    T7 = T6*Transforms{7};
    P7 = T7(1:3,4);
    X = [0 P1(1) P2(1) P3(1) P4(1) P5(1) P6(1), P7(1)];  % X Y Z joint positions
    Y = [0 P1(2) P2(2) P3(2) P4(2) P5(2) P6(2), P7(2)];
    Z = [0 P1(3) P2(3) P3(3) P4(3) P5(3) P6(3), P7(3)];
    Pt = targetOrientation(1:3,4);  % Target position
    axis([-0.8 0.8 -0.8 0.8 -0.8 0.8]);
    view(135+i/3,20)  % Rotate for perspective (and aesthetic) purposes
    xlabel("X Position");
    ylabel("Y Position");
    zlabel("Z Position");
    plot3(X,Y,Z, 'LineWidth', 4);
    grid()
    hold on
    scatter3(Pt(1), Pt(2), Pt(3))
    patch([.2 -.2 -.2 .2], [.2 .2 -.2 -.2], [0 0 0 0])  % Approximation for the table
    pause(0.01);
    cla
end

% Display final configuration
axis([-0.8 0.8 -0.8 0.8 -0.8 0.8]);
grid()
plot3(X,Y,Z, 'LineWidth', 4);
scatter3(Pt(1), Pt(2), Pt(3));
patch([.2 -.2 -.2 .2], [.2 .2 -.2 -.2], [0 0 0 0])
hold off
end

