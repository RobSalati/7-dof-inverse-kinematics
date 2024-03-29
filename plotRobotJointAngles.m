function plotRobotJointAngles(angles, velocities, accelerations, ...
    jointLimits, velocityLimits, time)
plot_titles = (["Joint A1","Joint A2","Joint A3","Joint A4","Joint A5","Joint A6","Joint A7"]);
for i=1:7
    figure();
    tiledlayout(3, 1, TileSpacing='Compact', Padding='Compact')
    title(plot_titles(i));

    % Plot positions
    nexttile(1)
    hold on
    plot(time,angles(i,:)*180/pi, LineWidth=2)
    plot([0,time(end)],[jointLimits(i)*180/pi, jointLimits(i)*180/pi], ...
        Color=[0.8500 0.3250 0.0980], LineWidth=2)
    plot([0,time(end)],-[jointLimits(i)*180/pi, jointLimits(i)*180/pi], ...
        Color=[0.8500 0.3250 0.0980], LineWidth=2)
    ylabel("Position [deg]")
    hold off

    % Plot Velocoties
    nexttile(2)
    hold on
    plot(time,velocities(i,:)*180/pi, LineWidth=2);
    plot([0,time(end)],[velocityLimits(i)*180/pi, velocityLimits(i)*180/pi], ...
        Color=[0.8500 0.3250 0.0980], LineWidth=2)
    plot([0,time(end)],-[velocityLimits(i)*180/pi, velocityLimits(i)*180/pi], ...
        Color=[0.8500 0.3250 0.0980], LineWidth=2)
    ylabel("Velocity [deg/s]")
    hold off

    % Plot Accelerations
    nexttile(3)
    hold on
    plot(time,accelerations(i,:)*180/pi, LineWidth=2);
    ylabel("Acceleration [deg/s^2]")
    xlabel("Time [s]")
    hold off
end
end

