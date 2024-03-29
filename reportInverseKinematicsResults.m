function reportInverseKinematicsResults(initialAngles, finalAngles, jointLimits)
for i = 1:7
    fprintf("Joint #%.0f, Initial Angle: %.4f, Final Angle: %.4f, Limit: +/- %.4f\n", ...
        i, initialAngles(i), finalAngles(i), jointLimits(i))
end
for i = 1:7
    if finalAngles(i) > jointLimits(i) || finalAngles(i) < -jointLimits(i)
        fprintf("Joint %d exceeded its limit", i)
    end
end
end

