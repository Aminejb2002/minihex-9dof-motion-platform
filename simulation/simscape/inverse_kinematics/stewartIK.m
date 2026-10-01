function servoAngles = stewartIK(pose)
    % pose is a 1x6 vector: [roll, pitch, yaw, X, Y, Z]
    [servoAngles, ~, ~, ~, ~, ~, ~] = stewartServoAngles( ...
        pose(1), pose(2), pose(3), pose(4:6));
        servoAngles(1) =-servoAngles(1);
        servoAngles(2) =servoAngles(2);
        servoAngles(3) =-servoAngles(3);
        servoAngles(4) =servoAngles(4);
        servoAngles(5) =-servoAngles(5);
        servoAngles(6) =servoAngles(6);
    servoAngles = reshape(servoAngles, [6, 1]);  % ensure column vector

end