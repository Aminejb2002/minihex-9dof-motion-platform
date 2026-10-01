 function [servoAngles, legLengths, p3, p2, p1, L1, L2] = stewartServoAngles( ...
                                            roll_deg, pitch_deg, yaw_deg, trans)
        % Physical constants for the mechanism
        l_connecting_rod = 308;          % Rod length [mm]
        r_sta_pivot = 160;               % Base pivot radius [mm]
        r_dyn_pivot = 60;                % Top pivot radius [mm]
        hornLength = 54;                 % Servo horn length [mm]
        % beta: orientation angles of each servo relative to base coordinate frame
        beta = [-pi/3, pi+pi/3, 2*pi/3-pi/3, -pi/3+pi/3, pi, pi/3+pi/3];

        % Define angles (in radians) for base pivots (same as above)
        p1_theta_z = deg2rad([75, 105, 195, 225, 315, 345]);
        % Define angles (in radians) for top platform pivots
        p3_theta_z = deg2rad([75, 105, 195, 225, 315, 345]);

        % Compute base pivot coordinates (p1) in 3D
        p1_x = r_sta_pivot * cos(p1_theta_z); 
        p1_y = r_sta_pivot * sin(p1_theta_z);
        p1_z = zeros(1,6);
        p1 = [p1_x', p1_y', p1_z'];  % 6×3 matrix of base pivot points

        % Compute top platform pivot coordinates in local (unrotated) frame
        p3_local = [r_dyn_pivot * cos(p3_theta_z); 
                    r_dyn_pivot * sin(p3_theta_z); 
                    zeros(1,6)];
        
        % Compute a neutral “home” height so rods and horns are at ~90° initially
        i = 3;  % choose one leg (arbitrary) for this calculation
        dx = p3_local(1,i) - p1(i,1);
        fprintf('dx: %.2f\n', dx);      
        dy = p3_local(2,i) - p1(i,2);
        fprintf('dy: %.2f\n', dy);
        z_dynamic_home = sqrt(l_connecting_rod^2 + hornLength^2 - dx^2 - dy^2);

        % Rotation matrices for roll (X), pitch (Y), and yaw (Z) angles
        Rx = @(a)[1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];
        Ry = @(a)[cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)];
        Rz = @(a)[cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];
        % Combined rotation matrix from roll, pitch, yaw (in degrees)
        R = Rx(deg2rad(roll_deg)) * Ry(deg2rad(pitch_deg)) * Rz(deg2rad(yaw_deg));

        % Apply rotation and translation to top platform pivots
        p3 = R * p3_local + trans(:) + [0; 0; z_dynamic_home];  
        p3 = p3';  % Convert to 6×3 matrix of (x,y,z) for top pivots

        % Preallocate arrays for servo angles and intermediate points p2
        servoAngles = zeros(1,6);
        p2 = zeros(6,3);

        for i = 1:6
            % Vector from base pivot p1(i,:) to top pivot p3(i,:)
            l = p3(i,:) - p1(i,:);
            % L is an intermediate term combining geometry (used to find servo angle)
            L = dot(l, l) - l_connecting_rod^2 + hornLength^2;
            M = 2 * hornLength * (p3(i,3) - p1(i,3));
            N = 2 * hornLength * (cos(beta(i)) * (p3(i,1) - p1(i,1)) + ...
                                  sin(beta(i)) * (p3(i,2) - p1(i,2)));
            % Compute the servo horn angle alpha (in radians)
            alpha = asin(L / sqrt(M^2 + N^2)) - atan2(N, M);
            servoAngles(i) = rad2deg(alpha);  % Convert to degrees

            % Compute the coordinates of the servo horn tip point p2(i,:)
            p2(i,:) = [hornLength * cos(alpha) * cos(beta(i)) + p1(i,1), ...
                       hornLength * cos(alpha) * sin(beta(i)) + p1(i,2), ...
                       hornLength * sin(alpha) + p1(i,3)];
            disp(servoAngles);  % Print servo angles for debugging

            % Compute actual lengths of horn (L1) and rod (L2) for checks
            L1 = zeros(1, 6);  % or 6x1 if you're using column format
            L2 = zeros(1, 6);

            L1(i) = norm(p2(i,:) - p1(i,:));          % Should equal hornLength
            L2(i) = norm(p3(i,:) - p2(i,:));          % Should equal l_connecting_rod

            % Error check: ensure horn and rod lengths match design
            if abs(L1(i) - hornLength) > 1e-3
                error('Horn length mismatch at leg %d: expected %.3f mm, got %.3f mm', ...
                      i, hornLength, L1(i));
            end
            if abs(L2(i) - l_connecting_rod) > 1e-3
                error('Rod length mismatch at leg %d: expected %.3f mm, got %.3f mm', ...
                      i, l_connecting_rod, L2(i));
            end
        end

        % Final output: servoAngles (1×6), combined leg lengths sqrt((p3-p1).^2), etc.
        legLengths = sqrt(sum((p3 - p1).^2, 2));
    end