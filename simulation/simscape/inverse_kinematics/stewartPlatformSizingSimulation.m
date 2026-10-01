%% 8 Geometry Parameters: 
% These define the physical dimensions and angles of the Stewart platform.
% r_sta_pivot       : radius of the base pivot points.
% p1_theta_z        : angular positions of the base joints around the base circle.
% r_dyn_pivot       : radius of the top platform pivot points.
% p3_theta_z        : angular positions of the top joints around the top circle.
% hornLength        : length of each servo horn (arm from servo to rod connection).
% l_connecting_rod  : length of each connecting rod between servo horn and top platform.
% beta              : orientation angle of each servo in the base plane.
% z_dynamic_home    : vertical offset to set the platform in a “neutral” height where rods and horns are at right angles.

function stewartPlatformDemo
    %% ---------- Define Base Geometry ---------
    % Set the radius of the circle where the six base pivot points lie
    r_sta_pivot = 160;                                            % [mm]
    % Define the six angles (in radians) for the base pivots around the circle
    p1_theta_z = deg2rad([75, 105, 195, 225, 315, 345]);         
    % Compute the (x,y) coordinates of the six base pivots from those angles
    p1_x = r_sta_pivot .* cos(p1_theta_z);   % X-coordinates of base pivots
    p1_y = r_sta_pivot .* sin(p1_theta_z);   % Y-coordinates of base pivots
    p1_z = zeros(6,1);                       % All base pivots lie in the Z = 0 plane

    %% --------- Set Up the Plot ------------
    % Create a figure window for plotting the platform
    f = figure('Name','Stewart Platform','NumberTitle','off', ...
               'Units','normalized','Position',[.5 .1 .6 .8]);
    % Create 3D axes with specified limits and enable grid
    ax = axes('Parent',f,'Position',[0.15 0.1 0.80 0.60], ...
              'XLim',[-450 450],'YLim',[-450 450],'ZLim',[-200 450]);
    view(ax,3);  grid(ax,'on');  hold(ax,'on');
    xlabel(ax,'X [mm]'); ylabel(ax,'Y [mm]'); zlabel(ax,'Z [mm]');

    %% -------- Plot Base Pivots --------------
    % Plot the six base pivot points as black circles
    plot3(ax,p1_x,p1_y,p1_z,'ko','MarkerSize',6,'MarkerFaceColor','k');
    % Connect the six base pivots with green lines to visualize the base frame
    for i = 1:6
        j = mod(i,6) + 1;  % Next index (wrap around after 6)
        plot3(ax, [p1_x(i) p1_x(j)], [p1_y(i) p1_y(j)], [p1_z(i) p1_z(j)], ...
              'g-', 'LineWidth',1.5);
    end

    %% ------ Initial Pose and First Plot Call -----
    % pose = [roll, pitch, yaw, X, Y, Z] in degrees and mm
    pose = [0 0 0 0 0 0];    
    dyn_plot = []; leg_plots = []; dyn_edges = [];
    % Draw the initial platform (flat, centered at origin)
    [dyn_plot, leg_plots, dyn_edges] = refreshPlot(pose);

    %% ------- GUI Sliders ------------------------
    % Labels for each slider (roll, pitch, yaw in degrees; translations in mm)
    labels = {'Roll°','Pitch°','Yaw°','X mm','Y mm','Z mm'};
    % Minimum values for each parameter
    mins   = [-20 -20 -20 -50 -50 -50];
    % Maximum values for each parameter
    maxs   = [ 20  20  20  50  50  50];
    
    % Create six sliders with accompanying text labels and small edit boxes
    for k = 1:6
        % Text label to describe the slider
        uicontrol('Style','text','String',labels{k}, ...
                  'Units','normalized','Position',[0.02 0.95-0.04*k 0.08 0.04]);
        % Slider control to adjust one element of pose
        sld(k) = uicontrol('Style','slider', ...
                  'Min',mins(k),'Max',maxs(k),'Value',pose(k), ...
                  'Units','normalized','Position',[0.12 0.95-0.04*k 0.65 0.04], ...
                  'Callback',@(src,evt) sliderCallback());
        % Small text box to show/edit the numeric slider value
        edt(k) = uicontrol('Style','edit', ...
                  'String', num2str(pose(k)), ...
                  'Units','normalized','Position',[0.80 0.96-0.038*k 0.07 0.03], ...
                  'Callback', @(src,evt) editCallback(k));
    end
   
    % --- View Buttons to change camera view quickly ---
    uicontrol('Style', 'pushbutton', 'String', 'View X', ...
        'Units', 'normalized', 'Position', [0.3 0.01 0.1 0.04], ...
        'Callback', @(~,~) view(ax, [1 0 0]));  % Camera along +X axis
    
    uicontrol('Style', 'pushbutton', 'String', 'View Y', ...
        'Units', 'normalized', 'Position', [0.42 0.01 0.1 0.04], ...
        'Callback', @(~,~) view(ax, [0 1 0]));  % Camera along +Y axis
    
    uicontrol('Style', 'pushbutton', 'String', 'View Z', ...
        'Units', 'normalized', 'Position', [0.54 0.01 0.1 0.04], ...
        'Callback', @(~,~) view(ax, [0 0 1]));  % Camera along +Z axis
    
    uicontrol('Style', 'pushbutton', 'String', 'ISO', ...
        'Units', 'normalized', 'Position', [0.66 0.01 0.1 0.04], ...
        'Callback', @(~,~) view(ax, 3));        % Isometric (3D) view

    % --- Home Button: Reset pose and view to defaults ---
    uicontrol('Style', 'pushbutton', 'String', 'Home', ...
        'Units', 'normalized', 'Position', [0.78 0.01 0.1 0.04], ...
        'Callback', @(~,~) homeCallback());

    %% ------- Slider, Edit Box, and Home Button Callbacks --------------------
    function sliderCallback
        % When any slider moves, update all pose values and refresh plot
        for n = 1:6
            pose(n) = get(sld(n), 'Value');                        % Read slider value
            set(edt(n), 'String', num2str(pose(n), '%.2f'));        % Update its edit box
        end
        [dyn_plot, leg_plots, dyn_edges] = refreshPlot(pose);      % Redraw platform
    end

    function editCallback(index)
        % When an edit box changes, read the number, clamp to [min,max], update slider, refresh plot
        val = str2double(get(edt(index), 'String'));
        val = max(mins(index), min(maxs(index), val));            % Clamp within allowed range
        pose(index) = val;
        set(sld(index), 'Value', val);                             % Update slider to match
        [dyn_plot, leg_plots, dyn_edges] = refreshPlot(pose);      % Redraw platform
    end

    function homeCallback()
        % Reset pose to all zeros, reset sliders and edit boxes, set isometric view, refresh plot
        pose = zeros(1,6);
        for n = 1:6
            set(sld(n), 'Value', 0);
            set(edt(n), 'String', '0');
        end
        view(ax, 3);                                              % Isometric view
        [dyn_plot, leg_plots, dyn_edges] = refreshPlot(pose);     % Redraw platform flat
    end

    %% ------- Refresh Plot Function ------------------------
    % This function recalculates and redraws the top platform, servo horns, and rods
    function [dyn_h, leg_h, edge_h] = refreshPlot(p)
        persistent horn_lines rod_lines p1_labels p2_labels p3_labels
        % Compute servo angles, lengths, and joint positions for new pose
        [servoAngles, lengths, p3, p2, p1, L1, L2] = stewartServoAngles( ...
                                                     p(1), p(2), p(3), p(4:6));
        % Print out the base (p1), intermediate (p2), and top platform (p3) coordinates
        fprintf('p1 matrix:\n'); disp(p1);
        fprintf('p2 matrix:\n'); disp(p2);
        fprintf('p3 matrix:\n'); disp(p3);
        % Print current pose to console
        fprintf('Pose →  Roll %+5.1f°   Pitch %+5.1f°   Yaw %+5.1f°   X %+6.2f mm   Y %+6.2f mm   Z %+6.2f mm\n', ...
                p(1), p(2), p(3), p(4), p(5), p(6));
        % Compute edge lengths of the top hexagon (six edges)
        edgeLen = vecnorm(p3 - p3([2:6 1],:), 2, 2);
        armLength = L1;    % length of each servo horn
        legLength = L2;    % length of each connecting rod
        % Print various lengths and servo angles
        fprintf('Arm Lengths [mm]:  %.1f %.1f %.1f %.1f %.1f %.1f\n', L1);
        fprintf('Leg Lengths [mm]:  %.1f %.1f %.1f %.1f %.1f %.1f\n', L2);
        fprintf('Top-plate edges [mm]: %.1f %.1f %.1f %.1f %.1f %.1f\n', edgeLen);
        fprintf('Servo angles [deg]:  %.1f %.1f %.1f %.1f %.1f %.1f\n', servoAngles);
        fprintf('Lengths [mm]:        %.1f %.1f %.1f %.1f %.1f %.1f\n', lengths);

        % If horn and rod lines already exist from previous plot, delete them first
        if ~isempty(horn_lines)
            for i = 1:numel(horn_lines)
                if isvalid(horn_lines{i}), delete(horn_lines{i}); end
            end
            horn_lines = {};
        end
        if ~isempty(rod_lines)
            for i = 1:numel(rod_lines)
                if isvalid(rod_lines{i}), delete(rod_lines{i}); end
            end
            rod_lines = {};
        end

        % If top platform (dyn_plot) already exists, update its data; otherwise draw it for first time
        if ~isempty(dyn_plot) && isvalid(dyn_plot)
            set(dyn_plot, 'XData', p3(:,1), 'YData', p3(:,2), 'ZData', p3(:,3));
            for i = 1:6
                % Update each leg (base-to-top) line
                set(leg_plots(i), 'XData', [p1_x(i) p3(i,1)], ...
                                  'YData', [p1_y(i) p3(i,2)], ...
                                  'ZData', [p1_z(i) p3(i,3)]);
                j = mod(i,6) + 1;
                % Update each top-plate edge line
                set(dyn_edges(i), 'XData', [p3(i,1) p3(j,1)], ...
                                  'YData', [p3(i,2) p3(j,2)], ...
                                  'ZData', [p3(i,3) p3(j,3)]);
            end
            dyn_h = dyn_plot; leg_h = leg_plots; edge_h = dyn_edges;
        else
            % First-time drawing of top platform joints
            dyn_h = plot3(ax, p3(:,1), p3(:,2), p3(:,3), 'ko', 'MarkerSize',6,'MarkerFaceColor','k');
            % Draw initial legs (lines from base p1 to top p3)
            for i = 1:6
                leg_h(i) = plot3(ax, [p1_x(i) p3(i,1)], [p1_y(i) p3(i,2)], [p1_z(i) p3(i,3)], ...
                                'b-', 'LineWidth',1.5);
            end
            % Draw the six edges that connect top platform pivots
            for i = 1:6
                j = mod(i,6) + 1;
                edge_h(i) = plot3(ax, [p3(i,1) p3(j,1)], [p3(i,2) p3(j,2)], [p3(i,3) p3(j,3)], ...
                                  'm-', 'LineWidth',1.5);
            end
            dyn_plot = dyn_h; leg_plots = leg_h; dyn_edges = edge_h;
        end

        % Draw servo horn (red) and connecting rod (cyan) for each of the six legs
        horn_lines = cell(1,6); rod_lines = cell(1,6);
        for i = 1:6
            p1_i = [p1_x(i), p1_y(i), p1_z(i)];  % base pivot
            p2_i = p2(i,:);                      % servo horn tip
            p3_i = p3(i,:);                      % top platform pivot
            % Hull from base (p1) to servo horn tip (p2)
            horn_lines{i} = plot3(ax, [p1_i(1) p2_i(1)], [p1_i(2) p2_i(2)], [p1_i(3) p2_i(3)], ...
                                   'r-', 'LineWidth', 1.5);
            % Rod from servo horn tip (p2) to top platform (p3)
            rod_lines{i}  = plot3(ax, [p2_i(1) p3_i(1)], [p2_i(2) p3_i(2)], [p2_i(3) p3_i(3)], ...
                                   'c-', 'LineWidth', 1.5);
        end

        % Delete old joint labels if they exist
        if ~isempty(p1_labels)
            for i = 1:numel(p1_labels)
                if isvalid(p1_labels{i}), delete(p1_labels{i}); end
            end
            p1_labels = {};
        end
        if ~isempty(p2_labels)
            for i = 1:numel(p2_labels)
                if isvalid(p2_labels{i}), delete(p2_labels{i}); end
            end
            p2_labels = {};
        end
        if ~isempty(p3_labels)
            for i = 1:numel(p3_labels)
                if isvalid(p3_labels{i}), delete(p3_labels{i}); end
            end
            p3_labels = {};
        end

        % Add text labels next to each joint: p1 (base), p2 (horn tip), p3 (top)
        p1_labels = cell(1,6); p2_labels = cell(1,6); p3_labels = cell(1,6);
        for i = 1:6
            p1_labels{i} = text(p1_x(i), p1_y(i), p1_z(i) - 5, sprintf('p1_{%d}', i), ...
                                'FontSize', 8, 'Color', 'k', 'HorizontalAlignment', 'center');
            p2_labels{i} = text(p2(i,1), p2(i,2), p2(i,3), sprintf('p2_{%d}', i), ...
                                'FontSize', 8, 'Color', 'r', 'HorizontalAlignment', 'left');
            p3_labels{i} = text(p3(i,1), p3(i,2), p3(i,3), sprintf('p3_{%d}', i), ...
                                'FontSize', 8, 'Color', 'b', 'HorizontalAlignment', 'right');
        end

        % For each leg, compute and print the angle at p2 (between p1→p2 and p2→p3)
        for i = 1:6
            vec1 = p2(i,:) - p1(i,:);  % vector from base to horn tip
            vec2 = p3(i,:) - p2(i,:);  % vector from horn tip to top pivot
            angle_rad = acos(dot(vec1, vec2) / (norm(vec1) * norm(vec2)));
            angle_deg = rad2deg(angle_rad);
            fprintf('Leg %d → Angle between p1-p2 and p2-p3: %.2f°\n', i, angle_deg);
        end
        drawnow;  % Force MATLAB to update the figure immediately
    end

    %% Nested: Stewart Servo Angles -------------------
    % This function calculates servo angles and joint positions given a desired pose
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
end
