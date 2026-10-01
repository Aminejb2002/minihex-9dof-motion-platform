function stewartPlatformGUI
% ---------------------------------------------------------------
%   Stewart Platform GUI
%   - Geometry + IK
%   - GUI controls (sliders, PWM, table)
% ---------------------------------------------------------------

%% =====================  GEOMETRY CONSTANTS  =====================
r1  = 160;          % base radius [mm]
r3  = 0.7 * r1;     % top radius  [mm]
La  = 60;           % arm length  [mm]
Lr  = 308;          % rod length  [mm]

Delta_chi_P1_deg      = 30;   % Δchi for base anchors P1 [deg]
Delta_chi_P3_deg      = 15;   % Δchi for top anchors P3 [deg]
Delta_chi_e1_deg      = 10;   % Δchi between P1 and e1 heading angles [deg]
gamma_P1_e1_deg   = 20;   % gamma at  point P1 for e1 [deg]

sigma     = [ 1, -1,  1, -1,  1, -1];
sigma_arm = [-1,  1, -1,  1, -1,  1];

chi_s = [0, 120, 120, 240, 240, 0];   % base sector angles [deg]

%% ==================== INITIAL POSE (degrees/mm) ======================
% pose = [Phi_deg Theta_deg Psi_deg  x_mm y_mm z_mm]
pose = [0 0 0 0 0 0];

%% ========================    GUI FIGURE    ========================
f = figure('Name','Stewart Platform GUI',...
           'NumberTitle','off',...
           'Units','normalized',...
           'Position',[0.15 0.05 0.7 0.85]);

% axes a bit more to the right to avoid overlap with sliders
ax = axes('Parent',f,'Position',[0.35 0.10 0.70 0.50]);
grid(ax,'on'); hold(ax,'on'); axis(ax,'equal');
xlabel(ax,'X (mm)'); ylabel(ax,'Y (mm)'); zlabel(ax,'Z (mm)');

% NED-frame like plots
ax.XDir = 'reverse';
ax.YDir = 'normal';
ax.ZDir = 'reverse';
xlim(ax,[-350 350]); ylim(ax,[-350 350]); zlim(ax,[-350 350]);
view(ax,90,90);

%% ========================  SLIDERS  ============================
labels = {'Roll°','Pitch°','Yaw°','X mm','Y mm','Z mm'};
mins   = [-30 -30 -30 -50 -50 -50];
maxs   = [ 30  30  30  50  50  50];

for k = 1:6
    uicontrol('Style','text','String',labels{k},...
        'Units','normalized','Position',[0.02 0.95-0.05*k 0.10 0.04],...
        'HorizontalAlignment','left');

    % slightly shorter sliders so they don't reach into the axes
    sld(k) = uicontrol('Style','slider',...
        'Min',mins(k),'Max',maxs(k),'Value',pose(k),...
        'Units','normalized','Position',[0.13 0.95-0.05*k 0.42 0.04],...
        'Callback',@(src,evt) sliderChanged);

    edt(k) = uicontrol('Style','edit',...
        'String',num2str(pose(k)),...
        'Units','normalized','Position',[0.57 0.96-0.051*k 0.08 0.035],...
        'Callback',@(src,evt) editChanged(k));
end

%% PWM limits
uicontrol('Style','text','String','PWM Min:',...
    'Units','normalized','Position',[0.02 0.60 0.12 0.04],...
    'HorizontalAlignment','left');
pwmMinBox = uicontrol('Style','edit','String','1000',...
    'Units','normalized','Position',[0.13 0.60 0.12 0.04]);

uicontrol('Style','text','String','PWM Max:',...
    'Units','normalized','Position',[0.02 0.55 0.12 0.04],...
    'HorizontalAlignment','left');
pwmMaxBox = uicontrol('Style','edit','String','2000',...
    'Units','normalized','Position',[0.13 0.55 0.12 0.04]);

%% Servo angle + PWM table

servoTable = uitable('Parent',f,'Units','normalized',...
    'Position',[0.02 0.30 0.26 0.22],...
    'ColumnName',{'Angle°','PWM'},...
    'RowName',{'S1','S2','S3','S4','S5','S6'},...
    'Data',zeros(6,2));

%% View buttons 

uicontrol('Style','pushbutton','String','View X',...
    'Units','normalized','Position',[0.02 0.20 0.08 0.04],...
    'Callback',@(~,~) view(ax,[1 0 0]));
uicontrol('Style','pushbutton','String','View Y',...
    'Units','normalized','Position',[0.12 0.20 0.08 0.04],...
    'Callback',@(~,~) view(ax,[0 1 0]));
uicontrol('Style','pushbutton','String','View Z',...
    'Units','normalized','Position',[0.22 0.20 0.08 0.04],...
    'Callback',@(~,~) view(ax,[0 0 1]));
uicontrol('Style','pushbutton','String','ISO',...
    'Units','normalized','Position',[0.32 0.20 0.08 0.04],...
    'Callback',@(~,~) view(ax,3));
uicontrol('Style','pushbutton','String','Home',...
    'Units','normalized','Position',[0.22 0.14 0.08 0.04],...
    'Callback',@(~,~) resetPose);

%% Graphics handles

gP1 = []; gP2 = []; gP3 = [];
gArms = {}; gRods = {};
gEdgesBase = {}; gEdgesTop = {};
gE1 = {};

%% ============ FIRST DRAW CALL =============
refreshPlot;

%% ====================== CALLBACKS =========================
    function sliderChanged
        for i = 1:6
            pose(i) = get(sld(i),'Value');
            set(edt(i),'String',num2str(pose(i)));
        end
        refreshPlot;
    end

    function editChanged(idx)
        val = str2double(get(edt(idx),'String'));
        val = max(mins(idx),min(maxs(idx),val));
        pose(idx) = val;
        set(sld(idx),'Value',val);
        refreshPlot;
    end

    function resetPose
        % Set all sliders and edit boxes to zero
        for i = 1:6
            sld(i).Value = 0;   % move slider thumb to center
        end

        % Now recompute pose from sliders and redraw
        sliderChanged();        % reads sliders → updates pose → refreshPlot

        % Restore default view
        view(ax,90,90);
    end

%% ===================== REFRESH PLOT ========================
    function refreshPlot

        %% ---------- IK + Geometry  ----------
        [pos_P1_O_mm,pos_P2_O_mm,pos_P3_O_mm,alpha_R_leg_rad,e1_O] = ...
            computeStewartInverseKinematics(pose);

        % convert to degrees for display + PWM
        alpha_deg = rad2deg(alpha_R_leg_rad);

        %% ---------- Delete old graphics ----------
        deleteIfValid(gP1); deleteIfValid(gP2); deleteIfValid(gP3);
        deleteAll(gArms); deleteAll(gRods);
        deleteAll(gEdgesBase); deleteAll(gEdgesTop);
        deleteAll(gE1);

        %% ---------- Plot P1, P2, P3 ----------
        gP1 = plot3(ax,pos_P1_O_mm(1,:),pos_P1_O_mm(2,:),pos_P1_O_mm(3,:),...
                    'ko','MarkerFaceColor','k');
        gP2 = plot3(ax,pos_P2_O_mm(1,:),pos_P2_O_mm(2,:),pos_P2_O_mm(3,:),...
                    'ro','MarkerFaceColor','r');
        gP3 = plot3(ax,pos_P3_O_mm(1,:),pos_P3_O_mm(2,:),pos_P3_O_mm(3,:),...
                    'bo','MarkerFaceColor','b');

        %% Lower plate edges
        for i = 1:6
            j = mod(i,6)+1;
            gEdgesBase{i} = plot3(ax,...
                [pos_P1_O_mm(1,i) pos_P1_O_mm(1,j)],...
                [pos_P1_O_mm(2,i) pos_P1_O_mm(2,j)],...
                [pos_P1_O_mm(3,i) pos_P1_O_mm(3,j)],...
                'k-','LineWidth',1.2);
        end

        %% Upper plate edges
        for i = 1:6
            j = mod(i,6)+1;
            gEdgesTop{i} = plot3(ax,...
                [pos_P3_O_mm(1,i) pos_P3_O_mm(1,j)],...
                [pos_P3_O_mm(2,i) pos_P3_O_mm(2,j)],...
                [pos_P3_O_mm(3,i) pos_P3_O_mm(3,j)],...
                'm-','LineWidth',1.6);
        end

        %% Arms and rods
        for i = 1:6
            gArms{i} = plot3(ax,...
                [pos_P1_O_mm(1,i) pos_P2_O_mm(1,i)],...
                [pos_P1_O_mm(2,i) pos_P2_O_mm(2,i)],...
                [pos_P1_O_mm(3,i) pos_P2_O_mm(3,i)],...
                'r-','LineWidth',2);

            gRods{i} = plot3(ax,...
                [pos_P2_O_mm(1,i) pos_P3_O_mm(1,i)],...
                [pos_P2_O_mm(2,i) pos_P3_O_mm(2,i)],...
                [pos_P2_O_mm(3,i) pos_P3_O_mm(3,i)],...
                'c-','LineWidth',2);
        end

        %% --- Leg number labels ---
        persistent legLabels
        if ~isempty(legLabels)
            for ii = 1:numel(legLabels)
                if isgraphics(legLabels{ii})
                    delete(legLabels{ii});
                end
            end
        end
        legLabels = cell(1,6);

        for i = 1:6
            mid = (pos_P2_O_mm(:,i) + pos_P3_O_mm(:,i)) / 2;  % midpoint along the rod
            legLabels{i} = text(ax, ...
                mid(1), mid(2), mid(3), ...
                sprintf('leg %d', i), ...
                'FontSize', 12, ...
                'Color', 'k', ...
                'HorizontalAlignment','center');
        end

        %% e1 vectors
        for i = 1:6
            gE1{i} = quiver3(ax, ...
                pos_P1_O_mm(1,i),pos_P1_O_mm(2,i),pos_P1_O_mm(3,i),...
                50*e1_O(1,i),    50*e1_O(2,i),    50*e1_O(3,i),...
                'LineWidth',1.8,'Color',[0 0 0]);
        end

        %% ---------- PWM mapping ----------
        pwm_min = str2double(get(pwmMinBox,'String'));
        pwm_max = str2double(get(pwmMaxBox,'String'));
        if isnan(pwm_min) || isnan(pwm_max) || pwm_min >= pwm_max
            pwm_min = 1000; pwm_max = 2000;
        end

        % assume ±90° range → linear map
        servoPWM = ((pwm_max+pwm_min)/2) + ...
                   (alpha_deg/90) * ((pwm_max-pwm_min)/2);
        servoPWM = round(max(pwm_min,min(pwm_max,servoPWM)));

        % update table (deg + PWM)
        servoTable.Data = [round(alpha_deg(:),1) servoPWM(:)];
    end

%% ====================== SUPPORT =============================
    function deleteIfValid(h)
        if ~isempty(h) && isgraphics(h), delete(h); end
    end

    function deleteAll(C)
        if isempty(C), return; end
        if iscell(C)
            for ii = 1:numel(C)
                if isgraphics(C{ii}), delete(C{ii}); end
            end
        else
            for ii = 1:numel(C)
                if isgraphics(C(ii)), delete(C(ii)); end
            end
        end
    end

%% ==================================================================
%%                 INVERSE KINEMATICS 
%% ==================================================================
    function [pos_P1_O_mm,pos_P2_O_mm,pos_P3_O_mm,alpha_R_leg_rad,e1_O] = ...
             computeStewartInverseKinematics(pose)

        % --- attitude (deg) & translation (mm) from GUI pose ---
        Phi_deg   = pose(1);   % roll
        Theta_deg = pose(2);   % pitch
        Psi_deg   = pose(3);   % yaw

        pos_OB_O_trans_mm = pose(4:6).';   % B wrt O (translation), in O, mm

        %% === (1) chi / gamma angles (deg → rad) ===
        Delta_chi_P1_deg_vec     = Delta_chi_P1_deg      .* ones(1,6);
        Delta_chi_P3_deg_vec     = Delta_chi_P3_deg      .* ones(1,6);
        gamma_P1_e1_deg_vec  = gamma_P1_e1_deg   .* ones(1,6);

        chi_P1_deg = chi_s + sigma .* Delta_chi_P1_deg_vec;
        chi_e1_deg = chi_P1_deg + sigma * Delta_chi_e1_deg;
        chi_P3_deg = chi_s + sigma .* Delta_chi_P3_deg_vec;

        chi_P1_rad      = deg2rad(chi_P1_deg);
        chi_P3_rad      = deg2rad(chi_P3_deg);
        chi_e1_rad      = deg2rad(chi_e1_deg);
        gamma_P1_e1_rad  = deg2rad(gamma_P1_e1_deg_vec);

        %% === (2) P1: base anchor points, in O, mm ===
        pos_P1_O_mm = zeros(3,6);
        pos_P1_O_mm(1,:) = r1 * cos(chi_P1_rad);
        pos_P1_O_mm(2,:) = r1 * sin(chi_P1_rad);
        pos_P1_O_mm(3,:) = 0;

        %% === (3) e1 vectors in O ===
        e1_O = [cos(chi_e1_rad).*cos(gamma_P1_e1_rad);
                sin(chi_e1_rad).*cos(gamma_P1_e1_rad);
               -sin(gamma_P1_e1_rad)];

        %% === (4) P2_0 using zero alpha, in O, mm ===
        pos_P2_O_alpha0_mm = zeros(3,6);

        Rz = @(a)[cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];
        Rx = @(a)[1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];
        Ry = @(a)[cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)];

        for i = 1:6
            M_OBr = Rz(chi_e1_rad(i) + sigma(i)*(pi/2)) * ...
                    Rx(gamma_P1_e1_rad(i)) * ...
                    Ry(0);
            pos_P2_O_alpha0_mm(:,i) = pos_P1_O_mm(:,i) + M_OBr * [La;0;0];
        end

        %% === (5) P3 in B frame (plate-local geometry), mm ===
        pos_BP3_B_mm = [r3 .* cos(chi_P3_rad);
                        r3 .* sin(chi_P3_rad);
                        zeros(1,6)];

        %% === (6) home height (defines pos_OB_O_home_mm) ===
        i0 = 2;
        dx = pos_BP3_B_mm(1,i0) - pos_P2_O_alpha0_mm(1,i0);
        dy = pos_BP3_B_mm(2,i0) - pos_P2_O_alpha0_mm(2,i0);
        h0 = sqrt(Lr^2 - dx^2 - dy^2);

        pos_OB_O_home_mm = [0;0;h0];                  % B wrt O at home
        pos_OB_O_mm      = pos_OB_O_trans_mm + pos_OB_O_home_mm;

        %% === (7) rotation of platform (deg → rad) ===
        Phi_rad   = deg2rad(Phi_deg);
        Theta_rad = deg2rad(Theta_deg);
        Psi_rad   = deg2rad(Psi_deg);

        R = Rx(Phi_rad) * Ry(Theta_rad) * Rz(Psi_rad);

        %% === (8) final P3 in O-frame ===
        pos_P3_O_mm = zeros(3,6);
        for i = 1:6
            pos_P3_O_mm(:,i) = R * pos_BP3_B_mm(:,i) - pos_OB_O_mm;
        end

        %% === (9) inverse kinematics (D,E,F) and P2 via Rodrigues ===
        alpha_R_leg_rad = zeros(1,6);
        pos_P2_O_mm     = zeros(3,6);

        for i = 1:6
            v0   = pos_P2_O_alpha0_mm(:,i) - pos_P1_O_mm(:,i);  % P1→P2 at alpha=0
            BP   = pos_P3_O_mm(:,i)        - pos_P1_O_mm(:,i);  % P1→P3
            e1_i = sigma_arm(i) * e1_O(:,i);

            Rm2 = cross(e1_i, v0);
            Rm1 = cross(e1_i, Rm2);

            D = norm(BP)^2 + La^2 - Lr^2 ...
                - 2*( dot(Rm1,BP) + dot(v0,BP) );
            E =  2 * dot(Rm2,BP);
            F = -2 * dot(Rm1,BP);

            alpha_R_leg_rad(i) = asin(D/sqrt(E^2+F^2)) - atan2(F,E);

            % Rodrigues rotation to get P2 at this alpha

            v_rot = v0 * cos(alpha_R_leg_rad(i)) + ...
                    cross(e1_i,v0) * sin(alpha_R_leg_rad(i)) + ...
                    e1_i * dot(e1_i,v0) * (1 - cos(alpha_R_leg_rad(i)));

            pos_P2_O_mm(:,i) = pos_P1_O_mm(:,i) + v_rot;
            
            % Checks for the leg lengths
            
            L_arm_i = norm(pos_P2_O_mm(:,i) - pos_P1_O_mm(:,i));   % Should equal La
            L_rod_i = norm(pos_P3_O_mm(:,i) - pos_P2_O_mm(:,i));   % Should equal Lr
            
            if abs(L_arm_i - La) > 1e-3
                warning('Arm length mismatch (leg %d): expected %.3f mm, got %.3f mm', ...
                         i, La, L_arm_i);
            end
            
            if abs(L_rod_i - Lr) > 1e-3
                warning('Rod length mismatch (leg %d): expected %.3f mm, got %.3f mm', ...
                         i, Lr, L_rod_i);
            end
        end
    end
end
