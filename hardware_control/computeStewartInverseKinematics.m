  function [pos_P1_O_mm,pos_P2_O_mm,pos_P3_O_mm,alpha_R_leg_rad,e1_O] = ...
             computeStewartInverseKinematics(pose)
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