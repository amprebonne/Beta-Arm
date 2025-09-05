% % Link parameters
% a1 = 150.933;
% a2 = 128.914; 
% a3 = 167.164;
% a4 = 50.056;
% a5 = 75.637;
% 
% % Joint limits
% joint_limits_deg = [-90, 90;    % Joint 1
%                      0, 180;   % Joint 2  
%                      -90, 90;   % Joint 3
%                      -90, 90;   % Joint 4
%                      -90, 90];  % Joint 5
% 
% % Get current joint angles
% % In practice, these come from the arm's servos
% current_joint_angles_deg = [40, 135, 0, -90, 0];  % degrees
% current_joint_angles = deg2rad(current_joint_angles_deg);
% 
% fprintf('Current joint angles: [%.1f, %.1f, %.1f, %.1f, %.1f] degrees\n', current_joint_angles_deg);
% 
% % Forward Kinematics at current position
% function [H0_5, position, orientation] = forward_kinematics(theta_deg, a1, a2, a3, a4, a5)
%     % Convert to radians
%     theta = deg2rad(theta_deg);
%     t1 = theta(1); t2 = theta(2); t3 = theta(3); t4 = theta(4); t5 = theta(5);
% 
%     % Basic rotation matrices
%     Ry1 = [cos(t1) 0 sin(t1); 0 1 0; -sin(t1) 0 cos(t1)];
%     Rz2 = [cos(t2) -sin(t2) 0; sin(t2) cos(t2) 0; 0 0 1];
%     Rz3 = [cos(t3) -sin(t3) 0; sin(t3) cos(t3) 0; 0 0 1];
%     Rz4 = [cos(t4) -sin(t4) 0; sin(t4) cos(t4) 0; 0 0 1];
%     Rz5 = [cos(t5) -sin(t5) 0; sin(t5) cos(t5) 0; 0 0 1];
% 
%     I = eye(3);
% 
%     % Static rotation matrices
%     R0_1_sm = [1 0 0; 0 0 -1; 0 1 0];
%     R1_2_sm = I; R2_3_sm = I; R4_5_sm = I;
%     R3_4_sm = [0 0 1; 1 0 0; 0 1 0];
% 
%     % Complete rotations
%     R0_1 = R0_1_sm * Ry1;
%     R1_2 = R1_2_sm * Rz2;
%     R2_3 = R2_3_sm * Rz3;
%     R3_4 = Rz4 * R3_4_sm;
%     R4_5 = R4_5_sm * Rz5;
% 
%     % Displacement vectors
%     D0_1 = [0; 0; a1];
%     D1_2 = [a2*cos(t2); a2*sin(t2); 0];
%     D2_3 = [a3*cos(t3); a3*sin(t3); 0];
%     D3_4 = [0; 0; 0];
%     D4_5 = [0; 0; a4+a5];
% 
%     % Homogeneous transformations
%     H0_1 = [R0_1 D0_1; 0 0 0 1];
%     H1_2 = [R1_2 D1_2; 0 0 0 1];
%     H2_3 = [R2_3 D2_3; 0 0 0 1];
%     H3_4 = [R3_4 D3_4; 0 0 0 1];
%     H4_5 = [R4_5 D4_5; 0 0 0 1];
% 
%     % Forward kinematics
%     H0_5 = H0_1 * H1_2 * H2_3 * H3_4 * H4_5;
% 
%     % Extract position and orientation
%     position = H0_5(1:3, 4);
%     orientation = H0_5(1:3, 1:3);
% end
% 
% 
% %Inverse kinematics function
% function theta_solutions = inverse_kinematics_symbolic(points, phi, a1, a2, a3, a4, a5)
%     % Inputs: px, py, pz (position), phi (end-effector orientation), link lengths
%     % Outputs: theta_solutions (all valid joint angle solutions in degrees)
% 
%     phi_rad = deg2rad(phi);
% 
%     theta_solutions = [];   % Initialize solutions array
% 
%     % End-effector points downward, so wrist center is:
%     wx = points(1);
%     wy = points(2); 
%     wz = points(3) + a4 + a5;
% 
%     theta1 = atan2(wy, wx)    % θ1 (base rotation)
% 
%     r = sqrt(wx^2 + wy^2);      % Distance in x-y plane to wrist center
% 
%     s = wz - a1;            % Vertical distance from joint 2 to wrist center 
% 
%     d = sqrt(r^2 + s^2)  % Diagonal distance from joint 2 to wrist center on xz plane
% 
%     if d > (a2 + a3) || d < abs(a2 - a3)
%         fprintf('Target unreachable: d=%.2f, max_reach=%.2f, min_reach=%.2f\n', ...
%                 d, a2+a3, abs(a2-a3));
%         return;
%     end
% 
% 
%     cos_beta = (a2^2 + a3^2 - d^2) / (2*a2*a3)
% 
%     if abs(cos_beta) > 1
%         fprintf('No solution: cos(beta) = %.3f\n', cos_alpha);
%         return;
%     end
% 
%     % θ3 (elbow angle) - two solutions
%     theta3_solutions = [ acos(-cos_beta), -acos(-cos_beta)];
% 
%     for i = 1:length(theta3_solutions)
% 
%         theta3 = theta3_solutions(i)
% 
%         % Solve for θ2 (shoulder angle)
%         alpha = atan2(a3*sin(theta3),a2 + a3 * cos(theta3))     % Calculate intermediate angles
%         rho = atan2(s, r);
%         theta2 = rho - alpha;
% 
%         % θ4 for downward pointing
%         theta4 = -pi/2 - theta2 - theta3
% 
%         theta5 = phi_rad;   % θ5 (end effector rotation) direct assignment
% 
%         solution = [rad2deg(theta1), rad2deg(theta2), rad2deg(theta3), rad2deg(theta4), rad2deg(theta5)]
% 
%         limits_deg = [0, 180;
%                   0, 180;
%                   -90, 90;
%                   -90.1, 90;
%                   0, 180];
%         % Check joint limits
%         valid = true;
%         for j = 1:5
%             if solution(j) < limits_deg(j,1) || solution(j) > limits_deg(j,2)
%                 fprintf('No solution:  %.3f, %.3f, %.1f\n', limits_deg(j,1),limits_deg(j,2),j);
%                 valid = false;
%                 break;
%             end
%         end
% 
%         if valid
%              theta_solutions = [theta_solutions; solution];
%         end
%     end
% end
% 
% % Compute numerical Jacobian at current position
% function J_num = compute_jacobian_at_angles(theta_vals, a1, a2, a3, a4, a5)
%     % Extract individual angles
%     t1 = theta_vals(1); t2 = theta_vals(2); t3 = theta_vals(3); 
%     t4 = theta_vals(4); t5 = theta_vals(5);
% 
%     % Basic rotation matrices
%     Ry1 = [cos(t1) 0 sin(t1); 0 1 0; -sin(t1) 0 cos(t1)];
%     Rz2 = [cos(t2) -sin(t2) 0; sin(t2) cos(t2) 0; 0 0 1];
%     Rz3 = [cos(t3) -sin(t3) 0; sin(t3) cos(t3) 0; 0 0 1];
%     Rz4 = [cos(t4) -sin(t4) 0; sin(t4) cos(t4) 0; 0 0 1];
%     Rz5 = [cos(t5) -sin(t5) 0; sin(t5) cos(t5) 0; 0 0 1];
% 
%     I = eye(3);
% 
%     % Static rotation matrices
%     R0_1_sm = [1 0 0; 0 0 -1; 0 1 0];
%     R1_2_sm = I; R2_3_sm = I; R4_5_sm = I;
%     R3_4_sm = [0 0 1; 1 0 0; 0 1 0];
% 
%     % Complete rotations
%     R0_1 = R0_1_sm * Ry1;
%     R1_2 = R1_2_sm * Rz2;
%     R2_3 = R2_3_sm * Rz3;
%     R3_4 = Rz4 * R3_4_sm;
%     R4_5 = R4_5_sm * Rz5;
% 
%     % Displacement vectors
%     D0_1 = [0; 0; a1];
%     D1_2 = [a2*cos(t2); a2*sin(t2); 0];
%     D2_3 = [a3*cos(t3); a3*sin(t3); 0];
%     D3_4 = [0; 0; 0];
%     D4_5 = [0; 0; a4+a5];
% 
%     % Homogeneous transformations
%     H0_1 = [R0_1 D0_1; 0 0 0 1];
%     H1_2 = [R1_2 D1_2; 0 0 0 1];
%     H2_3 = [R2_3 D2_3; 0 0 0 1];
%     H3_4 = [R3_4 D3_4; 0 0 0 1];
%     H4_5 = [R4_5 D4_5; 0 0 0 1];
% 
%     % Forward kinematics
%     H0_2 = H0_1 * H1_2;
%     H0_3 = H0_2 * H2_3;
%     H0_4 = H0_3 * H3_4;
%     H0_5 = H0_4 * H4_5;
% 
%     % Extract positions and z-axes
%     p0 = [0; 0; 0];
%     p1 = H0_1(1:3, 4);
%     p2 = H0_2(1:3, 4); 
%     p3 = H0_3(1:3, 4);
%     p4 = H0_4(1:3, 4);
%     p5 = H0_5(1:3, 4);
% 
%     z0 = [0; 0; 1];
%     z1 = H0_1(1:3, 3);
%     z2 = H0_2(1:3, 3);
%     z3 = H0_3(1:3, 3);
%     z4 = H0_4(1:3, 3);
% 
%     % Build Jacobian
%     Jv1 = cross(z0, p5 - p0); Jv2 = cross(z1, p5 - p1); 
%     Jv3 = cross(z2, p5 - p2); Jv4 = cross(z3, p5 - p3);
%     Jv5 = cross(z4, p5 - p4);
% 
%     Jw1 = z0; Jw2 = z1; Jw3 = z2; Jw4 = z3; Jw5 = z4;
% 
%     J_num = [Jv1, Jv2, Jv3, Jv4, Jv5;
%              Jw1, Jw2, Jw3, Jw4, Jw5];
% end
% 
% % Compute current Jacobian
% J_current = compute_jacobian_at_angles(current_joint_angles, a1, a2, a3, a4, a5);
% 
% fprintf('\nCurrent Jacobian matrix (6x5):\n');
% disp(J_current);
% 
% % === STEP 3: Choose pseudoinverse based on application ===
% % Option 1: Moore-Penrose (general purpose)
% J_pinv_mp = pinv(J_current);
% 
% % Option 2: Left pseudoinverse (smooth joint motion priority)
% J_pinv_left = (J_current' * J_current) \ J_current';
% 
% fprintf('\nPseudoinverse matrices computed.\n');
% 
% % === STEP 4: Define desired end-effector motion ===
% % Example: Move 5 cm/s in x-direction, rotate 10 deg/s about z-axis
% desired_linear_velocity = [0.05; 0; 0];      % 5 cm/s in x (m/s)
% desired_angular_velocity = [0; 0; deg2rad(10)]; % 10 deg/s about z (rad/s)
% desired_twist = [desired_linear_velocity; desired_angular_velocity];
% 
% fprintf('\nDesired end-effector motion:\n');
% fprintf('Linear velocity: [%.3f, %.3f, %.3f] m/s\n', desired_linear_velocity);
% fprintf('Angular velocity: [%.1f, %.1f, %.1f] deg/s\n', rad2deg(desired_angular_velocity));
% 
% % === STEP 5: Calculate required joint velocities ===
% joint_vel_mp = J_pinv_mp * desired_twist;
% joint_vel_left = J_pinv_left * desired_twist;
% 
% fprintf('\nRequired joint velocities (rad/s):\n');
% fprintf('Moore-Penrose: [%.4f, %.4f, %.4f, %.4f, %.4f]\n', joint_vel_mp);
% fprintf('Left Pseudo:   [%.4f, %.4f, %.4f, %.4f, %.4f]\n', joint_vel_left);
% 
% fprintf('\nRequired joint velocities (deg/s):\n');
% fprintf('Moore-Penrose: [%.2f, %.2f, %.2f, %.2f, %.2f]\n', rad2deg(joint_vel_mp));
% fprintf('Left Pseudo:   [%.2f, %.2f, %.2f, %.2f, %.2f]\n', rad2deg(joint_vel_left));
% 
% % === STEP 6: Verify the solution ===
% achieved_twist_mp = J_current * joint_vel_mp;
% error_mp = desired_twist - achieved_twist_mp;
% 
% fprintf('\nVerification (Moore-Penrose):\n');
% fprintf('Desired twist:  [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n', desired_twist);
% fprintf('Achieved twist: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n', achieved_twist_mp);
% fprintf('Error:          [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n', error_mp);
% fprintf('Max error magnitude: %.6f\n', max(abs(error_mp)));
% 
% % === STEP 7: Practical usage ===
% fprintf('4. Update Jacobian at ~100-1000 Hz for real-time control\n');
% 
% % Test 1: Forward Kinematics
% fprintf('\n=== TEST 1: FORWARD KINEMATICS ===\n');
% [H, pos, rot] = forward_kinematics(current_joint_angles_deg, a1, a2, a3, a4, a5);
% 
% fprintf('Joint angles: [%.1f, %.1f, %.1f, %.1f, %.1f] deg\n', current_joint_angles_deg);
% fprintf('End-effector position: [%.2f, %.2f, %.2f] mm\n', pos);
% 
% % Test 2: Inverse Kinematics
% fprintf('\n=== TEST 2: INVERSE KINEMATICS ===\n');
% target_pos = pos;  % Use forward kinematics result as target
% target_phi = 0;    % Tool rotation
% 
% fprintf('Target position: [%.2f, %.2f, %.2f] mm\n', target_pos);
% fprintf('Target tool rotation: %.1f deg\n', target_phi);
% 
% solutions = inverse_kinematics_symbolic(target_pos, ...
%                                        target_phi, a1, a2, a3, a4, a5);
% 
% if ~isempty(solutions)
%     fprintf('Found %d valid solution(s):\n', size(solutions, 1));
%     for i = 1:size(solutions, 1)
%         fprintf('Solution %d: [%.1f, %.1f, %.1f, %.1f, %.1f] deg\n', ...
%                 i, solutions(i, :));
% 
%         % Verify solution
%         [~, verify_pos, ~] = forward_kinematics(solutions(i, :), a1, a2, a3, a4, a5);
%         error = norm(verify_pos - target_pos');
%         fprintf('  Verification error: %.4f mm\n', error);
%     end
% else
%     fprintf('No valid solutions found!\n');
% end




% Link parameters
a1 = 150.933;
a2 = 128.914; 
a3 = 167.164;
a4 = 50.056;
a5 = 75.637;

% Joint limits
joint_limits_deg = [-90, 90;    % Joint 1
                     -90, 90;   % Joint 2  
                     -90, 90;   % Joint 3
                     -90, 90;   % Joint 4
                     -90, 90];  % Joint 5

% Get current joint angles
current_joint_angles_deg = [0, 0, 0, -90, 0];  % degrees
current_joint_angles = deg2rad(current_joint_angles_deg);

fprintf('Current joint angles: [%.1f, %.1f, %.1f, %.1f, %.1f] degrees\n', current_joint_angles_deg);

%% D-H Parameter Function
function H = dh_transform(theta, alpha, r, d)
    
    ct = cos(theta); st = sin(theta);
    ca = cos(alpha); sa = sin(alpha);
    
    H = [ct    -st*ca   st*sa    r*ct;
         st     ct*ca  -ct*sa    r*st;
          0       sa      ca       d;
          0        0       0       1];
end

%% Custom D-H Transform for Joint 3->4 (matching manual rotation method)
function H = dh_transform_joint34(theta4, alpha, r, d)
    
    ct = cos(theta4); st = sin(theta4);
   
    % Manual method gives: [-sin(t4), 0, cos(t4); cos(t4), 0, sin(t4); 0, 1, 0]
    R_custom = [-st   0   ct;
                 ct   0   st;
                  0   1    0];
    
    p_custom = [r*ct; r*st; d];
    
    H = [R_custom  p_custom;
         0 0 0     1      ];
end

%% Core transformation matrix calculation function
function [R_matrices, D_vectors, H_matrices, H_cumulative] = compute_transforms(theta, a1, a2, a3, a4, a5)
    % Compute all transformation matrices for given joint angles (in radians)
    % Uses D-H parameters except for joint 3->4 which uses rotation matrix method
    
    t1 = theta(1); t2 = theta(2); t3 = theta(3); t4 = theta(4); t5 = theta(5);
    
    % D-H Parameters
    theta_dh = [t1, t2, t3, t4, t5];
    alpha_dh = [pi/2, 0, 0, pi/2, 0];  % [90, 0, 0, 90, 0] degrees in radians
    r_dh = [0, a2, a3, 0, 0];
    d_dh = [a1, 0, 0, 0, a4+a5];
    
    % Compute transforms using D-H parameters
    H0_1 = dh_transform(theta_dh(1), alpha_dh(1), r_dh(1), d_dh(1));
    H1_2 = dh_transform(theta_dh(2), alpha_dh(2), r_dh(2), d_dh(2));
    H2_3 = dh_transform(theta_dh(3), alpha_dh(3), r_dh(3), d_dh(3));
    H3_4 = dh_transform_joint34(theta_dh(4), alpha_dh(4), r_dh(4), d_dh(4));
    H4_5 = dh_transform(theta_dh(5), alpha_dh(5), r_dh(5), d_dh(5));
     
    
    % Extract rotation matrices and displacement vectors for compatibility
    R0_1 = H0_1(1:3, 1:3); D0_1 = H0_1(1:3, 4);
    R1_2 = H1_2(1:3, 1:3); D1_2 = H1_2(1:3, 4);
    R2_3 = H2_3(1:3, 1:3); D2_3 = H2_3(1:3, 4);
    R3_4 = H3_4(1:3, 1:3); D3_4 = H3_4(1:3, 4);
    R4_5 = H4_5(1:3, 1:3); D4_5 = H4_5(1:3, 4);
    
    % Store rotation matrices and displacement vectors
    R_matrices = {R0_1, R1_2, R2_3, R3_4, R4_5};
    D_vectors = {D0_1, D1_2, D2_3, D3_4, D4_5};
    
    % Store individual transforms
    H_matrices = {H0_1, H1_2, H2_3, H3_4, H4_5};
    
    % Cumulative transformations
    H0_1_cum = H0_1;
    H0_2_cum = H0_1_cum * H1_2;
    H0_3_cum = H0_2_cum * H2_3;
    H0_4_cum = H0_3_cum * H3_4;
    H0_5_cum = H0_4_cum * H4_5;
    
    % Store cumulative transforms
    H_cumulative = {H0_1_cum, H0_2_cum, H0_3_cum, H0_4_cum, H0_5_cum};
end

%% Forward Kinematics
function [H0_5, position, orientation] = forward_kinematics(theta_deg, a1, a2, a3, a4, a5)
    % Convert to radians and compute transforms
    theta = deg2rad(theta_deg);
    [~, ~, ~, H_cumulative] = compute_transforms(theta, a1, a2, a3, a4, a5);
    
    % Extract final transformation
    H0_5 = H_cumulative{5}
    position = H0_5(1:3, 4);
    orientation = H0_5(1:3, 1:3);
end

%% Inverse Kinematics
function theta_solutions = inverse_kinematics_symbolic(points, phi, a1, a2, a3, a4, a5)
    % Inputs: px, py, pz (position), phi (end-effector orientation), link lengths
    % Outputs: theta_solutions (all valid joint angle solutions in degrees)
    
    phi_rad = deg2rad(phi);
    theta_solutions = [];   % Initialize solutions array
    
    % End-effector points downward, so wrist center is:
    wx = points(1);
    wy = points(2); 
    wz = points(3) + a4 + a5;
    
    theta1 = atan2(wy, wx);    % θ1 (base rotation)
    
    r = sqrt(wx^2 + wy^2);      % Distance in x-y plane to wrist center
    s = wz - a1;                % Vertical distance from joint 2 to wrist center 
    d = sqrt(r^2 + s^2);        % Diagonal distance from joint 2 to wrist center on xz plane
    
    if d > (a2 + a3) || d < abs(a2 - a3)
        fprintf('Target unreachable: d=%.2f, max_reach=%.2f, min_reach=%.2f\n', ...
                d, a2+a3, abs(a2-a3));
        return;
    end
    
    cos_alpha = (d^2 + a2^2 - a3^2) / (2*a2*d);
    
    if abs(cos_alpha) > 1
        fprintf('No solution: cos(alpha) = %.3f\n', cos_alpha);
        return;
    end
    alpha = acos(cos_alpha);

    % Solve for θ2 (shoulder angle)
    rho = atan2(s, r);     % Calculate intermediate angles
    theta2 = rho - alpha;
    
    % θ3 (elbow angle) - two solutions
    theta3_solutions = [asin((d*sin(alpha))/a3), pi - asin((d*sin(alpha))/a3)];
    
    for i = 1:length(theta3_solutions)
        theta3 = theta3_solutions(i);
        
        % θ4 for downward pointing
        theta4 = -pi/2 - theta2 - theta3;
        
        theta5 = phi_rad;   % θ5 (end effector rotation) direct assignment
        
        solution = [rad2deg(theta1), rad2deg(theta2), rad2deg(theta3), rad2deg(theta4), rad2deg(theta5)];
        
        % Check joint limits
        valid = true;
        for j = 1:5
            if solution(j) < -90 || solution(j) > 90
                valid = false;
                break;
            end
        end
        
        if valid
            theta_solutions = [theta_solutions; solution];
        end
    end
end

%% Jacobian Computation
function J_num = compute_jacobian_at_angles(theta_vals, a1, a2, a3, a4, a5)
    % Compute all transformations once
    [~, ~, ~, H_cumulative] = compute_transforms(theta_vals, a1, a2, a3, a4, a5);
    
    % Extract positions and z-axes from cumulative transforms
    p0 = [0; 0; 0];
    p1 = H_cumulative{1}(1:3, 4);
    p2 = H_cumulative{2}(1:3, 4); 
    p3 = H_cumulative{3}(1:3, 4);
    p4 = H_cumulative{4}(1:3, 4);
    p5 = H_cumulative{5}(1:3, 4);
    
    z0 = [0; 0; 1];
    z1 = H_cumulative{1}(1:3, 3);
    z2 = H_cumulative{2}(1:3, 3);
    z3 = H_cumulative{3}(1:3, 3);
    z4 = H_cumulative{4}(1:3, 3);
    
    % Build Jacobian columns
    Jv1 = cross(z0, p5 - p0); Jv2 = cross(z1, p5 - p1); 
    Jv3 = cross(z2, p5 - p2); Jv4 = cross(z3, p5 - p3);
    Jv5 = cross(z4, p5 - p4);
    
    Jw1 = z0; Jw2 = z1; Jw3 = z2; Jw4 = z3; Jw5 = z4;
    
    J_num = [Jv1, Jv2, Jv3, Jv4, Jv5;
             Jw1, Jw2, Jw3, Jw4, Jw5];
end

%% Main Execution
% Compute current Jacobian
J_current = compute_jacobian_at_angles(current_joint_angles, a1, a2, a3, a4, a5);

fprintf('\nCurrent Jacobian matrix (6x5):\n');
disp(J_current);

% Compute pseudoinverses
J_pinv_mp = pinv(J_current);                              % Moore-Penrose

fprintf('\nPseudoinverse matrices computed.\n');

% Define desired end-effector motion
desired_linear_velocity = [0.05; 0; 0];      % 5 cm/s in x (m/s)
desired_angular_velocity = [0; 0; deg2rad(10)]; % 10 deg/s about z (rad/s)
desired_twist = [desired_linear_velocity; desired_angular_velocity];

fprintf('\nDesired end-effector motion:\n');
fprintf('Linear velocity: [%.3f, %.3f, %.3f] m/s\n', desired_linear_velocity);
fprintf('Angular velocity: [%.1f, %.1f, %.1f] deg/s\n', rad2deg(desired_angular_velocity));

% Calculate required joint velocities
joint_vel_mp = J_pinv_mp * desired_twist;

fprintf('\nRequired joint velocities (rad/s):\n');
fprintf('Moore-Penrose: [%.4f, %.4f, %.4f, %.4f, %.4f]\n', joint_vel_mp);

fprintf('\nRequired joint velocities (deg/s):\n');
fprintf('Moore-Penrose: [%.2f, %.2f, %.2f, %.2f, %.2f]\n', rad2deg(joint_vel_mp));

% Verification
achieved_twist_mp = J_current * joint_vel_mp;
error_mp = desired_twist - achieved_twist_mp;

fprintf('\nVerification (Moore-Penrose):\n');
fprintf('Desired twist:  [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n', desired_twist);
fprintf('Achieved twist: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n', achieved_twist_mp);
fprintf('Error:          [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n', error_mp);
fprintf('Max error magnitude: %.6f\n', max(abs(error_mp)));

%% Testing Section
% Test 1: Forward Kinematics
fprintf('\n=== TEST 1: FORWARD KINEMATICS ===\n');
[H, pos, rot] = forward_kinematics(current_joint_angles_deg, a1, a2, a3, a4, a5);

fprintf('Joint angles: [%.1f, %.1f, %.1f, %.1f, %.1f] deg\n', current_joint_angles_deg);
fprintf('End-effector position: [%.2f, %.2f, %.2f] mm\n', pos);

% Test 2: Inverse Kinematics
fprintf('\n=== TEST 2: INVERSE KINEMATICS ===\n');
target_pos = pos;  % Use forward kinematics result as target
target_phi = 0;    % Tool rotation

fprintf('Target position: [%.2f, %.2f, %.2f] mm\n', target_pos);
fprintf('Target tool rotation: %.1f deg\n', target_phi);

solutions = inverse_kinematics_symbolic(target_pos, target_phi, a1, a2, a3, a4, a5);

if ~isempty(solutions)
    fprintf('Found %d valid solution(s):\n', size(solutions, 1));
    for i = 1:size(solutions, 1)
        fprintf('Solution %d: [%.1f, %.1f, %.1f, %.1f, %.1f] deg\n', ...
                i, solutions(i, :));
        
        % Verify solution
        [~, verify_pos, ~] = forward_kinematics(solutions(i, :), a1, a2, a3, a4, a5);
        error = norm(verify_pos - target_pos');
        fprintf('  Verification error: %.4f mm\n', error);
    end
else
    fprintf('No valid solutions found!\n');
end