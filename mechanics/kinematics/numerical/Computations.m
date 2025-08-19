
% Define symbolic variables
syms t1 t2 t3 t4 t5 

% Link lengths
a1 = 150.933;
a2 = 128.914;
a3 = 167.164;
a4 = 50.056;
a5 = 75.637;

% Basic rotation axis matrices 
Ry1 = [cos(t1) 0 sin(t1);
       0 1 0;
       -sin(t1) 0 cos(t1)];

Rz2 = [cos(t2) -sin(t2) 0;
       sin(t2) cos(t2) 0;
       0 0 1];

Rz3 = [cos(t3) -sin(t3) 0;
       sin(t3) cos(t3) 0;
       0 0 1];

Rz4 = [cos(t4) -sin(t4) 0;
       sin(t4) cos(t4) 0;
       0 0 1];

Rz5 = [cos(t5) -sin(t5) 0;
       sin(t5) cos(t5) 0;
       0 0 1];

I = eye(3);

% Static rotation matrices between frames
R0_1_sm = [1 0 0;
           0 0 -1;
           0 1 0];

R1_2_sm = I;
R2_3_sm = I;

R3_4_sm = [0 0 1;
           1 0 0;
           0 1 0];

R4_5_sm = I;

% Complete rotation matrices
R0_1 = R0_1_sm*Ry1;
R1_2 = R1_2_sm*Rz2;
R2_3 = R2_3_sm*Rz3;
R3_4 = Rz4*R3_4_sm;
R4_5 = R4_5_sm*Rz5;

% Displacement vectors
D0_1 = [0; 0; a1];
D1_2 = [a2*cos(t2); a2*sin(t2); 0];
D2_3 = [a3*cos(t3); a3*sin(t3); 0];
D3_4 = [0; 0; 0];
D4_5 = [0; 0; a4+a5];

% Homogeneous transformation matrices
H0_1 = [R0_1 D0_1; 0 0 0 1];
H1_2 = [R1_2 D1_2; 0 0 0 1];
H2_3 = [R2_3 D2_3; 0 0 0 1];
H3_4 = [R3_4 D3_4; 0 0 0 1];
H4_5 = [R4_5 D4_5; 0 0 0 1];

% Forward kinematics - end effector pose
H0_5 = H0_1*H1_2*H2_3*H3_4*H4_5;

% Extract position and rotation matrix of end effector
p_end = H0_5(1:3, 4);  % End effector position
R0_5 = H0_5(1:3, 1:3)  % End effector orientation

disp('End effector position:');
disp(p_end);

% Calculate intermediate transformations for Jacobian
H0_2 = H0_1*H1_2;
H0_3 = H0_2*H2_3;
H0_4 = H0_3*H3_4;

% Extract positions of each joint
p0 = [0; 0; 0];  % Base position
p1 = H0_1(1:3, 4);
p2 = H0_2(1:3, 4);
p3 = H0_3(1:3, 4);
p4 = H0_4(1:3, 4);
p5 = p_end;  % End effector position


% Extract z-axes of each frame (rotation axes)
z0 = [0; 0; 1];  % Base z-axis
z1 = H0_1(1:3, 3);  % Z-axis of frame 1
z2 = H0_2(1:3, 3);  % Z-axis of frame 2  
z3 = H0_3(1:3, 3);  % Z-axis of frame 3
z4 = H0_4(1:3, 3);  % Z-axis of frame 4

% Build Jacobian matrix J = [Jv; Jw] where Jv is linear velocity part, Jw is angular velocity part
% For each joint i, if it's revolute: 
% Jv_i = z_{i-1} × (p_end - p_{i-1})
% Jw_i = z_{i-1}

% Linear velocity part of Jacobian
Jv1 = cross(z0, p5 - p0);  % Joint 1 contribution
Jv2 = cross(z1, p5 - p1);  % Joint 2 contribution  
Jv3 = cross(z2, p5 - p2);  % Joint 3 contribution
Jv4 = cross(z3, p5 - p3);  % Joint 4 contribution
Jv5 = cross(z4, p5 - p4);  % Joint 5 contribution

% Angular velocity part of Jacobian
Jw1 = z0;  % Joint 1 contribution
Jw2 = z1;  % Joint 2 contribution
Jw3 = z2;  % Joint 3 contribution
Jw4 = z3;  % Joint 4 contribution  
Jw5 = z4;  % Joint 5 contribution

% Combine to form complete Jacobian matrix (6x5)
J = [Jv1, Jv2, Jv3, Jv4, Jv5;
     Jw1, Jw2, Jw3, Jw4, Jw5];

disp('Jacobian Matrix J (6x5):');
disp(J);

% For inverse Jacobian, since J is 6x5 (not square), we use pseudoinverse

J_pinv_mp = pinv(J);

disp('Moore-Penrose Pseudoinverse of Jacobian (5x6):');
disp(J_pinv_mp);

% Verify pseudoinverse properties
disp('Verification - J * J_pinv_mp * J should equal J:');
verification = simplify(J * J_pinv_mp * J - J);
disp('Difference (should be close to zero):');
disp(verification);


