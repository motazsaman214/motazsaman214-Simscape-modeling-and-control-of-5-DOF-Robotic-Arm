
%  Motaz Ahmed Mohamed Kamel 120210316
%  simulate the multi-body model
%  lab task
% === Define a 5-DOF robot using Denavit-Hartenberg param   eters ===
% Each joint is defined as a revolute joint with specific D-H parameters
j1 = Link('revolute', 'd', 0.06932, 'a', 0.01698, 'alpha', -pi/2);
j2 = Link('revolute', 'd', 0,       'a', 0.09,    'alpha', 0); 
j3 = Link('revolute', 'd', 0,       'a', 0,       'alpha', -pi/2); 
j4 = Link('revolute', 'd', 0.0904,  'a', 0,       'alpha', pi/2);    
j5 = Link('revolute', 'd', 0,       'a', 0.0165,  'alpha', pi/2);      

% Create a SerialLink robot model using the defined joints
robot5 = SerialLink([j1 j2 j3 j4 j5], 'name', '5DOF');

% === Generate a circular trajectory in the YZ plane ===
% Circle centered at Z=0.1 with fixed X = 0.1
radius = 0.05;                         % Radius of the circle
angles = linspace(0, 2*pi, 100);       % 100 points around the circle
X_fixed = 0.1;                         % Constant X position

% Compute circular path coordinates in YZ plane
Y = radius * cos(angles);             
Z = 0.1 + radius * sin(angles);       
X = X_fixed * ones(size(angles));     % Fixed X for all trajectory points

% === Construct 4x4 homogeneous transformation matrices for the path ===
T_path = zeros(4, 4, length(angles)); 
for i = 1:length(angles)
    T_path(:, :, i) = transl([X(i), Y(i), Z(i)]); % Translation-only transform
end

% === Solve inverse kinematics for each trajectory point ===
% Preallocate joint trajectory and initialize the guess
q_traj = zeros(length(angles), 5);    
q_guess = zeros(1,5);                 % Initial guess for IK solver

for i = 1:length(angles)
    try
        % Solve IK with position-only constraint (mask disables rotation)
        q = robot5.ikine(T_path(:, :, i), q_guess, 'mask', [1 1 1 0 0 0]); 
        q_traj(i, :) = q;             % Store result
        q_guess = q;                  % Use current result as next guess
    catch
        % If IK fails, reuse last valid solution and print warning
        fprintf('Warning: IK failed at step %d\n', i);
        if i > 1
            q_traj(i, :) = q_traj(i-1, :); 
            q_guess = q_traj(i, :);
        end
    end
end

% === Create time vector and export joint trajectories as timeseries ===
t_sim = linspace(0, 10, length(angles)); % Time vector for simulation
for j = 1:5
    ts = timeseries(q_traj(:, j), t_sim);           % Create timeseries
    assignin('base', sprintf('J%d', j), ts); % Export to base workspace
end

% === Animate the robot following the computed joint trajectory ===
figure;
robot5.plot(q_traj, 'trail', {'b', 'LineWidth', 2}); % Animate with green trail
title('Circular Path in YZ Plane');
