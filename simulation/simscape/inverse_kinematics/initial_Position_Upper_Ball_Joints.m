% Get the time vector and data
t = Ball_joint_init_pos6.Time;      % Time array
v = Ball_joint_init_pos6.Data;      % Data array

% Find the index where time is zero
idx = find(t == 0, 1);              % This finds t=0

% Extract the first 3 values at t=0
first3 = v(idx, 1:3);

% Display the result
disp(first3)