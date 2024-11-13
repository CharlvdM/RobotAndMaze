close all;

% Given parameters
Wc = 1;
r = Wc / 2;
mazeTrackLength = 4 + 8 * (pi / 4);
n = 500;
s_track = linspace(0, mazeTrackLength, n);

plotFigures = false;
% plotFigures = true;

% Initialize C array
C_track = zeros(size(s_track));

s_cell1 = Wc;
s_cell2 = Wc + (pi/4)*Wc;
s_cell4 = Wc + 3*(pi/4)*Wc;
s_cell5 = Wc + 4*(pi/4)*Wc;
s_cell6 = Wc + 5*(pi/4)* Wc;
s_cell7 = 2*Wc + 5*(pi/4)*Wc;
s_cell8 = 2*Wc + 6*(pi/4)* Wc;
s_cell9 = 3*Wc + 6*(pi/4)* Wc;
s_cell10 = 4*Wc + 6*(pi/4)*Wc;
s_cell11 = 4*Wc + 7*(pi/4)*Wc;
s_cell12 = 4*Wc + 8*(pi/4)*Wc;

% Calculate C as a function of s
for i = 1:n
    if s_track(i) < s_cell1 % Cell 1
        C_track(i) = 0;
    elseif s_track(i) < s_cell2 % Cell 2
        C_track(i) = 1 / r;
    elseif s_track(i) < s_cell4 % Cell 3 & 4
        C_track(i) = -1 / r;
    elseif s_track(i) < s_cell6 % Cell 5 & 6
        C_track(i) = 1 / r;
    elseif s_track(i) < s_cell7 % Cell 7
        C_track(i) = 0;
    elseif s_track(i) < s_cell8 % Cell 8
        C_track(i) = 1 / r;
    elseif s_track(i) < s_cell10 % Cell 9 & 10
        C_track(i) = 0;
    elseif s_track(i) < s_cell11 % Cell 11
        C_track(i) = 1 / r;
    elseif s_track(i) < s_cell12 % Cell 12
        C_track(i) = -1 / r;
    else
        C_track(i) = 0;
    end
end

n_interp = 1000;
s_interp = linspace(0, mazeTrackLength, n_interp);

C_interp = interp1(s_track,C_track, s_interp, 'pchip');
% C_interp = interp1(s_track,C_track, s_interp, 'linear');
% C_interp = interp1(s_track,C_track, s_interp, 'spline');
% C_interp = interp1(s_track,C_track, s_interp, 'makima');

if plotFigures
figure;
plot(s_track, C_track, '-o', 'LineWidth', 1.5);
hold on
plot(s_interp, C_interp, 'LineWidth', 1.5);
title('Change in Track Angle (C) vs Centre Line Displacement (s)');
xlabel('s (Centre Line Displacement)');
ylabel('C (Change in Track Angle)');
legend('Computed Points', 'Interpolation')
grid on;
end


% Function to interpolate C(s)
C_interp = @(s) interp1(s_track, C_track, s, 'pchip');

% Define the system of ODEs
odefun = @(s, y) [cos(y(3));          % dx/ds
                  sin(y(3));          % dy/ds
                  C_interp(s)];       % dtheta/ds

% Initial conditions: [x(0), y(0), theta(0)]
y0 = [0; 0.5; 0];

% Set options for higher accuracy in ode45
options = odeset('RelTol', 1e-8, 'AbsTol', 1e-10, 'MaxStep', 0.01);
s_span = linspace(0, mazeTrackLength, 1000); % Denser output points
[s_sol, y_sol] = ode45(odefun, s_span, y0, options);

% % Solve the ODE using ode45
% [s_sol, y_sol] = ode45(odefun, [0 mazeTrackLength], y0);

% Extract x, y, theta from the solution
x_sol = y_sol(:, 1);
y_sol = y_sol(:, 2);

if plotFigures
% Plot the trajectory
figure;
plot(x_sol, y_sol, 'b-', 'LineWidth', 1.5);
title('Trajectory of Robot on Maze (ode45)');
xlabel('x');
ylabel('y');
grid on;
axis equal; % To keep the aspect ratio equal
end

% New s array for integration and plotting
s = linspace(0, mazeTrackLength, 1000);

% Interpolate C for new s points
C = interp1(s_track, C_track, s, 'pchip');

% Step 1: Integrate C to get theta(s)
theta = cumtrapz(s, C);

% Step 2: Calculate dx/ds = cos(theta) and dy/ds = sin(theta)
dx_ds = cos(theta);
dy_ds = sin(theta);

% Step 3: Integrate dx/ds and dy/ds to get x(s) and y(s)
x = cumtrapz(s, dx_ds);
y = cumtrapz(s, dy_ds);

if plotFigures
% Plot x and y vs s
figure;
plot(x, y, 'b-', 'LineWidth', 1.5);
title('Trajectory of Robot on Maze (cumtrapz)');
xlabel('x');
ylabel('y');
grid on;
axis equal; % To keep the aspect ratio equal
end