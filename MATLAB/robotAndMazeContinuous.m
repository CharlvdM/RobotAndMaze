%---------------------------------------------%
% BEGIN: function robotAndMazeContinuous.m    %
%---------------------------------------------%
function phaseout = robotAndMazeContinuous(input)

I                 = input.auxdata.I;
m                 = input.auxdata.m;
w                 = input.auxdata.w;

v                 = input.phase.state(:,1);
theta             = input.phase.state(:,2);
x                 = input.phase.state(:,3);
y                 = input.phase.state(:,4);
omega             = input.phase.state(:,5);
Fr                = input.phase.control(:,1);
Fl                = input.phase.control(:,2);
vDot              = (1/m)*(Fr+Fl);
thetaDot          = omega;
xDot              = v.*cos(theta);
yDot              = v.*sin(theta);
omegaDot          = (1/I)*w*(Fr-Fl);
% phaseout.dynamics = [vDot, thetaDot, xDot, yDot, omegaDot];

N = size(x,1);
d = zeros(N,1);
s = zeros(N,1);
vs = zeros(N,1);
for i = 1:N
    % [~, ~, d(i)] = centerLineDisplacement(x(i), y(i), ...
    %     input.auxdata.Maze, input.auxdata.MazeOrder, input.auxdata.Wc);
    [d(i), s(i), vs(i)] = centerLineDispNew(x(i), y(i), xDot(i), yDot(i), ...
        input.auxdata.Maze, input.auxdata.MazeOrder, input.auxdata.Wc);
end

x_prime = (1./vs).*[vDot, thetaDot, xDot, yDot, omegaDot];


phaseout.dynamics = x_prime;
phaseout.integrand = (1./vs);

if input.auxdata.pathConstraintsActive

    phaseout.path = d;
end

%---------------------------------------------%
% END: function brachistochroneContinuous.m   %
%---------------------------------------------%
