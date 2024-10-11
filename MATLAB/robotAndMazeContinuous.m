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
Fr                = input.phase.state(:,6);
Fl                = input.phase.state(:,7);
FrDot             = input.phase.control(:,1);
FlDot             = input.phase.control(:,2);
vDot              = (1/m)*(Fr+Fl);
thetaDot          = omega;
xDot              = v.*cos(theta);
yDot              = v.*sin(theta);
omegaDot          = (1/I)*w*(Fr-Fl);

sim = input.auxdata.sim;
FIRST_SIM = input.auxdata.FIRST_SIM;
CIRCULAR_TRACK = input.auxdata.CIRCULAR_TRACK;
Wc = input.auxdata.Wc;

if (sim ~= FIRST_SIM) && (sim ~= CIRCULAR_TRACK)
    N = size(x,1);
    n = zeros(N,1);
    vs = zeros(N,1);
    for i = 1:N
        [n(i), vs(i)] = centerLineDispAndSpeed(x(i), y(i), xDot(i), yDot(i), ...
            input.auxdata.Maze, input.auxdata.Wc);
    end
elseif (sim == CIRCULAR_TRACK)
    re = sqrt(x.^2 + y.^2);
    n = re - 0.5*Wc;
    vs = (0.5*Wc).*(x.*yDot - y.*xDot)./(x.^2+y.^2);
end

if input.auxdata.primeDynamicsUsed
    x_prime = (1./vs).*[vDot, thetaDot, xDot, yDot, omegaDot, FrDot, FlDot];
    t_prime = (1./vs);
    
    phaseout.dynamics = [x_prime, t_prime];
    phaseout.integrand = (1./vs);
else
    phaseout.dynamics = [vDot, thetaDot, xDot, yDot, omegaDot, FrDot, FlDot];
end

if input.auxdata.pathConstraintsActive
    phaseout.path = n;
end

%---------------------------------------------%
% END: function brachistochroneContinuous.m   %
%---------------------------------------------%
