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
phaseout.dynamics = [vDot, thetaDot, xDot, yDot, omegaDot];

%---------------------------------------------%
% END: function brachistochroneContinuous.m   %
%---------------------------------------------%
