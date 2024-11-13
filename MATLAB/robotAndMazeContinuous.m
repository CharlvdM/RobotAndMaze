%---------------------------------------------%
% BEGIN: function robotAndMazeContinuous.m    %
%---------------------------------------------%
function phaseout = robotAndMazeContinuous(input)

I                 = input.auxdata.I;
m                 = input.auxdata.m;
w                 = input.auxdata.w;
C_track           = input.auxdata.C_track;
s_track           = input.auxdata.s_track;

s                 = input.phase.time;

u                 = input.phase.state(:,1);
psi               = input.phase.state(:,2);
x                 = input.phase.state(:,3);
y                 = input.phase.state(:,4);
omega             = input.phase.state(:,5);

Fr             = input.phase.control(:,1);
Fl             = input.phase.control(:,2);

uDot              = (1/m)*(Fr+Fl);
psiDot            = omega;
xDot              = u.*cos(psi);
yDot              = u.*sin(psi);
omegaDot          = (1/I)*w*(Fr-Fl);

n                 = input.phase.state(:,6);
xi                = input.phase.state(:,7);

C = interp1(s_track, C_track, s, 'pchip');

Sf = (1 - n.*C)./(u.*cos(xi));

nDot = u.*sin(xi);

x_prime = [Sf.*uDot, Sf.*psiDot, Sf.*xDot, Sf.*yDot, Sf.*omegaDot];
n_prime = Sf.*nDot;
xi_prime = Sf.*omega - C;
t_prime = Sf;

phaseout.dynamics = [x_prime, n_prime, xi_prime, t_prime];
inputWeight = 0.01;
phaseout.integrand = Sf + inputWeight*Fr.^2 + inputWeight*Fl.^2;

phaseout.path = n;

%---------------------------------------------%
% END: function brachistochroneContinuous.m   %
%---------------------------------------------%
