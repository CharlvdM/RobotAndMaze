%---------------------------------------------------%
% Robot and Maze Problem:                           %
%---------------------------------------------------%
% The goal is for a small circular robot with two   %
% rigid wheels (iRobot Create 3) to run through a   %
% track (the solution of a maze) in miminimum time. %
%---------------------------------------------------%

clear all; close all; clc

% This first iteration simply attempts to get the robot to the specified
% final location and orientation, without any maze/track contraints.
FIRST_SIM = 1;
% Here the robot runs through the basic maze with the constraints active
BASIC_MAZE = 2;
% Now we attempt the same simulation, but where the dynamics are redefined
% so that the independent variable is "s", the center line displacement.
BASIC_MAZE_PRIME = 3;
% This maze is truncated to the bottom two rows. This is much easier to
% solve it seems
BASIC_MAZE_SIMPLIFIED = 4;
% I believe the problem with the prime dynamics is that the speed and path
% constraint isn't well defined everywhere for the maze. It can jump
% discretely between cells.
CIRCULAR_TRACK = 5;

sim = BASIC_MAZE_PRIME;

Wc = 1; % Maze cell width
Maze = ["R", "ANW", "ANE", "ANW";
        "CNW", "CSE", "CSW", "U";
        "ASE", "L", "L", "ASW"];
MazeOrder = {[0, 0],                [1, Wc],                [4, Wc*(1+3*(pi/4))],   [5, Wc*(1+4*(pi/4))];
            [11, Wc*(4+7*(pi/4))],  [2, Wc*(1+pi/4)],       [3, Wc*(1+2*(pi/4))],   [6, Wc*(1+5*(pi/4))];
            [10, Wc*(4+6*(pi/4))],  [9, Wc*(3+6*(pi/4))],   [8, Wc*(2+6*(pi/4))],   [7, Wc*(2+5*(pi/4))]};

m = 5.925;              % Robot mass
rRobot = 0.336/2;       % Robot diameter
I = 0.5*m*(rRobot^2);   % Robot z axis inertia
w = 0.13;               % Distance from the wheels to robot CoG

motorMaxTorue = 0.6; % Equates to a max force of 20 N per motor
wheelRadius = 0.03;
MaxForce = motorMaxTorue / wheelRadius;
Frmin = -MaxForce; Frmax = MaxForce; % Right wheel limits
Flmin = -MaxForce; Flmax = MaxForce; % Left wheel limits
F_DotMax = 500;
FrDotMax = F_DotMax;
FlDotMax = F_DotMax;

t0 = 0;                                             % initial time
v0 = 0.5; theta0 = 0; x0 = 0.5; y0 = 0.5; omega0 = 0; % initial state
vmin = 0.5; vmax = 16.5;
thetamin = -2*pi; thetamax = 2*pi;
thetafmin = thetamin; thetafmax = thetamax;
xmin = 0; xmax = 4;
ymin = 0; ymax = 3;
omegamin = -20; omegamax = 20; % Angular rate (rad/s) limit
switch sim
    case FIRST_SIM
        pathConstraintsActive = false;
        primeDynamicsUsed = false;
        tfmin = 0; tfmax = 5;           % time boundary
        thetatf = 0; xf = 20; yf = 10;  % final state
        thetafmin = thetatf; thetafmax = thetatf;
        xmin = 0; xmax = 25;
        ymin = 0; ymax = 20;
    case BASIC_MAZE
        pathConstraintsActive = true;
        primeDynamicsUsed = false;
        tfmin = 0; tfmax = 3;   % time boundary
        % xf = 2.5; yf = 1.3;   % final state
        % More complex final states
        xf = 3.5; yf = 1.5;     % can be solved
        % xf = 3.5; yf = 2.5;   % can't be solved currently

        % Trying to start and end in different locations.
        % thetamin = 0; thetamax = 3*pi/4;
        % thetafmin = thetamin; thetafmax = thetamax;
        % v0 = 0; theta0 = pi/2; x0 = 3.5; y0 = 0.5; omega0 = 0;
        % % xf = 3.5; yf = 2.5;     % can be solved
        % xf = 1.5; yf = 2.5;     % can't be solved
    case BASIC_MAZE_PRIME
        pathConstraintsActive = true;
        primeDynamicsUsed = true;
        tfmin = 0; tfmax = 3;                  % time boundary
        xf = 2.5; yf = 1.3;                    % final state

        n0 = 0;
        nmin = -(0.5*Wc - rRobot);
        nmax = (0.5*Wc - rRobot);

        xi0 = 0;
        ximin = -1.1*pi/2;
        ximax = 1.1*pi/2;

        % Initial and final center line displacement
        s0 = 0.5*Wc;
        sfmin = Wc + 5 * (pi / 4) * Wc;
        sfmax = 2 * Wc + 5 * (pi / 4) * Wc;
    case BASIC_MAZE_SIMPLIFIED
        pathConstraintsActive = true;
        primeDynamicsUsed = false;
        tfmin = 0; tfmax = 3;                  % time boundary
        Maze = ["R", "ANW", "ANE", "ANW";
        "CNW", "CSE", "CSW", "U"];
        ymin = 0; ymax = 2;
        xf = 3.5; yf = 1.5;                    % final state
    case CIRCULAR_TRACK
        pathConstraintsActive = true;
        primeDynamicsUsed = true;
        % pathConstraintsActive = false;
        % primeDynamicsUsed = false;
        tfmin = 0; tfmax = 3;                  % time boundary
        Maze = "ASW";
        Wc = 10;
        rRobot = 3;
        v0 = 1; theta0 = pi/2; x0 = 4.33; y0 = 2.5; omega0 = 0;
        xf = 2.5; yf = 4.33;
        xmin = 0; xmax = 10;
        ymin = 0; ymax = 10;
        % Initial and final center line displacement
        s0 = 0;
        sfmin = 2;
        sfmax = 3.5;
    otherwise
end

run CreateTrackCurvature.m
auxdata.C_track = C_track;
auxdata.s_track = s_track;

auxdata.sim = sim;
auxdata.FIRST_SIM = FIRST_SIM;
auxdata.CIRCULAR_TRACK = CIRCULAR_TRACK;
auxdata.pathConstraintsActive = pathConstraintsActive;
auxdata.primeDynamicsUsed = primeDynamicsUsed;
auxdata.Maze = convertMazeCellType(Maze);
% auxdata.Maze = Maze;
% auxdata.MazeOrder = MazeOrder;
auxdata.Wc = Wc;
auxdata.m = m;
auxdata.I = I;
auxdata.w = w;

%-------------------------------------------------------------------------%
%----------------------- Setup for Problem Bounds ------------------------%
%-------------------------------------------------------------------------%
if primeDynamicsUsed
    bounds.phase.initialtime.lower = s0; 
    bounds.phase.initialtime.upper = s0;
    bounds.phase.finaltime.lower = sfmin; 
    bounds.phase.finaltime.upper = sfmax;
    bounds.phase.initialstate.lower = [v0,theta0,x0,y0,omega0,Frmin,Flmin,n0,xi0,t0]; 
    bounds.phase.initialstate.upper = [v0,theta0,x0,y0,omega0,Frmax,Flmax,n0,xi0,t0]; 
    bounds.phase.state.lower = [vmin,thetamin,xmin,ymin,omegamin,Frmin,Flmin,nmin,ximin,t0]; 
    bounds.phase.state.upper = [vmax,thetamax,xmax,ymax,omegamax,Frmax,Flmax,nmax,ximax,tfmax]; 
    bounds.phase.finalstate.lower = [vmin,thetafmin,xf,yf,omegamin,Frmin,Flmin,nmin,ximin,tfmin]; 
    bounds.phase.finalstate.upper = [vmax,thetafmax,xf,yf,omegamax,Frmax,Flmax,nmax,ximax,tfmax]; 
    bounds.phase.control.lower = [-FrDotMax, -FlDotMax]; 
    bounds.phase.control.upper = [FrDotMax, FrDotMax];
    % The result of the integral is time
    bounds.phase.integral.lower = tfmin;
    bounds.phase.integral.upper = tfmax;
else
    bounds.phase.initialtime.lower = t0; 
    bounds.phase.initialtime.upper = t0;
    bounds.phase.finaltime.lower = tfmin; 
    bounds.phase.finaltime.upper = tfmax;
    bounds.phase.initialstate.lower = [v0,theta0,x0,y0,omega0,Frmin,Flmin]; 
    bounds.phase.initialstate.upper = [v0,theta0,x0,y0,omega0,Frmax,Flmax]; 
    bounds.phase.state.lower = [vmin,thetamin,xmin,ymin,omegamin,Frmin,Flmin]; 
    bounds.phase.state.upper = [vmax,thetamax,xmax,ymax,omegamax,Frmax,Flmax]; 
    bounds.phase.finalstate.lower = [vmin,thetafmin,xf,yf,omegamin,Frmin,Flmin]; 
    bounds.phase.finalstate.upper = [vmax,thetafmax,xf,yf,omegamax,Frmax,Flmax]; 
    bounds.phase.control.lower = [-FrDotMax, -FlDotMax]; 
    bounds.phase.control.upper = [FrDotMax, FrDotMax];
end

if pathConstraintsActive
    lowerPathBounds = -(0.5*Wc - rRobot);  % Path constraint lower bounds (>= 0)
    upperPathBounds = (0.5*Wc - rRobot);  % Path constraint upper bounds
    
    % Add these bounds to the phase
    bounds.phase.path.lower = lowerPathBounds;
    bounds.phase.path.upper = upperPathBounds;
end

%-------------------------------------------------------------------------%
%---------------------- Provide Guess of Solution ------------------------%
%-------------------------------------------------------------------------%
if primeDynamicsUsed
    guess.phase.time    = [s0; sfmax]; % The independent variable is now s (center line displacement)
    guess.phase.state   = [[v0; vmax], [theta0; theta0], [x0; xf], ...
        [y0; yf], [omega0; omega0], [Frmax; Frmax], [Flmax; Flmax], ...
        [n0; n0], [xi0; xi0], [t0; tfmax]];
    guess.phase.control = [[0; 0],[0; 0]];
    guess.phase.integral = tfmax; % guess the final time
else
    guess.phase.time    = [t0; tfmax];
    guess.phase.state   = [[v0; vmax], [theta0; theta0], [x0; xf], ...
        [y0; yf], [omega0; omega0], [Frmax; Frmax], [Flmax; Flmax]];
    guess.phase.control = [[0; 0],[0; 0]];

    % guess.phase.time    = [t0; 0.8; 1; 1.8];
    % guess.phase.state   = [...
    %     v0      theta0  x0      y0      omega0  Frmax   Flmax;
    %     5       pi/2    3.5     2       3       Frmax   -Flmax
    %     7       pi      3       2.5     6       Frmax   0
    %     vmax    pi      xf      yf      omega0  Frmax   Flmax];
    % guess.phase.control = [[10; 0; 0; 0],[10; -10; -10; 0]];
end

%-------------------------------------------------------------------------%
%----------Provide Mesh Refinement Method and Initial Mesh ---------------%
%-------------------------------------------------------------------------%
% mesh.method       = 'hp-PattersonRao';
% mesh.tolerance    = 1e-6;
% mesh.maxiterations = 45;
% mesh.colpointsmin = 4;
% mesh.colpointsmax = 10;

mesh.method       = 'hp-LiuRao-Legendre';
mesh.tolerance    = 1e-6;
mesh.colpointsmin = 4;
% mesh.colpointsmax = 10;
mesh.colpointsmax = 6;
mesh.sigma        = 0.75;

% mesh.method       = 'hp-LiuRao-Legendre';
% mesh.tolerance    = 1e-5;
% mesh.colpointsmin = 4;
% mesh.colpointsmax = 6;
% mesh.sigma        = 0.75;

mesh.maxiterations              = 10;

%-------------------------------------------------------------------------%
%------------- Assemble Information into Problem Structure ---------------%        
%-------------------------------------------------------------------------%
setup.name                        = 'RobotAndMaze-Problem';
setup.functions.continuous        = @robotAndMazeContinuous;
setup.functions.endpoint          = @robotAndMazeEndpoint;
setup.auxdata                     = auxdata;
setup.bounds                      = bounds;
setup.guess                       = guess;
setup.mesh                        = mesh; 
setup.nlp.solver                  = 'ipopt';
% setup.derivatives.supplier        = 'sparseCD';
setup.derivatives.supplier           = 'adigator';
setup.derivatives.derivativelevel = 'second';
setup.method                      = 'RPM-Differentiation';
setup.scales.method                  = 'automatic-bounds';

% setup.name                           = 'RobotAndMaze-Problem';
% setup.functions.continuous           = @robotAndMazeContinuous;
% setup.functions.endpoint             = @robotAndMazeEndpoint;
% setup.displaylevel                   = 2;
% setup.nlp.solver                     = 'ipopt';
% setup.nlp.ipoptoptions.linear_solver = 'ma57';
% setup.bounds                         = bounds;
% setup.guess                          = guess;
% setup.mesh                           = mesh;
% setup.auxdata                        = auxdata;
% setup.derivatives.supplier           = 'sparseCD';
% % setup.derivatives.supplier           = 'adigator';
% setup.derivatives.derivativelevel    = 'second';
% setup.derivatives.dependencies       = 'sparseNaN';
% setup.scales.method                  = 'automatic-bounds';
% setup.method                         = 'RPM-Differentiation';

% %-------------------------------------------------------------------------%
% %----------Provide Mesh Refinement Method and Initial Mesh ---------------%
% %-------------------------------------------------------------------------%
% mesh.maxiterations              = 10;
% mesh.method                     = 'hp-LiuRao';
% mesh.tolerance                  = 1e-6;
% 
% %-------------------------------------------------------------------%
% %---------- Configure Setup Using the information provided ---------%
% %-------------------------------------------------------------------%
% setup.name                             = 'Dynamic-Soaring-Problem';
% setup.functions.continuous             = @dynamicSoaringContinuous;
% setup.functions.endpoint               = @dynamicSoaringEndpoint;
% setup.nlp.solver                       = 'ipopt';
% setup.nlp.ipoptoptions.linear_solver   = 'ma57';
% setup.displaylevel                     = 2;
% setup.auxdata                          = auxdata;
% setup.bounds                           = bounds;
% setup.guess                            = guess;
% setup.mesh                             = mesh;
% setup.derivatives.supplier             = 'adigator';
% setup.derivatives.derivativelevel      = 'second';
% setup.scales.method                    = 'automatic-bounds';
% setup.method                           = 'RPM-Differentiation';

%-------------------------------------------------------------------------%
%------------------------- Solve Problem Using GPOP2 ---------------------%
%-------------------------------------------------------------------------%
tic
output   = gpops2(setup);
solution = output.result.solution;
elapsedTime = toc;  % Stop timer and get elapsed time
fprintf('Elapsed time: %.4f seconds\n', elapsedTime);  % Print elapsed time
