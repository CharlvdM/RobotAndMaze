%---------------------------------------------------%
% Robot and Maze Problem:                           %
%---------------------------------------------------%
% The goal is for a small circular robot with two   %
% rigid wheels (iRobot Create 3) to run through a   %
% track (the solution of a maze) in miminimum time. %
%---------------------------------------------------%

clear all; close all; clc

FIRST_SIM = 1;
BASIC_MAZE = 2;

sim = FIRST_SIM;

auxdata.pathConstraintsActive = true;

% Maze x-collision matrix
xColMatrix =   [1   0   1   0
                1   1   0   1
                1   0   0   0];
yColMatrix =   [1   1   1   1
                1   0   0   0
                0   1   1   0];
auxdata.xColMatrix = xColMatrix;
auxdata.yColMatrix = yColMatrix;

Wc = 1; % Maze cell width
Maze = ["R", "ANW", "ANE", "ANW";
        "CNW", "CSE", "CSW", "U";
        "ASE", "L", "L", "ASW"];
auxdata.Maze = Maze;
MazeOrder = {[0, 0],                [1, Wc],                [4, Wc*(1+3*(pi/4))],   [5, Wc*(1+4*(pi/4))];
            [11, Wc*(4+7*(pi/4))],  [2, Wc*(1+pi/4)],       [3, Wc*(1+2*(pi/4))],   [6, Wc*(1+5*(pi/4))];
            [10, Wc*(4+6*(pi/4))],  [9, Wc*(3+6*(pi/4))],   [8, Wc*(2+6*(pi/4))],   [7, Wc*(2+5*(pi/4))]};
auxdata.MazeOrder = MazeOrder;
auxdata.Wc = Wc; 
% This table's columns are: 
%   s       xLeft   xRight  yTop    yBottom
MazeBoundaries = [...
    0       0       2       0.5     0.5
    1       1       1       0.5     0.5
    1.5     1.5     0.5     1.5     0.5
    2       0.5     0.5     1       1
    2.5     0.5     1.5     0.5     1.5
    3       1       1       0.5     0.5
    3.5     1.5     0.5     0.5     1.5
    4       0.5     0.5     1       1
    4.5     0.5     1.5     1.5     0.5
    5       1       1       0.5     0.5
    5.5     1.5     0.5     2.5     0.5
    6       0.5     0.5     2       1
    7       0.5     0.5     1       2
    7.5     3.5     0.5     0.5     2.5
    8       3       1       0.5     0.5
    10      1       3       0.5     0.5
    10.5    0.5     3.5     0.5     1.5
    11      0.5     0.5     1       1
    11.5    0.5     0.5     1.5     0.5
    12      0       1       1.5     0.5];

% This first iteration simply attempts to get the robot to the specified
% final location and orientation, without any maze/track contraints.

auxdata.m = 5.925;                  % Robot mass
rRobot = 0.336/2;                   % Robot diameter             
auxdata.I = 0.5*auxdata.m*(rRobot^2);   % Robot z axis inertia
auxdata.w = 0.13;                   % Distance from the wheels to robot CoG

t0 = 0;                                             % initial time
tfmin = 0; tfmax = 5;                              % time boundary
v0 = 0; theta0 = 0; x0 = 0.5; y0 = 0.5; omega0 = 0; % initial state
% thetatf = 0; xf = 2.5; yf = 1.5;                    % final state
xf = 2.5; yf = 1.4;                    % final state
% thetatf = 0; xf = 3.5; yf = 0.5;                    % final state
% vmin = -16.5; vmax = 16.5;
vmin = 0; vmax = 16.5;
thetamin = -2*pi; thetamax = 2*pi;
xmin = 0; xmax = 4;
ymin = 0; ymax = 3;
omegamin = -20; omegamax = 20; % Angular rate (rad/s) limit

% motorMaxTorue = 20;
motorMaxTorue = 0.6; % Equates to a max force of 20 N per motor
wheelRadius = 0.03;
MaxForce = motorMaxTorue / wheelRadius;
Frmin = -MaxForce; Frmax = MaxForce; % Right wheel limits
Flmin = -MaxForce; Flmax = MaxForce; % Left wheel limits

%-------------------------------------------------------------------------%
%----------------------- Setup for Problem Bounds ------------------------%
%-------------------------------------------------------------------------%
bounds.phase.initialtime.lower = t0; 
bounds.phase.initialtime.upper = t0;
bounds.phase.finaltime.lower = tfmin; 
bounds.phase.finaltime.upper = tfmax;
bounds.phase.initialstate.lower = [v0,theta0,x0,y0,omega0]; 
bounds.phase.initialstate.upper = [v0,theta0,x0,y0,omega0]; 
bounds.phase.state.lower = [vmin,thetamin,xmin,ymin,omegamin]; 
bounds.phase.state.upper = [vmax,thetamax,xmax,ymax,omegamax]; 
bounds.phase.finalstate.lower = [vmin,thetamin,xf,yf,omegamin]; 
bounds.phase.finalstate.upper = [vmax,thetamax,xf,yf,omegamax]; 
bounds.phase.control.lower = [Frmin, Flmin]; 
bounds.phase.control.upper = [Frmax, Flmax];

% Assuming you have 4 path constraints (xLeft, xRight, yBottom, yTop)
% lowerPathBounds = [0, 0, 0, 0];  % Path constraint lower bounds (>= 0)
% upperPathBounds = [xmax, xmax, ymax, ymax];  % Path constraint upper bounds
% upperPathBounds = [Inf, Inf, Inf, Inf];  % Path constraint upper bounds
lowerPathBounds = -(0.5*Wc - rRobot);  % Path constraint lower bounds (>= 0)
upperPathBounds = (0.5*Wc - rRobot);  % Path constraint upper bounds

% Add these bounds to the phase
bounds.phase.path.lower = lowerPathBounds;
bounds.phase.path.upper = upperPathBounds;
% bounds.phase.path.lower = -100;
% bounds.phase.path.upper = 100;

%-------------------------------------------------------------------------%
%---------------------- Provide Guess of Solution ------------------------%
%-------------------------------------------------------------------------%
guess.phase.time    = [t0; tfmax]; 
guess.phase.state   = [[v0; vmax], [theta0; 0], [x0; xf], ...
    [y0; yf], [omega0; omegamin]];
guess.phase.control = [[Frmax; Frmax],[Flmax; Flmax]];

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
mesh.colpointsmax = 10;
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
% setup.name                        = 'RobotAndMaze-Problem';
% setup.functions.continuous        = @robotAndMazeContinuous;
% setup.functions.endpoint          = @robotAndMazeEndpoint;
% setup.auxdata                     = auxdata;
% setup.bounds                      = bounds;
% setup.guess                       = guess;
% setup.mesh                        = mesh; 
% setup.nlp.solver                  = 'ipopt';
% setup.derivatives.supplier        = 'sparseCD';
% setup.derivatives.derivativelevel = 'second';
% setup.method                      = 'RPM-Differentiation';

setup.name                           = 'RobotAndMaze-Problem';
setup.functions.continuous           = @robotAndMazeContinuous;
setup.functions.endpoint             = @robotAndMazeEndpoint;
setup.displaylevel                   = 2;
setup.nlp.solver                     = 'ipopt';
setup.nlp.ipoptoptions.linear_solver = 'ma57';
setup.bounds                         = bounds;
setup.guess                          = guess;
setup.mesh                           = mesh;
setup.auxdata                        = auxdata;
setup.derivatives.supplier           = 'sparseCD';
% setup.derivatives.supplier           = 'adigator';
setup.derivatives.derivativelevel    = 'second';
setup.derivatives.dependencies       = 'sparseNaN';
setup.scales.method                  = 'automatic-bounds';
setup.method                         = 'RPM-Differentiation';

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
output   = gpops2(setup);
solution = output.result.solution;
