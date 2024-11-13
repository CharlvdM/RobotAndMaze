%---------------------------------------------------%
% Robot and Maze Problem:                           %
%---------------------------------------------------%
% The goal is for a small circular robot with two   %
% rigid wheels (iRobot Create 3) to run through a   %
% track (the solution of a maze) in miminimum time. %
%---------------------------------------------------%

clear all; close all; clc

run CreateTrackCurvature.m

Wc = 1; % Maze cell width

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

    pathConstraintsActive = true;
    tfmin = 0; tfmax = 3;                  % time boundary
    % xf = 2.5; yf = 1.3;                    % final state
    % xf = 1; yf = 0.5;                    % final state
    % Initial and final center line displacement
    s0 = 0.5*Wc;
    % sfmin = Wc + 5 * (pi / 4) * Wc;
    % sfmax = 2 * Wc + 5 * (pi / 4) * Wc;
    % sfmin = 0.5*Wc;
    % sfmax = 1.5*Wc;

    % inputCostFuncWeight = 1;
    % xf = 1.5; yf = 1;                    % final state
    % sfmin = Wc;
    % sfmax = Wc + (pi/4)*Wc;

    % xf = 2.5; yf = 1;                    % final state
    % sfmin = Wc + 2*(pi/4)*Wc;
    % sfmax = Wc + 4*(pi/4)*Wc;

    % xf = 3.5; yf = 1;                    % final state
    % sfmin = s_cell5;
    % sfmax = s_cell7;

    % inputCostFuncWeight = 1;
    % xf = 3.5; yf = 2.5;                    % final state
    % sfmin = s_cell7;
    % sfmax = s_cell8;
    % tfmax = 5;
    % 
    % inputCostFuncWeight = 10;
    % xf = 0.5; yf = 2.5;                    % final state
    % sfmin = s_cell10;
    % sfmax = s_cell11;
    % tfmax = 10;

    inputCostFuncWeight = 10;
    xf = 0.5; yf = 1.5;                    % final state
    sfmin = s_cell11;
    sfmax = s_cell12;
    tfmax = 10;

    n0 = 0;
    nmin = -(0.5*Wc - rRobot);
    nmax = (0.5*Wc - rRobot);

    xi0 = 0;
    ximin = -0.7*pi/2;
    ximax = 0.7*pi/2;

auxdata.inputCostFuncWeight = inputCostFuncWeight;
auxdata.C_track = C_track;
auxdata.s_track = s_track;

auxdata.Wc = Wc;
auxdata.m = m;
auxdata.I = I;
auxdata.w = w;

%-------------------------------------------------------------------------%
%----------------------- Setup for Problem Bounds ------------------------%
%-------------------------------------------------------------------------%
bounds.phase.initialtime.lower = s0; 
bounds.phase.initialtime.upper = s0;
bounds.phase.finaltime.lower = sfmin; 
bounds.phase.finaltime.upper = sfmax;
bounds.phase.initialstate.lower = [v0,theta0,x0,y0,omega0,n0,xi0,t0]; 
bounds.phase.initialstate.upper = [v0,theta0,x0,y0,omega0,n0,xi0,t0]; 
bounds.phase.state.lower = [vmin,thetamin,xmin,ymin,omegamin,nmin,ximin,t0]; 
bounds.phase.state.upper = [vmax,thetamax,xmax,ymax,omegamax,nmax,ximax,tfmax]; 
bounds.phase.finalstate.lower = [vmin,thetafmin,xf,yf,omegamin,nmin,ximin,tfmin]; 
bounds.phase.finalstate.upper = [vmax,thetafmax,xf,yf,omegamax,nmax,ximax,tfmax]; 
bounds.phase.control.lower = [-Frmax, -Flmax]; 
bounds.phase.control.upper = [Frmax, Flmax];
% The result of the integral is time
bounds.phase.integral.lower = tfmin;
bounds.phase.integral.upper = 100;

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
sfave = (sfmax+sfmin)/2;
% guess.phase.time    = [s0; sfave]; % The independent variable is s (center line displacement)
% guess.phase.state   = [...
%     v0,     theta0,     x0,     y0,     omega0,     n0,     xi0,    t0
%     5,   theta0,     x0,     y0,     omega0,     n0,     xi0,    tfmax];
% guess.phase.control = [[Frmax; Frmax],[Flmax; Flmax]];
% guess.phase.integral = tfmax; % guess the final time

sHalfWay = (s0+sfave)/2; % This is at the end of cell 6 currently
guess.phase.time    = [s0; sHalfWay; sfave]; % The independent variable is s (center line displacement)
guess.phase.state   = [...
    v0,     theta0,     x0,     y0,     omega0,     n0,     xi0,    t0
    1,      pi/2,       3.3,    1,      omega0,     n0,     xi0,    (tfmax-t0)/2
    1,      pi,         xf,     yf,     omega0,     n0,     xi0,    tfmax];
guess.phase.control = [[Frmax; Frmax; Frmax],[Flmax; Flmax; Flmax]];
guess.phase.integral = tfmax; % guess the final time

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
