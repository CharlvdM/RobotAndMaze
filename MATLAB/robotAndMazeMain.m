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

auxdata.m = 5.925;                  % Robot mass
rRobot = 0.336/2;                   % Robot diameter             
auxdata.I = 0.5*auxdata.m*(rRobot^2);   % Robot z axis inertia
auxdata.w = 0.13;                   % Distance from the wheels to robot CoG

t0 = 0;                                         % initial time
tfmin = 0; tfmax = 20;                          % time boundary
v0 = 0; theta0 = 0; x0 = 0; y0 = 0; omega0 = 0; % initial state
thetatf = 0; xf = 20; yf = 0;                  % final state
% vmin = -16.5; vmax = 16.5;
vmin = 0; vmax = 16.5;
thetamin = -2*pi; thetamax = 2*pi;
xmin = -10; xmax = 25;
ymin = -10; ymax = 20;
omegamin = -10; omegamax = 10; % Angular rate (rad/s) limit

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
bounds.phase.finalstate.lower = [vmin,thetatf,xf,yf,omegamin]; 
bounds.phase.finalstate.upper = [vmax,thetatf,xf,yf,omegamax]; 
bounds.phase.control.lower = [Frmin, Flmin]; 
bounds.phase.control.upper = [Frmax, Flmax];

%-------------------------------------------------------------------------%
%---------------------- Provide Guess of Solution ------------------------%
%-------------------------------------------------------------------------%
guess.phase.time    = [t0; tfmax]; 
guess.phase.state   = [[v0; v0], [theta0; thetatf], [x0; xf], [y0; yf], ...
    [omegamin; omegamax]];
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
% setup.derivatives.supplier           = 'sparseCD';
setup.derivatives.supplier           = 'adigator';
setup.derivatives.derivativelevel    = 'second';
setup.derivatives.dependencies       = 'sparseNaN';
setup.scales.method                  = 'automatic-bounds';
setup.method                         = 'RPM-Differentiation';

%-------------------------------------------------------------------------%
%------------------------- Solve Problem Using GPOP2 ---------------------%
%-------------------------------------------------------------------------%
output   = gpops2(setup);
solution = output.result.solution;
