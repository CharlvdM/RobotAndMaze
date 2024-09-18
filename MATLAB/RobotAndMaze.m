clear; close all;

initialConditions = zeros(6, 1);
systemVars.odeOpts = odeset('RelTol',1e-8,'AbsTol',1e-10);
tf = 300;

func = @f;
tspan = [0 tf];
[t, x_Ode] = ode45(func, tspan, initialConditions, systemVars.odeOpts);

% func_new = @f_new;
func_new = @robotAndMazeContinuousWrapper;
[t_new, x_Ode_new] = ode45(func_new, tspan, zeros(5, 1), systemVars.odeOpts);

figure
pp = plot(t, x_Ode(:,1:3),'-o', t_new, x_Ode_new(:,3:4), t_new, x_Ode_new(:,2));
xl = xlabel('$t$','Interpreter','LaTeX');
yl = ylabel('Costate Variables','Interpreter','LaTeX');
ll = legend('$x$','$y$','$\theta$',...
    'Location','NorthWest');
set(pp,'LineWidth',1,'MarkerSize',6); 
set(xl,'FontSize',16);
set(yl,'FontSize',16);
set(ll,'FontSize',16,'Interpreter','LaTeX');
set(gca,'FontSize',16,'FontName','Times');
grid on

figure
pp = plot(t, x_Ode(:,4:5));
xl = xlabel('$t$','Interpreter','LaTeX');
yl = ylabel('Costate Variables','Interpreter','LaTeX');
ll = legend('$v_x$','$v_y$',...
    'Location','NorthWest');
set(pp,'LineWidth',1,'MarkerSize',6);
set(xl,'FontSize',16);
set(yl,'FontSize',16);
set(ll,'FontSize',16,'Interpreter','LaTeX');
set(gca,'FontSize',16,'FontName','Times');
grid on

figure
pp = plot(x_Ode(:,1), x_Ode(:,2));
% xl = xlabel('$t$','Interpreter','LaTeX');
% yl = ylabel('Costate Variables','Interpreter','LaTeX');
% ll = legend('$x$','$y$','$\theta$',...
%     'Location','NorthWest');
set(pp,'LineWidth',1,'MarkerSize',6);
set(xl,'FontSize',16);
set(yl,'FontSize',16);
set(ll,'FontSize',16,'Interpreter','LaTeX');
set(gca,'FontSize',16,'FontName','Times');
grid on


figure
pp = plot(x_Ode_new(:,3), x_Ode_new(:,4));
% xl = xlabel('$t$','Interpreter','LaTeX');
% yl = ylabel('Costate Variables','Interpreter','LaTeX');
% ll = legend('$x$','$y$','$\theta$',...
%     'Location','NorthWest');
set(pp,'LineWidth',1,'MarkerSize',6);
set(xl,'FontSize',16);
set(yl,'FontSize',16);
set(ll,'FontSize',16,'Interpreter','LaTeX');
set(gca,'FontSize',16,'FontName','Times');
grid on


figure
pp = plot(t_new, x_Ode_new(:,1), t, sqrt(x_Ode(:,4).^2 + x_Ode(:,4).^2));
xl = xlabel('$t$','Interpreter','LaTeX');
yl = ylabel('Velocity','Interpreter','LaTeX');
ll = legend('New','Old', 'Location','NorthWest');
set(pp,'LineWidth',1,'MarkerSize',6);
set(xl,'FontSize',16);
set(yl,'FontSize',16);
set(ll,'FontSize',16,'Interpreter','LaTeX');
set(gca,'FontSize',16,'FontName','Times');
grid on

figure
pp = plot(t_new, x_Ode_new(:,2));
xl = xlabel('$t$','Interpreter','LaTeX');
yl = ylabel('Angle','Interpreter','LaTeX');
set(pp,'LineWidth',1,'MarkerSize',6);
set(xl,'FontSize',16);
set(yl,'FontSize',16);
set(ll,'FontSize',16,'Interpreter','LaTeX');
set(gca,'FontSize',16,'FontName','Times');
grid on

figure
pp = plot(t_new, x_Ode_new(:,3:4));
xl = xlabel('$t$','Interpreter','LaTeX');
yl = ylabel('Anlge','Interpreter','LaTeX');
ll = legend('New','Old', 'Location','NorthWest');
set(pp,'LineWidth',1,'MarkerSize',6);
set(xl,'FontSize',16);
set(yl,'FontSize',16);
set(ll,'FontSize',16,'Interpreter','LaTeX');
set(gca,'FontSize',16,'FontName','Times');
grid on

function xDot = f(~, x)
    u1 = 0.5;
    u2 = 1.5;
    m = 20;
    Izz = 10;
    w = 0.01;

    xDot = zeros(6, 1);
    xDot(1) = x(4);
    xDot(2) = x(5);
    xDot(3) = x(6);
    xDot(4) = (1/m)*(u1+u2)*cos(x(3));
    xDot(5) = (1/m)*(u1+u2)*sin(x(3));
    xDot(6) = (1/Izz)*w*(u1-u2);
end

function xDot = f_new(~, x)
    u1 = 0.5;
    u2 = 1.5;
    m = 20;
    Izz = 10;
    w = 0.01;

    xDot = zeros(5, 1);
    xDot(1) = (1/m)*(u1+u2);
    xDot(2) = x(5);
    xDot(3) = x(1)*cos(x(2));
    xDot(4) = x(1)*sin(x(2));
    xDot(5) = (1/Izz)*w*(u1-u2);
end

function xDot = robotAndMazeContinuousWrapper(~, x)
    input.auxdata.m = 5.925;    % Robot mass
    rRobot = 0.336/2;           % Robot diameter             
    input.auxdata.I = 0.5*input.auxdata.m*(rRobot^2);   % Robot z axis inertia
    input.auxdata.w = 0.13;     % Distance from the wheels to robot CoG

    input.phase.state = x';
    input.phase.control(:,1) = 1.1;
    input.phase.control(:,2) = 1;

    phaseout = robotAndMazeContinuous(input);
    xDot = phaseout.dynamics';
end