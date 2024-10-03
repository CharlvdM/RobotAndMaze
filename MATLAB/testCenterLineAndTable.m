savePlotData = false;
figDirectory = '../Latex/Figures/';
figScreenPosition = [5 5 15 9];

t = solution.phase(1).time;
v = solution.phase(1).state(:,1);
theta = solution.phase(1).state(:,2);
x = solution.phase(1).state(:,3);
y = solution.phase(1).state(:,4);
xDot = v.*cos(theta);
yDot = v.*sin(theta);
N = length(t);
% s = zeros(N,1);
cellnr = zeros(N,1);
d = zeros(N,1);
s = zeros(N,1);
vs = zeros(N,1);

for i = 1:N
    % [s(i), cellnr(i), ~] = centerLineDisplacement(x(i), y(i), Maze, MazeOrder, Wc);
    [d(i), s(i), vs(i)] = centerLineDispNew(x(i), y(i), xDot(i), yDot(i), Maze, MazeOrder, Wc);
end
method = 'pchip';
xLeft   = interp1(MazeBoundaries(:,1),MazeBoundaries(:,2),s,method);
xRight  = interp1(MazeBoundaries(:,1),MazeBoundaries(:,3),s,method);
yTop    = interp1(MazeBoundaries(:,1),MazeBoundaries(:,4),s,method);
yBottom = interp1(MazeBoundaries(:,1),MazeBoundaries(:,5),s,method);
[xLeftActual, xRightActual, yBottomActual, yTopActual] = ...
    calcCollisionDistances(x, y, xColMatrix, yColMatrix);

s_test = linspace(0, 12, 12*4);
% method = 'pchip';
method = 'splin';
xLeft_test   = interp1(MazeBoundaries(:,1),MazeBoundaries(:,2),s_test,method);
xRight_test  = interp1(MazeBoundaries(:,1),MazeBoundaries(:,3),s_test,method);
yTop_test    = interp1(MazeBoundaries(:,1),MazeBoundaries(:,4),s_test,method);
yBottom_test = interp1(MazeBoundaries(:,1),MazeBoundaries(:,5),s_test,method);

figure(6)
pp = plot(x, s,'-o');
xl = xlabel('$x$','Interpreter','LaTeX');
yl = ylabel('$s$','Interpreter','LaTeX');
% ll = legend('$x(t)$','$y(t)$','$v(t)$','Location','NorthWest');
set(pp,'LineWidth',1.25,'MarkerSize',8);
set(xl,'FontSize',18);
set(yl,'FontSize',18);
% set(ll,'FontSize',18,'Interpreter','LaTeX');
set(gca,'FontSize',16,'FontName','Times');
grid on
% set(gca, 'FontSize', fontSize, 'FontName', font);
set(gcf, 'PaperUnits', 'centimeters', 'Units', 'centimeters')
set(gcf,'Position', figScreenPosition);
set(gcf, 'PaperPosition', [0 0 figScreenPosition(3:4)],...
    'PaperSize', figScreenPosition(3:4));
if savePlotData == true
    % print(gcf, '-dpdf', '-painters', strcat(figDirectory, 'States'))
end

% figure(7)
% pp = plot(x, cellnr,'-o');
% xl = xlabel('$x$','Interpreter','LaTeX');
% yl = ylabel('$cellnr$','Interpreter','LaTeX');
% % ll = legend('$x(t)$','$y(t)$','$v(t)$','Location','NorthWest');
% set(pp,'LineWidth',1.25,'MarkerSize',8);
% set(xl,'FontSize',18);
% set(yl,'FontSize',18);
% % set(ll,'FontSize',18,'Interpreter','LaTeX');
% set(gca,'FontSize',16,'FontName','Times');
% grid on
% % set(gca, 'FontSize', fontSize, 'FontName', font);
% set(gcf, 'PaperUnits', 'centimeters', 'Units', 'centimeters')
% set(gcf,'Position', figScreenPosition);
% set(gcf, 'PaperPosition', [0 0 figScreenPosition(3:4)],...
%     'PaperSize', figScreenPosition(3:4));
% if savePlotData == true
%     % print(gcf, '-dpdf', '-painters', strcat(figDirectory, 'States'))
% end

% figure(8)
% pp = plot(x, xLeft, x, xLeftActual, '-o');
% xl = xlabel('$x$','Interpreter','LaTeX');
% yl = ylabel('xLeft','Interpreter','LaTeX');
% % ll = legend('$x(t)$','$y(t)$','$v(t)$','Location','NorthWest');
% set(pp,'LineWidth',1.25,'MarkerSize',8);
% set(xl,'FontSize',18);
% set(yl,'FontSize',18);
% % set(ll,'FontSize',18,'Interpreter','LaTeX');
% set(gca,'FontSize',16,'FontName','Times');
% grid on
% % set(gca, 'FontSize', fontSize, 'FontName', font);
% set(gcf, 'PaperUnits', 'centimeters', 'Units', 'centimeters')
% set(gcf,'Position', figScreenPosition);
% set(gcf, 'PaperPosition', [0 0 figScreenPosition(3:4)],...
%     'PaperSize', figScreenPosition(3:4));
% if savePlotData == true
%     % print(gcf, '-dpdf', '-painters', strcat(figDirectory, 'States'))
% end

% figure(9)
% pp = plot(x, yTop, x, yTopActual, '-o');
% xl = xlabel('$x$','Interpreter','LaTeX');
% yl = ylabel('yTop','Interpreter','LaTeX');
% % ll = legend('$x(t)$','$y(t)$','$v(t)$','Location','NorthWest');
% % set(ll,'FontSize',18,'Interpreter','LaTeX');
% set(pp,'LineWidth',1.25,'MarkerSize',8);
% set(xl,'FontSize',18);
% set(yl,'FontSize',18);
% set(gca,'FontSize',16,'FontName','Times');
% grid on
% % set(gca, 'FontSize', fontSize, 'FontName', font);
% set(gcf, 'PaperUnits', 'centimeters', 'Units', 'centimeters')
% set(gcf,'Position', figScreenPosition);
% set(gcf, 'PaperPosition', [0 0 figScreenPosition(3:4)],...
%     'PaperSize', figScreenPosition(3:4));
% if savePlotData == true
%     % print(gcf, '-dpdf', '-painters', strcat(figDirectory, 'States'))
% end

% figure
% pp = plot(s_test, xRight_test);
% xl = xlabel('$s$','Interpreter','LaTeX');
% yl = ylabel('xRight','Interpreter','LaTeX');
% % ll = legend('$x(t)$','$y(t)$','$v(t)$','Location','NorthWest');
% % set(ll,'FontSize',18,'Interpreter','LaTeX');
% set(pp,'LineWidth',1.25,'MarkerSize',8);
% set(xl,'FontSize',18);
% set(yl,'FontSize',18);
% set(gca,'FontSize',16,'FontName','Times');
% grid on
% % set(gca, 'FontSize', fontSize, 'FontName', font);
% set(gcf, 'PaperUnits', 'centimeters', 'Units', 'centimeters')
% set(gcf,'Position', figScreenPosition);
% set(gcf, 'PaperPosition', [0 0 figScreenPosition(3:4)],...
%     'PaperSize', figScreenPosition(3:4));
% if savePlotData == true
%     % print(gcf, '-dpdf', '-painters', strcat(figDirectory, 'States'))
% end

% figure
% pp = plot(s_test, yTop_test);
% xl = xlabel('$s$','Interpreter','LaTeX');
% yl = ylabel('yTop','Interpreter','LaTeX');
% % ll = legend('$x(t)$','$y(t)$','$v(t)$','Location','NorthWest');
% % set(ll,'FontSize',18,'Interpreter','LaTeX');
% set(pp,'LineWidth',1.25,'MarkerSize',8);
% set(xl,'FontSize',18);
% set(yl,'FontSize',18);
% set(gca,'FontSize',16,'FontName','Times');
% grid on
% % set(gca, 'FontSize', fontSize, 'FontName', font);
% set(gcf, 'PaperUnits', 'centimeters', 'Units', 'centimeters')
% set(gcf,'Position', figScreenPosition);
% set(gcf, 'PaperPosition', [0 0 figScreenPosition(3:4)],...
%     'PaperSize', figScreenPosition(3:4));
% if savePlotData == true
%     % print(gcf, '-dpdf', '-painters', strcat(figDirectory, 'States'))
% end

figure(12)
pp = plot(x, d,'-o', [x(1), x(end)], [0.5*Wc - rRobot, 0.5*Wc - rRobot], 'k');
xl = xlabel('$x$','Interpreter','LaTeX');
yl = ylabel('$d$','Interpreter','LaTeX');
% ll = legend('$x(t)$','$y(t)$','$v(t)$','Location','NorthWest');
set(pp,'LineWidth',1.25,'MarkerSize',8);
set(xl,'FontSize',18);
set(yl,'FontSize',18);
% set(ll,'FontSize',18,'Interpreter','LaTeX');
set(gca,'FontSize',16,'FontName','Times');
grid on
% set(gca, 'FontSize', fontSize, 'FontName', font);
set(gcf, 'PaperUnits', 'centimeters', 'Units', 'centimeters')
set(gcf,'Position', figScreenPosition);
set(gcf, 'PaperPosition', [0 0 figScreenPosition(3:4)],...
    'PaperSize', figScreenPosition(3:4));
if savePlotData == true
    % print(gcf, '-dpdf', '-painters', strcat(figDirectory, 'States'))
end

figure(13)
pp = plot(t, d,'-o', [t(1), t(end)], [0.5*Wc - rRobot, 0.5*Wc - rRobot], 'k');
xl = xlabel('$t$','Interpreter','LaTeX');
yl = ylabel('$d$','Interpreter','LaTeX');
% ll = legend('$x(t)$','$y(t)$','$v(t)$','Location','NorthWest');
set(pp,'LineWidth',1.25,'MarkerSize',8);
set(xl,'FontSize',18);
set(yl,'FontSize',18);
% set(ll,'FontSize',18,'Interpreter','LaTeX');
set(gca,'FontSize',16,'FontName','Times');
grid on
% set(gca, 'FontSize', fontSize, 'FontName', font);
set(gcf, 'PaperUnits', 'centimeters', 'Units', 'centimeters')
set(gcf,'Position', figScreenPosition);
set(gcf, 'PaperPosition', [0 0 figScreenPosition(3:4)],...
    'PaperSize', figScreenPosition(3:4));
if savePlotData == true
    % print(gcf, '-dpdf', '-painters', strcat(figDirectory, 'States'))
end

figure(14)
pp = plot(t, s,'-o');
xl = xlabel('$t$','Interpreter','LaTeX');
yl = ylabel('$s$','Interpreter','LaTeX');
% ll = legend('$x(t)$','$y(t)$','$v(t)$','Location','NorthWest');
set(pp,'LineWidth',1.25,'MarkerSize',8);
set(xl,'FontSize',18);
set(yl,'FontSize',18);
% set(ll,'FontSize',18,'Interpreter','LaTeX');
set(gca,'FontSize',16,'FontName','Times');
grid on
% set(gca, 'FontSize', fontSize, 'FontName', font);
set(gcf, 'PaperUnits', 'centimeters', 'Units', 'centimeters')
set(gcf,'Position', figScreenPosition);
set(gcf, 'PaperPosition', [0 0 figScreenPosition(3:4)],...
    'PaperSize', figScreenPosition(3:4));
if savePlotData == true
    % print(gcf, '-dpdf', '-painters', strcat(figDirectory, 'States'))
end

dt = diff(t);       % Differences in time (dt)
ds = diff(s);       % Differences in displacement (ds)
% vs = ds ./ dt;      % Element-wise division to get velocity
% vs = [0; ds ./ dt];
vs_calc = [ds(1) ./ dt(1); (ds(1:end-1) + ds(2:end)) ./ (dt(1:end-1) + dt(2:end)); ds(end) ./ dt(end)];

figure(15)
pp = plot(t, v, t, vs,'-o', t, vs_calc);
xl = xlabel('$t$','Interpreter','LaTeX');
yl = ylabel('$v$','Interpreter','LaTeX');
ll = legend('$v(t)$','$v_s(t)$','$v_\mathrm{calc}(t)$','Location','NorthWest');
set(pp,'LineWidth',1.25,'MarkerSize',8);
set(xl,'FontSize',18);
set(yl,'FontSize',18);
set(ll,'FontSize',18,'Interpreter','LaTeX');
set(gca,'FontSize',16,'FontName','Times');
grid on
% set(gca, 'FontSize', fontSize, 'FontName', font);
set(gcf, 'PaperUnits', 'centimeters', 'Units', 'centimeters')
set(gcf,'Position', figScreenPosition);
set(gcf, 'PaperPosition', [0 0 figScreenPosition(3:4)],...
    'PaperSize', figScreenPosition(3:4));
if savePlotData == true
    % print(gcf, '-dpdf', '-painters', strcat(figDirectory, 'States'))
end

figure(16)
pp = plot(x, v, x, vs,'-o');
xl = xlabel('$x$','Interpreter','LaTeX');
yl = ylabel('$v$','Interpreter','LaTeX');
% ll = legend('$x(t)$','$y(t)$','$v(t)$','Location','NorthWest');
set(pp,'LineWidth',1.25,'MarkerSize',8);
set(xl,'FontSize',18);
set(yl,'FontSize',18);
% set(ll,'FontSize',18,'Interpreter','LaTeX');
set(gca,'FontSize',16,'FontName','Times');
grid on
% set(gca, 'FontSize', fontSize, 'FontName', font);
set(gcf, 'PaperUnits', 'centimeters', 'Units', 'centimeters')
set(gcf,'Position', figScreenPosition);
set(gcf, 'PaperPosition', [0 0 figScreenPosition(3:4)],...
    'PaperSize', figScreenPosition(3:4));
if savePlotData == true
    % print(gcf, '-dpdf', '-painters', strcat(figDirectory, 'States'))
end