savePlotData = false;
figDirectory = '../Latex/Figures/';
figScreenPosition = [5 5 15 9];

t = solution.phase(1).time;
x = solution.phase(1).state(:,3);
y = solution.phase(1).state(:,4);
N = length(t);
s = zeros(N,1);
cellnr = zeros(N,1);
xLeft = zeros(N,1);
xRight = zeros(N,1);
yTop = zeros(N,1);
yBottom = zeros(N,1);
Wc = 1;

for i = 1:N
    [s(i), cellnr(i)] = centerLineDisplacement(x(i), y(i), Maze, MazeOrder, Wc);
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

figure(7)
pp = plot(x, cellnr,'-o');
xl = xlabel('$x$','Interpreter','LaTeX');
yl = ylabel('$cellnr$','Interpreter','LaTeX');
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

figure(8)
pp = plot(x, xLeft, x, xLeftActual, '-o');
xl = xlabel('$x$','Interpreter','LaTeX');
yl = ylabel('xLeft','Interpreter','LaTeX');
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

figure(9)
pp = plot(x, yTop, x, yTopActual, '-o');
xl = xlabel('$x$','Interpreter','LaTeX');
yl = ylabel('yTop','Interpreter','LaTeX');
% ll = legend('$x(t)$','$y(t)$','$v(t)$','Location','NorthWest');
% set(ll,'FontSize',18,'Interpreter','LaTeX');
set(pp,'LineWidth',1.25,'MarkerSize',8);
set(xl,'FontSize',18);
set(yl,'FontSize',18);
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

figure
pp = plot(s_test, xRight_test);
xl = xlabel('$s$','Interpreter','LaTeX');
yl = ylabel('xRight','Interpreter','LaTeX');
% ll = legend('$x(t)$','$y(t)$','$v(t)$','Location','NorthWest');
% set(ll,'FontSize',18,'Interpreter','LaTeX');
set(pp,'LineWidth',1.25,'MarkerSize',8);
set(xl,'FontSize',18);
set(yl,'FontSize',18);
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

figure
pp = plot(s_test, yTop_test);
xl = xlabel('$s$','Interpreter','LaTeX');
yl = ylabel('yTop','Interpreter','LaTeX');
% ll = legend('$x(t)$','$y(t)$','$v(t)$','Location','NorthWest');
% set(ll,'FontSize',18,'Interpreter','LaTeX');
set(pp,'LineWidth',1.25,'MarkerSize',8);
set(xl,'FontSize',18);
set(yl,'FontSize',18);
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