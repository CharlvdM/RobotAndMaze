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

figure(1)
pp = plot(solution.phase(1).time, solution.phase(1).state(:,3:4),'-o', ...
    solution.phase(1).time, solution.phase(1).state(:,1),'-o');
xl = xlabel('$t$','Interpreter','LaTeX');
yl = ylabel('$(x(t),y(t),v(t))$','Interpreter','LaTeX');
ll = legend('$x(t)$','$y(t)$','$v(t)$','Location','NorthWest');
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
    print(gcf, '-dpdf', '-painters', strcat(figDirectory, 'States'))
end
% print -dpng robotAndMazeState.png

figure(2)
pp = plot(solution.phase(1).time, solution.phase(1).state(:,2),'-o');
xl = xlabel('$t$','Interpreter','LaTeX');
yl = ylabel('$\theta(t)$','Interpreter','LaTeX');
set(pp,'LineWidth',1.25,'MarkerSize',8);
set(xl,'FontSize',18);
set(yl,'FontSize',18);
set(ll,'FontSize',18,'Interpreter','LaTeX');
set(gca,'FontSize',16,'FontName','Times');
grid on
set(gcf, 'PaperUnits', 'centimeters', 'Units', 'centimeters')
set(gcf,'Position', figScreenPosition);
set(gcf, 'PaperPosition', [0 0 figScreenPosition(3:4)],...
    'PaperSize', figScreenPosition(3:4));
if savePlotData == true
    print(gcf, '-dpdf', '-painters', strcat(figDirectory, 'Angle'))
end

figure(3)
pp = plot(solution.phase(1).time,solution.phase(1).control,'-o');
xl = xlabel('$t$','Interpreter','LaTeX');
yl = ylabel('$u(t)$','Interpreter','LaTeX');
ll = legend('$F_R(t)$','$F_L(t)$','Location','NorthWest');
set(pp,'LineWidth',1.25,'MarkerSize',8);
set(xl,'FontSize',18);
set(yl,'FontSize',18);
set(ll,'FontSize',18,'Interpreter','LaTeX');
set(gca,'FontSize',16,'FontName','Times');
grid on
set(gcf, 'PaperUnits', 'centimeters', 'Units', 'centimeters')
set(gcf,'Position', figScreenPosition);
set(gcf, 'PaperPosition', [0 0 figScreenPosition(3:4)],...
    'PaperSize', figScreenPosition(3:4));
if savePlotData == true
    print(gcf, '-dpdf', '-painters', strcat(figDirectory, 'Forces'))
end

figure(4)
pp = plot(x,y,'-o');
hold on
LW = 2;
plot([0 4 4 0 0], [0 0 3 3 0], 'k', 'LineWidth', LW);
plot([2 2], [0 1], 'k', 'LineWidth', LW);
plot([0 1 1 3 3], [1 1 2 2 1], 'k', 'LineWidth', LW);
clear LW;
hold off
xl = xlabel('$x$','Interpreter','LaTeX');
yl = ylabel('$y$','Interpreter','LaTeX');
set(pp,'LineWidth',1.25,'MarkerSize',20);
set(xl,'FontSize',18);
set(yl,'FontSize',18);
set(ll,'FontSize',18,'Interpreter','LaTeX');
set(gca,'FontSize',16,'FontName','Times');
grid on
set(gcf, 'PaperUnits', 'centimeters', 'Units', 'centimeters')
figScreenPositionTrace = [5 5 12 9];
set(gcf,'Position', figScreenPositionTrace);
set(gcf, 'PaperPosition', [0 0 figScreenPositionTrace(3:4)],...
    'PaperSize', figScreenPositionTrace(3:4));
if savePlotData == true
    print(gcf, '-dpdf', '-painters', strcat(figDirectory, 'Trajectory'))
end

figure(5)
pp = plot(solution.phase(1).time,solution.phase(1).state(:,5),'-o');
xl = xlabel('$t$','Interpreter','LaTeX');
yl = ylabel('$\omega(t)$','Interpreter','LaTeX');
set(pp,'LineWidth',1.25,'MarkerSize',8);
set(xl,'FontSize',18);
set(yl,'FontSize',18);
set(ll,'FontSize',18,'Interpreter','LaTeX');
set(gca,'FontSize',16,'FontName','Times');
grid on
set(gcf, 'PaperUnits', 'centimeters', 'Units', 'centimeters')
set(gcf,'Position', figScreenPosition);
set(gcf, 'PaperPosition', [0 0 figScreenPosition(3:4)],...
    'PaperSize', figScreenPosition(3:4));
