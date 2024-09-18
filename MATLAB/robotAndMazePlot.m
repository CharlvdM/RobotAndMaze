savePlotData = true;
figDirectory = '../Latex/Figures/';
figScreenPosition = [5 5 15 9];

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
pp = plot(solution.phase(1).state(:,3),solution.phase(1).state(:,4),'-o');
xl = xlabel('$x$','Interpreter','LaTeX');
yl = ylabel('$y$','Interpreter','LaTeX');
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
    print(gcf, '-dpdf', '-painters', strcat(figDirectory, 'Trajectory'))
end
