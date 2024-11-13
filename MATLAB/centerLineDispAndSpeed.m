% x, y  The robot x and y coordinates
% Maze  2D array containing the MazeCell based on the current cell
% MazeOrder Current cell's number
% Wc    A cell winth. It is assumed that the cells are square
function [n, vs] = ...
    centerLineDispAndSpeed(x, y, xDot, yDot, Maze, Wc)
N = size(x,1);
if isnan(x(1)) || isnan(y(1))
    n = NaN .* ones(N,1);
    vs = 1e-6 .* ones(N,1);
else
    % re = sqrt(x.^2 + y.^2);
    % n = re - 0.5*Wc;
    % vs = (0.5*Wc).*(x.*yDot - y.*xDot)./(x.^2+y.^2);    
    c = floor(x/Wc); % rows
    r = floor(y/Wc); % columns
    linearIndices = sub2ind(size(Maze), r+1, c+1);
    cell = Maze(linearIndices);

    n = zeros(N,1);
    vs = zeros(N,1);
    dir = zeros(N,1);
    X = zeros(N,1);
    Y = zeros(N,1);
    corner = zeros(N,1); % Will have a value of 1 if it's a corner cell
    for i = 1:N
        if cell(i) <= 4
            switch cell(i)
                case 1 % Right
                    n(i) = (r(i)+0.5)*Wc - y(i);
                    vs(i) = xDot(i);
                case 2 % Left
                    n(i) = y(i) - (r(i)+0.5)*Wc;
                    vs(i) = -xDot(i);
                case 3 % Up
                    n(i) = x(i) - (c(i)+0.5)*Wc;
                    vs(i) = yDot(i);
                case 4 % Down
                    n(i) = (c(i)+0.5)*Wc - x(i);
                    vs(i) = -yDot(i);
                otherwise
                    disp('Error in centerLineDisplacement');
            end
        else
            corner(i) = 1;
            switch cell(i)
                case 5 % "CSW"
                    X(i) = c(i)*Wc;
                    Y(i) = r(i)*Wc;
                    dir(i) = -1;
                case 6 % "CNW"
                    X(i) = c(i)*Wc;
                    Y(i) = (r(i)+1)*Wc;
                    dir(i) = -1;
                case 7 % "CNE"
                    X(i) = (c(i)+1)*Wc;
                    Y(i) = (r(i)+1)*Wc;
                    dir(i) = -1;
                case 8 % "CSE"
                    X(i) = (c(i)+1)*Wc;
                    Y(i) = r(i)*Wc;
                    dir(i) = -1;
                case 9 % "ANW"
                    X(i) = c(i)*Wc;
                    Y(i) = (r(i)+1)*Wc;
                    dir(i) = 1;
                case 10 % "ANE"
                    X(i) = (c(i)+1)*Wc;
                    Y(i) = (r(i)+1)*Wc;
                    dir = 1;
                case 11 % "ASE"
                    X(i) = (c(i)+1)*Wc;
                    Y(i) = r(i)*Wc;
                    dir(i) = 1;
                case 12 % "ASW"
                    X(i) = c(i)*Wc;
                    Y(i) = r(i)*Wc;
                    dir(i) = 1;
                otherwise
                    disp('Error in centerLineDisplacement');
            end
            % re(i) = sqrt((x(i)-X(i))^2 + (y(i)-Y(i))^2);
            % n(i) = 0.5*Wc - re(i);
            % vs(i) = (0.5*Wc)*((x(i)-X(i))*yDot(i) - (y(i)-Y(i))*xDot(i))/((x(i)-X(i))^2+(y(i)-Y(i))^2);
            % vs(i) = dir(i)*vs(i);
        end
    end
    re = sqrt((x-X).^2 + (y-Y).^2);
    n = n + corner .* (re - 0.5*Wc);
    vs = vs + dir .* ((0.5*Wc).*(x.*yDot - y.*xDot)./(x.^2+y.^2));

    % 
    % for i = 1:N
    %     if cell(i) <= 4
    %         switch cell
    %             case 1 % Right
    %                 n = (r+0.5)*Wc - y;
    %                 vs = xDot;
    %             case 2 % Left
    %                 n = y - (r+0.5)*Wc;
    %                 vs = -xDot;
    %             case 3 % Up
    %                 n = x - (c+0.5)*Wc;
    %                 vs = yDot;
    %             case 4 % Down
    %                 n = (c+0.5)*Wc - x;
    %                 vs = -yDot;
    %             otherwise
    %                 disp('Error in centerLineDisplacement');
    %         end
    %     else
    %         switch cell
    %             case 5 % "CSW"
    %                 X = c*Wc;
    %                 Y = r*Wc;
    %                 dir = -1;
    %             case 6 % "CNW"
    %                 X = c*Wc;
    %                 Y = (r+1)*Wc;
    %                 dir = -1;
    %             case 7 % "CNE"
    %                 X = (c+1)*Wc;
    %                 Y = (r+1)*Wc;
    %                 dir = -1;
    %             case 8 % "CSE"
    %                 X = (c+1)*Wc;
    %                 Y = r*Wc;
    %                 dir = -1;
    %             case 9 % "ANW"
    %                 X = c*Wc;
    %                 Y = (r+1)*Wc;
    %                 dir = 1;
    %             case 10 % "ANE"
    %                 X = (c+1)*Wc;
    %                 Y = (r+1)*Wc;
    %                 dir = 1;
    %             case 11 % "ASE"
    %                 X = (c+1)*Wc;
    %                 Y = r*Wc;
    %                 dir = 1;
    %             case 12 % "ASW"
    %                 X = c*Wc;
    %                 Y = r*Wc;
    %                 dir = 1;
    %             otherwise
    %                 disp('Error in centerLineDisplacement');
    %         end
    %         re = sqrt((x-X)^2 + (y-Y)^2);
    %         n = 0.5*Wc - re;
    %         vs = (0.5*Wc)*((x-X)*yDot - (y-Y)*xDot)/((x-X)^2+(y-Y)^2);
    %         vs = dir*vs;
    %     end
    % end




    % c = floor(x/Wc); % row
    % r = floor(y/Wc); % column
    % % [x, y];
    % % [r, c];
    % % try
    % cell = Maze(r+1, c+1);
    % if (cell == "R") || (cell == "L") || (cell == "U") || (cell == "D")
    %     switch cell
    %         case "R"
    %             n = (r+0.5)*Wc - y;
    %             vs = xDot;
    %         case "L"
    %             n = y - (r+0.5)*Wc;
    %             vs = -xDot;
    %         case "U"
    %             n = x - (c+0.5)*Wc;
    %             vs = yDot;
    %         case "D"
    %             n = (c+0.5)*Wc - x;
    %             vs = -yDot;
    %         otherwise
    %             disp('Error in centerLineDisplacement');
    %     end
    % else
    %     switch cell
    %         case "CSW"
    %             X = c*Wc;
    %             Y = r*Wc;
    %             re = sqrt((x-X)^2 + (y-Y)^2);
    %             n = 0.5*Wc - re;
    %         case "CNW"
    %             X = c*Wc;
    %             Y = (r+1)*Wc;
    %             re = sqrt((x-X)^2 + (y-Y)^2);
    %             n = 0.5*Wc - re;
    %         case "CNE"
    %             X = (c+1)*Wc;
    %             Y = (r+1)*Wc;
    %             re = sqrt((x-X)^2 + (y-Y)^2);
    %             n = 0.5*Wc - re;
    %         case "CSE"
    %             X = (c+1)*Wc;
    %             Y = r*Wc;
    %             re = sqrt((x-X)^2 + (y-Y)^2);
    %             n = 0.5*Wc - re;
    %         case "ANW"
    %             X = c*Wc;
    %             Y = (r+1)*Wc;
    %             re = sqrt((x-X)^2 + (y-Y)^2);
    %             n = re - 0.5*Wc;
    %         case "ANE"
    %             X = (c+1)*Wc;
    %             Y = (r+1)*Wc;
    %             re = sqrt((x-X)^2 + (y-Y)^2);
    %             n = re - 0.5*Wc;
    %         case "ASE"
    %             X = (c+1)*Wc;
    %             Y = r*Wc;
    %             re = sqrt((x-X)^2 + (y-Y)^2);
    %             n = re - 0.5*Wc;
    %         case "ASW"
    %             X = c*Wc;
    %             Y = r*Wc;
    %             re = sqrt((x-X)^2 + (y-Y)^2);
    %             n = re - 0.5*Wc;
    %         otherwise
    %             disp('Error in centerLineDisplacement');
    %     end
    %         vs = (0.5*Wc)*((x-X)*yDot - (y-Y)*xDot)/((x-X)^2+(y-Y)^2);
    %         if (cell == "CSW") || (cell == "CNW") || (cell == "CNE") || (cell == "CSE")
    %             vs = -1*vs;
    %         end
    % end
    % % catch
    % % catch EXP
    % %     disp([x, y]);
    % %     disp([r, c]);
    % %     rethrow(EXP)
    % %     n = Wc/2;
    % %     vs = 1e-6;
    % % end
    % if vs == 0
    %     vs = 1e-6;
    % end
end
end