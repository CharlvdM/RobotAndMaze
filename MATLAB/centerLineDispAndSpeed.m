% x, y  The robot x and y coordinates
% Maze  2D array containing the MazeCell based on the current cell
% MazeOrder Current cell's number
% Wc    A cell winth. It is assumed that the cells are square
function [n, vs] = ...
    centerLineDispAndSpeed(x, y, xDot, yDot, Maze, Wc)
if isnan(x(1)) || isnan(y(1))
    n = NaN;
    vs = 1e-6;
else
    c = floor(x/Wc); % row
    r = floor(y/Wc); % column
    % [x, y];
    % [r, c];
    % try
    cell = Maze(r+1, c+1);
    if (cell == "R") || (cell == "L") || (cell == "U") || (cell == "D")
        switch cell
            case "R"
                n = (r+0.5)*Wc - y;
                vs = xDot;
            case "L"
                n = y - (r+0.5)*Wc;
                vs = -xDot;
            case "U"
                n = x - (c+0.5)*Wc;
                vs = yDot;
            case "D"
                n = (c+0.5)*Wc - x;
                vs = -yDot;
            otherwise
                disp('Error in centerLineDisplacement');
        end
    else
        switch cell
            case "CSW"
                X = c*Wc;
                Y = r*Wc;
                re = sqrt((x-X)^2 + (y-Y)^2);
                n = 0.5*Wc - re;
            case "CNW"
                X = c*Wc;
                Y = (r+1)*Wc;
                re = sqrt((x-X)^2 + (y-Y)^2);
                n = 0.5*Wc - re;
            case "CNE"
                X = (c+1)*Wc;
                Y = (r+1)*Wc;
                re = sqrt((x-X)^2 + (y-Y)^2);
                n = 0.5*Wc - re;
            case "CSE"
                X = (c+1)*Wc;
                Y = r*Wc;
                re = sqrt((x-X)^2 + (y-Y)^2);
                n = 0.5*Wc - re;
            case "ANW"
                X = c*Wc;
                Y = (r+1)*Wc;
                re = sqrt((x-X)^2 + (y-Y)^2);
                n = re - 0.5*Wc;
            case "ANE"
                X = (c+1)*Wc;
                Y = (r+1)*Wc;
                re = sqrt((x-X)^2 + (y-Y)^2);
                n = re - 0.5*Wc;
            case "ASE"
                X = (c+1)*Wc;
                Y = r*Wc;
                re = sqrt((x-X)^2 + (y-Y)^2);
                n = re - 0.5*Wc;
            case "ASW"
                X = c*Wc;
                Y = r*Wc;
                re = sqrt((x-X)^2 + (y-Y)^2);
                n = re - 0.5*Wc;
            otherwise
                disp('Error in centerLineDisplacement');
        end
            vs = (0.5*Wc)*((x-X)*yDot - (y-Y)*xDot)/((x-X)^2+(y-Y)^2);
            if (cell == "CSW") || (cell == "CNW") || (cell == "CNE") || (cell == "CSE")
                vs = -1*vs;
            end
    end
    % catch
    % catch EXP
    %     disp([x, y]);
    %     disp([r, c]);
    %     rethrow(EXP)
    %     n = Wc/2;
    %     vs = 1e-6;
    % end
    if vs == 0
        vs = 1e-6;
    end
end
end