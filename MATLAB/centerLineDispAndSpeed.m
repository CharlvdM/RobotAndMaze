% x, y  The robot x and y coordinates
% Maze  2D array containing the MazeCell based on the current cell
% MazeOrder Current cell's number
% Wc    A cell width. It is assumed that the cells are square
function [d, vs] = ...
    centerLineDispAndSpeed(x, y, xDot, yDot, Maze, Wc)
if isnan(x(1)) || isnan(y(1))
    d = NaN;
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
                d = (r+0.5)*Wc - y;
                vs = xDot;
            case "L"
                d = y - (r+0.5)*Wc;
                vs = -xDot;
            case "U"
                d = x - (c+0.5)*Wc;
                vs = yDot;
            case "D"
                d = (c+0.5)*Wc - x;
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
                d = 0.5*Wc - re;
            case "CNW"
                X = c*Wc;
                Y = (r+1)*Wc;
                re = sqrt((x-X)^2 + (y-Y)^2);
                d = 0.5*Wc - re;
            case "CNE"
                X = (c+1)*Wc;
                Y = (r+1)*Wc;
                re = sqrt((x-X)^2 + (y-Y)^2);
                d = 0.5*Wc - re;
            case "CSE"
                X = (c+1)*Wc;
                Y = r*Wc;
                re = sqrt((x-X)^2 + (y-Y)^2);
                d = 0.5*Wc - re;
            case "ANW"
                X = c*Wc;
                Y = (r+1)*Wc;
                re = sqrt((x-X)^2 + (y-Y)^2);
                d = re - 0.5*Wc;
            case "ANE"
                X = (c+1)*Wc;
                Y = (r+1)*Wc;
                re = sqrt((x-X)^2 + (y-Y)^2);
                d = re - 0.5*Wc;
            case "ASE"
                X = (c+1)*Wc;
                Y = r*Wc;
                re = sqrt((x-X)^2 + (y-Y)^2);
                d = re - 0.5*Wc;
            case "ASW"
                X = c*Wc;
                Y = r*Wc;
                re = sqrt((x-X)^2 + (y-Y)^2);
                d = re - 0.5*Wc;
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
    %     d = Wc/2;
    %     vs = 1e-6;
    % end
    if vs == 0
        vs = 1e-6;
    end
end
end