% x, y  The robot x and y coordinates
% Maze  2D array containing the MazeCell based on the current cell
% MazeOrder Current cell's number
% Wc    A cell width. It is assumed that the cells are square
function [d, s, vs] = ...
    centerLineDispNew(x, y, xDot, yDot, Maze, MazeOrder, Wc)
if isnan(x(1)) || isnan(y(1))
    d = NaN;
    s = NaN;
    vs = NaN;
else
    c = floor(x/Wc); % row
    r = floor(y/Wc); % column
    % [x, y];
    % [r, c];
    try
    cell = Maze(r+1, c+1);
    s = MazeOrder{r+1, c+1}(2); % Displacement thus far to get to the current cell
    if (cell == "R") || (cell == "L") || (cell == "U") || (cell == "D")
        switch cell
            case "R"
                d = (r+0.5)*Wc - y;
                s = s + (x - (r*Wc));
                vs = xDot;
            case "L"
                d = y - (r+0.5)*Wc;
                s = s + ((r+1)*Wc - x);
                vs = -xDot;
            case "U"
                d = x - (c+0.5)*Wc;
                s = s + (y - (c*Wc));
                vs = yDot;
            case "D"
                d = (c+0.5)*Wc - x;
                s = s + ((c+1)*Wc - y);
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
                s = s + 0.5*Wc*(pi/2 - atan((y-Y)/(x-X)));
            case "CNW"
                X = c*Wc;
                Y = (r+1)*Wc;
                re = sqrt((x-X)^2 + (y-Y)^2);
                d = 0.5*Wc - re;
                s = s - 0.5*Wc*atan((y-Y)/(x-X));
            case "CNE"
                X = (c+1)*Wc;
                Y = (r+1)*Wc;
                re = sqrt((x-X)^2 + (y-Y)^2);
                d = 0.5*Wc - re;
                s = s + 0.5*Wc*(pi/2 - atan((y-Y)/(x-X)));
            case "CSE"
                X = (c+1)*Wc;
                Y = r*Wc;
                re = sqrt((x-X)^2 + (y-Y)^2);
                d = 0.5*Wc - re;
                s = s - 0.5*Wc*atan((y-Y)/(x-X));
            case "ANW"
                X = c*Wc;
                Y = (r+1)*Wc;
                re = sqrt((x-X)^2 + (y-Y)^2);
                d = re - 0.5*Wc;
                s = s + 0.5*Wc*(pi/2 + atan((y-Y)/(x-X)));
            case "ANE"
                X = (c+1)*Wc;
                Y = (r+1)*Wc;
                re = sqrt((x-X)^2 + (y-Y)^2);
                d = re - 0.5*Wc;
                s = s + 0.5*Wc*atan((y-Y)/(x-X));
            case "ASE"
                X = (c+1)*Wc;
                Y = r*Wc;
                re = sqrt((x-X)^2 + (y-Y)^2);
                d = re - 0.5*Wc;
                s = s + 0.5*Wc*(pi/2 + atan((y-Y)/(x-X)));
            case "ASW"
                X = c*Wc;
                Y = r*Wc;
                re = sqrt((x-X)^2 + (y-Y)^2);
                d = re - 0.5*Wc;
                s = s + 0.5*Wc*atan((y-Y)/(x-X));
            otherwise
                disp('Error in centerLineDisplacement');
        end
            vs = (0.5*Wc)*((x-X)*yDot - (y-Y)*xDot)/((x-X)^2+(y-Y)^2);
            if (cell == "CSW") || (cell == "CNW") || (cell == "CNE") || (cell == "CSE")
                vs = -1*vs;
            end
    end
    catch
    % catch EXP
    %     disp([x, y]);
    %     disp([r, c]);
    %     rethrow(EXP)
        d = Wc/2;
        s = 0;
        vs = 0;
    end
end
end