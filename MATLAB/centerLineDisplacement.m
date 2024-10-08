% x, y  The robot x and y coordinates
% Maze  2D array containing the MazeCell based on the current cell
% MazeOrder Current cell's number
% Wc    A cell width. It is assumed that the cells are square
function [s, cellnr, d] = centerLineDisplacement(x, y, Maze, MazeOrder, Wc)
if isnan(x(1)) || isnan(y(1))
    s = NaN;
    cellnr = NaN;
    d = NaN;
else
    c = floor(x/Wc); % row
    r = floor(y/Wc); % column
    % [x, y];
    % [r, c];
    try
    cell = Maze(r+1, c+1);
    catch
    % catch EXP
        % disp([x, y]);
        % disp([r, c]);
        % rethrow(EXP)
        d = Wc/2;
    end

    cellnr = MazeOrder(r+1, c+1);
    s = Wc * cellnr; % Displacement thus far to get to the current cell

    if (cell == "R") || (cell == "L") || (cell == "U") || (cell == "D")
        switch cell
            case "R"
                s = s + (x - (r*Wc));
                d = (r+0.5)*Wc - y;
            case "L"
                s = s + ((r+1)*Wc - x);
                d = y - (r+0.5)*Wc;
            case "U"
                s = s + (y - (c*Wc));
                d = x - (c+0.5)*Wc;
            case "D"
                s = s + ((c+1)*Wc - y);
                d = (c+0.5)*Wc - x;
            otherwise
                disp('Error in centerLineDisplacement');
        end
    else
        a = 0;
        b = 0;
        switch cell
            case "CSW"
                X = c*Wc;
                Y = r*Wc;
                a = x-X;
                b = y-Y;
                re = sqrt((x-X)^2 + (y-Y)^2);
                d = 0.5*Wc - re;
            case "CNW"
                X = c*Wc;
                Y = (r+1)*Wc;
                a = Y-y;
                b = x-X;
                re = sqrt((x-X)^2 + (y-Y)^2);
                d = 0.5*Wc - re;
            case "CNE"
                X = (c+1)*Wc;
                Y = (r+1)*Wc;
                a = X-x;
                b = Y-y;
                re = sqrt((x-X)^2 + (y-Y)^2);
                d = 0.5*Wc - re;
            case "CSE"
                X = (c+1)*Wc;
                Y = r*Wc;
                a = y-Y;
                b = X-x;
                re = sqrt((x-X)^2 + (y-Y)^2);
                d = 0.5*Wc - re;
            case "ANW"
                X = c*Wc;
                Y = (r+1)*Wc;
                a = x-X;
                b = Y-y;
                re = sqrt((x-X)^2 + (y-Y)^2);
                d = re - 0.5*Wc;
            case "ANE"
                X = (c+1)*Wc;
                Y = (r+1)*Wc;
                a = Y-y;
                b = X-x;
                re = sqrt((x-X)^2 + (y-Y)^2);
                d = re - 0.5*Wc;
            case "ASE"
                X = (c+1)*Wc;
                Y = r*Wc;
                a = X-x;
                b = y-Y;
                re = sqrt((x-X)^2 + (y-Y)^2);
                d = re - 0.5*Wc;
            case "ASW"
                X = c*Wc;
                Y = r*Wc;
                a = y-Y;
                b = x-X;
                re = sqrt((x-X)^2 + (y-Y)^2);
                d = re - 0.5*Wc;
            otherwise
                disp('Error in centerLineDisplacement');
        end
        if a < b
            s = s + Wc*(0.5*a/b);
        else
            s = s + Wc*(1-(0.5*b/a));
        end
    end
end
end