% x, y  The robot x and y coordinates
% Maze  2D array containing the MazeCell based on the current cell
% MazeOrder Current cell's number
% Wc    A cell width. It is assumed that the cells are square
function [s, cellnr] = centerLineDisplacement(x, y, Maze, MazeOrder, Wc)
    c = floor(x/Wc); % row
    r = floor(y/Wc); % column
    [x, y];
    [r, c];
    cell = Maze(r+1, c+1);

    cellnr = MazeOrder(r+1, c+1);
    s = Wc * cellnr; % Displacement thus far to get to the current cell

    if (cell == MazeCell.R) || (cell == MazeCell.L) || (cell == MazeCell.U) || (cell == MazeCell.D)
        switch cell
            case MazeCell.R
                s = s + (x - (r*Wc));
            case MazeCell.L
                s = s + ((r+1)*Wc - x);
            case MazeCell.U
                s = s + (y - (c*Wc));
            case MazeCell.D
                s = s + ((c+1)*Wc - y);
            otherwise
                disp('Error in centerLineDisplacement');
        end
    else
        a = 0;
        b = 0;
        switch cell
            case MazeCell.CSW
                X = c*Wc;
                Y = r*Wc;
                a = x-X;
                b = y-Y;
            case MazeCell.CNW
                X = c*Wc;
                Y = (r+1)*Wc;
                a = Y-y;
                b = x-X;
            case MazeCell.CNE
                X = (c+1)*Wc;
                Y = (r+1)*Wc;
                a = X-x;
                b = Y-y;
            case MazeCell.CSE
                X = (c+1)*Wc;
                Y = r*Wc;
                a = y-Y;
                b = X-x;
            case MazeCell.ANW
                X = c*Wc;
                Y = (r+1)*Wc;
                a = x-X;
                b = Y-y;
            case MazeCell.ANE
                X = (c+1)*Wc;
                Y = (r+1)*Wc;
                a = Y-y;
                b = X-x;
            case MazeCell.ASE
                X = (c+1)*Wc;
                Y = r*Wc;
                a = X-x;
                b = y-Y;
            case MazeCell.ASW
                X = c*Wc;
                Y = r*Wc;
                a = y-Y;
                b = x-X;
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