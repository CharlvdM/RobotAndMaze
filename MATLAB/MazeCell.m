classdef MazeCell
    enumeration
         % Corner cells. Clockwise (C), anti-clockwise (A), with the corner
         % specified, e.g. North West (NW)
        CSW, CNW, CNE, CSE, ANW, ANE, ASE, ASW
        % Straight cells. Right, left, up, down
        R, L, U, D
    end
end