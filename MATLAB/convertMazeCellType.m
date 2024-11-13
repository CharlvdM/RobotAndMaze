function MazeNumbers = convertMazeCellType(MazeStrings)
    [M, N] = size(MazeStrings);
    MazeNumbers = zeros(M,N);
    for i = 1:M
        for j = 1:N
            switch MazeStrings(i,j)
                case "R"
                    MazeNumbers(i,j) = 1;
                case "L"
                    MazeNumbers(i,j) = 2;
                case "U"
                    MazeNumbers(i,j) = 3;
                case "D"
                    MazeNumbers(i,j) = 4;
                case "CSW"
                    MazeNumbers(i,j) = 5;
                case "CNW"
                    MazeNumbers(i,j) = 6;
                case "CNE"
                    MazeNumbers(i,j) = 7;
                case "CSE"
                    MazeNumbers(i,j) = 8;
                case "ANW"
                    MazeNumbers(i,j) = 9;
                case "ANE"
                    MazeNumbers(i,j) = 10;
                case "ASE"
                    MazeNumbers(i,j) = 11;
                case "ASW"
                    MazeNumbers(i,j) = 12;
                otherwise
            end
        end
    end
end