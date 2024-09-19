function [xLeft, xRight, yBottom, yTop] = ...
    calcCollisionDistances(x, y, xCollisonMatrix, yCollisonMatrix)

xLeft = zeros(size(x));
xRight = zeros(size(x));
yBottom = zeros(size(y));
yTop = zeros(size(y));

N = size(x,1); % Number of collocation points

for i = 1:N
    
    matRow = floor(y(i));
    matCol = floor(x(i));
    
    xLeftSearch = matCol;
    while xLeftSearch >= 0
        if xCollisonMatrix(matRow+1, xLeftSearch+1)
            xLeft(i) = x(i) - xLeftSearch;
            break;
        end
        xLeftSearch = xLeftSearch - 1;
    end
    
    xRightSearch = matCol;
    xMatrixWidth = size(xCollisonMatrix, 2);
    while xRightSearch < xMatrixWidth
        xRightSearch = xRightSearch + 1;
        if xRightSearch >= xMatrixWidth
            xRight(i) = xMatrixWidth - x(i);
        elseif xCollisonMatrix(matRow+1, xRightSearch+1)
            xRight(i) = xRightSearch - x(i);
            break;
        end
    end
    
    yBottomSearch = matRow;
    while yBottomSearch >= 0
        matColPlusOne = matCol+1
        if yCollisonMatrix(yBottomSearch+1, matCol+1)
            yBottom(i) = y(i) - yBottomSearch;
            break;
        end
        yBottomSearch = yBottomSearch - 1;
    end
    
    yTopSearch = matRow;
    yMatrixLength = size(yCollisonMatrix, 1);
    while yTopSearch < yMatrixLength
        yTopSearch = yTopSearch + 1;
        if yTopSearch >= yMatrixLength
            yTop(i) = yMatrixLength - y(i);
        elseif yCollisonMatrix(yTopSearch+1, matCol+1)
            yTop(i) = yTopSearch - y(i);
            break;
        end
    end
end

end