function trnsMat = TrnsMatrix(length, axis)
% TrnsMatrix returns the 4*4 translation matrix in given axis and length.

    switch axis
        % along the x axis
        case 'x'
            trnsMat = [1   0   0 length
                       0   1   0   0
                       0   0   1   0
                       0   0   0   1];
        % along the y axis
        case 'y'
            trnsMat = [1   0   0     0
                       0   1   0   length
                       0   0   1     0
                       0   0   0     1];

        % along the z axis
        case 'z'
            trnsMat = [1   0   0     0
                       0   1   0     0
                       0   0   1   length
                       0   0   0    1];
    end
end