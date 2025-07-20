function trnsMat = TrnsMatrix(length, axis)
    %TRNSMATRIX Generates a 4x4 homogeneous translation matrix.
    %   T = TRNSMATRIX(LENGTH, AXIS) creates a 4x4 homogeneous transformation
    %   matrix representing a pure translation of LENGTH along the specified AXIS
    %   ('x', 'y', or 'z').
    %
    %   Input Arguments:
    %       length - The distance to translate.
    %       axis   - The axis of translation, must be 'x', 'y', or 'z'.
    %
    %   Output Arguments:
    %       trnsMat - The resulting 4x4 homogeneous translation matrix.
    %
    %   Example:
    %       % A translation of 5 units along the y-axis
    %       Ty = TrnsMatrix(5, 'y');
    %
    %   See also: RotMatrix, HomoTrans.

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