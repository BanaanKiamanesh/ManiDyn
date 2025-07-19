function T = HomoTrans(DHRow, DHNotation)
    % Function to Calculate a Homogeneous Transform from a Given DH Table Row

    alpha = DHRow(1);
    a     = DHRow(2);
    d     = DHRow(3);
    theta = DHRow(4);

    % Calculate the HomoTrans Matrix Regarding DH Notation
    if strcmpi(DHNotation, 'modified')
        % Modified Case
        T = RotMatrix(alpha, 'x') * TrnsMatrix(a, 'x') * TrnsMatrix(d, 'z') * RotMatrix(theta, 'z');
    else
        % Original Case
        T = RotMatrix(theta, 'z') * TrnsMatrix(d, 'z') * TrnsMatrix(a, 'x') * RotMatrix(alpha, 'x');
    end    
end