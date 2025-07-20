function T = HomoTrans(DHRow, DHNotation)
    %HOMOTRANS Calculates the homogeneous transformation matrix from DH parameters.
    %   T = HOMOTRANS(DHROW, DHNOTATION) computes the 4x4 homogeneous
    %   transformation matrix for a single link of a manipulator.
    %
    %   Input Arguments:
    %       DHRow      - A 1-by-4 vector containing a single row of the DH table,
    %                    in the order [alpha, a, d, theta].
    %       DHNotation - A string specifying the DH convention, either 'original'
    %                    (default in other functions) or 'modified'. This determines
    %                    the order of transformations.
    %
    %   Output Arguments:
    %       T - The 4x4 homogeneous transformation matrix.
    %
    %   The function calculates T based on the specified notation:
    %   - 'original': T = Rot_z(theta) * Trans_z(d) * Trans_x(a) * Rot_x(alpha)
    %   - 'modified': T = Rot_x(alpha) * Trans_x(a) * Trans_z(d) * Rot_z(theta)
    %
    %   Example:
    %       % DH parameters for one link
    %       dh_row = [pi/2, 0.5, 0.1, pi/4];
    %       % Using original DH notation
    %       T_orig = HomoTrans(dh_row, 'original');
    %       % Using modified DH notation
    %       T_mod = HomoTrans(dh_row, 'modified');
    %
    %   See also: RotMatrix, TrnsMatrix, ParseDH.

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