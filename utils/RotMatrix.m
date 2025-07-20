function rotMat = RotMatrix(theta, axis, varargin)
    %ROTMATRIX Generates a 4x4 homogeneous rotation matrix.
    %   R = ROTMATRIX(THETA, AXIS) creates a 4x4 homogeneous transformation
    %   matrix for a rotation of THETA radians about the specified AXIS ('x',
    %   'y', or 'z').
    %
    %   R = ROTMATRIX(K, 'k', V) computes the rotation about an arbitrary axis V,
    %   where V is a 3x1 vector and K is the angle of rotation.
    %
    %   R = ROTMATRIX(ANGLES, 'rpy') computes the rotation matrix for given
    %   Roll-Pitch-Yaw angles. ANGLES should be a 3-element vector [Phi, Theta, Psi].
    %   The rotation order is Z (Phi), Y (Theta), X (Psi).
    %
    %   R = ROTMATRIX(ANGLES, 'euler') computes the rotation matrix for ZYZ Euler
    %   angles. ANGLES should be a 3-element vector [Phi, Theta, Psi]. The
    %   rotation order is Z (Phi), Y (Theta), Z (Psi).
    %
    %   Input Arguments:
    %       theta - The angle of rotation in radians. For 'rpy' and 'euler', this
    %               is a 3-element vector of angles.
    %       axis  - The axis of rotation. Can be 'x', 'y', 'z', 'k' (arbitrary
    %               axis), 'rpy', or 'euler'.
    %
    %   Optional Input Arguments:
    %       varargin - For 'k' axis, this should be the 3x1 vector representing
    %                  the arbitrary axis of rotation.
    %
    %   Output Arguments:
    %       rotMat - The resulting 4x4 homogeneous rotation matrix.
    %
    %   Examples:
    %       % 90-degree rotation about the x-axis
    %       Rx = RotMatrix(pi/2, 'x');
    %
    %       % Rotation about an arbitrary axis [1;1;1]
    %       Rk = RotMatrix(pi/4, 'k', [1;1;1]/sqrt(3));
    %
    %       % Rotation from Roll-Pitch-Yaw angles
    %       Rrpy = RotMatrix([0.1, 0.2, 0.3], 'rpy');
    %
    %   See also: TrnsMatrix, HomoTrans, Rot2Eul.

    % RotMatrix returns the 4*4 rotation matrix in given axis and angle.

    switch axis
        % along the x axis
        case 'x'
            rotMat = [1       0            0         0
                0   cos(theta)   -sin(theta)   0
                0   sin(theta)    cos(theta)   0
                0       0             0        1];

            % along the y axis
        case 'y'
            rotMat = [cos(theta)   0   sin(theta)   0
                0            1       0        0
                -sin(theta)  0   cos(theta)   0
                0            0       0        1];

            % along the z axis
        case 'z'
            rotMat = [cos(theta)   -sin(theta)    0    0
                sin(theta)   cos(theta)     0    0
                0                 0         1    0
                0                 0         0    1];

            % along the arbitrary axis k that is a 3*1 vector
        case 'k'
            kx = varargin{1}(1);
            ky = varargin{1}(2);
            kz = varargin{1}(3);

            alpha = atan2(kx/sqrt(kx^2 + ky^2), ky/sqrt(kx^2 + ky^2));
            beta  = atan2(kz, sqrt(kx^2 + ky^2));

            rotMat = RotMatrix(alpha, 'z') * RotMatrix(beta, 'y') * RotMatrix(theta, 'z') * RotMatrix(-beta, 'y') * RotMatrix(-alpha, 'z');

            % compute the rotation matrix with given roll-pitch-yaw angles
        case 'rpy'

            Phi   = theta(1);
            Theta = theta(2);
            Psi   = theta(3);

            rotMat = RotMatrix(Phi, 'z') * RotMatrix(Theta, 'y') * RotMatrix(Psi, 'x');

            % compute the rotation matrix with given euler angles
        case 'euler'

            Phi   = theta(1);
            Theta = theta(2);
            Psi   = theta(3);

            rotMat = RotMatrix(Phi, 'z') * RotMatrix(Theta, 'y') * RotMatrix(Psi, 'z');

    end
end