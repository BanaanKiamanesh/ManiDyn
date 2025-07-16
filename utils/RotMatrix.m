function rotMat = RotMatrix(theta, axis, varargin)
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