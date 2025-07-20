function Eul = Rot2Eul(R)
    %ROT2EUL Converts a ZYX rotation matrix to Yaw, Pitch, Roll Euler angles.
    %   Eul = ROT2EUL(R) calculates the ZYX Euler angles (Yaw, Pitch, Roll)
    %   from a 3x3 rotation matrix R.
    %
    %   The conversion assumes a rotation order of Z (Yaw), then Y (Pitch),
    %   then X (Roll). The function handles both numeric and symbolic matrices.
    %   It also correctly handles the gimbal lock singularity.
    %
    %   Input Arguments:
    %       R - A 3x3 rotation matrix (numeric or symbolic).
    %
    %   Output Arguments:
    %       Eul - A 3x1 vector containing the Euler angles [Yaw; Pitch; Roll].
    %             - Yaw:   Rotation about the Z-axis.
    %             - Pitch: Rotation about the new Y-axis.
    %             - Roll:  Rotation about the new X-axis.
    %
    %   Example:
    %       % Create a rotation matrix
    %       R = eul2rotm([0.1, 0.2, 0.3], 'ZYX'); % Note: MATLAB's eul2rotm
    %       % Convert back to Euler angles
    %       eul_angles = Rot2Eul(R);
    %
    %   Throws:
    %       Rot2Eul:BadInput - If the input is not a 3x3 matrix.
    %
    %   See also: RotMatrix.

    % Function to Calculate Yaw, Pitch, Roll from ZYX Rotation Matrix

    if ~(isequal(size(R), [3, 3]))
        error('Rot2Eul:BadInput', ...
            'Input must be a 3×3 rotation matrix.');
    end

    if ~isa(R, 'sym')
        Pitch = -asin(R(3, 1));
        cp    = cos(Pitch);

        if abs(cp) > 1e-8          % Regular Case
            Yaw  = atan2(R(2, 1), R(1, 1));
            Roll = atan2(R(3, 2), R(3, 3));
        else                       % Gimbal Lock
            Roll = 0;
            if Pitch < 0  % Pitch ~ +pi/2
                Yaw = atan2(-R(1, 2),  R(2, 2));
            else          % Pitch ~ -pi/2
                Yaw = atan2( R(1, 2), -R(2, 2));
            end
        end

    else
        % Direct Analytic Expressions
        Yaw   = atan2(R(2, 1), R(1, 1));
        Pitch = -asin(R(3, 1));
        Roll  = atan2(R(3, 2), R(3, 3));
    end

    Eul = [Yaw; Pitch; Roll];
end
