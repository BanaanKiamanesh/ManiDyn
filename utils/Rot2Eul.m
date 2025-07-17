function Eul = Rot2Eul(R)
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
