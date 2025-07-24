function [R, P] = ParseDH(DHTable)
    %PARSEDH Computes rotation matrices and position vectors for each frame.
    %   [R, P] = PARSEDH(DHTable) processes a Denavit-Hartenberg (DH) parameter
    %   table to calculate the orientation and position of each link's frame
    %   relative to the base frame.
    %
    %   This function iterates through the links, computes the homogeneous
    %   transformation for each, and accumulates the transformations to find the
    %   pose of each frame.
    %
    %   Input Arguments:
    %       DHTable - A structure containing the DH parameters, typically created
    %                 by `DHStruct`. It must contain the following fields:
    %                 .alpha    - Vector of link twist angles.
    %                 .a        - Vector of link lengths.
    %                 .d        - Vector of link offsets.
    %                 .theta    - Vector of joint angles.
    %                 .notation - String specifying 'original' or 'modified' DH
    %                             convention.
    %
    %   Output Arguments:
    %       R - A 1-by-n cell array where each cell `R{i}` contains the 3x3
    %           rotation matrix of frame {i} with respect to the base frame {0}.
    %       P - A 1-by-n cell array where each cell `P{i}` contains the 3x1
    %           position vector of the origin of frame {i} with respect to the
    %           base frame {0}.
    %
    %   Example:
    %       % DH parameters for a 2-DOF planar robot
    %       dh = DHStruct('alpha', [0,0], 'a', [1,1], 'd', [0,0], ...
    %                     'theta', [pi/4, pi/4], 'type', 'rr');
    %       [R, P] = ParseDH(dh);
    %       % R{2} is the orientation of the end-effector
    %       % P{2} is the position of the end-effector
    %
    %   See also: HomoTrans, DHStruct.

    % Function to Compute the Rotation Matrix and Position Vectors for Each Frame

    AlphaList = DHTable.alpha;
    AList     = DHTable.a;
    DList     = DHTable.d;
    ThetaList = DHTable.theta;
    Notation  = DHTable.notation;

    N = numel(AList);           % Number of Links

    % Mem Alloc
    R = cell(1, N);           % Rotation Matrices
    P = cell(1, N);           % Pos Vectors

    TCurr = eye(4);           % Base-Frame Transform

    for i = 1:N
        DHRow = [AlphaList(i), AList(i), DList(i), ThetaList(i)];
        T = HomoTrans(DHRow, Notation);

        TCurr = TCurr * T;

        % Store Rotation and Pos w.r.t. base
        R{i} = simplify(TCurr(1:3, 1:3));
        P{i} = simplify(TCurr(1:3, 4));
    end
end
