function [R, P] = ParseDH(DHTable)
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
        R{i} = TCurr(1:3, 1:3);
        P{i} = TCurr(1:3, 4);
    end
end
