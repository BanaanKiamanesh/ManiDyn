function [R, P] = ParseDH(AlphaList, AList, DList, ThetaList)

    nParams = cellfun(@numel, {AlphaList, AList, DList, ThetaList});

    if ~all(nParams == nParams(1))
        msg = sprintf(['All DH parameter lists must have the same length, ' ...
            'but got lengths [α a d θ] = [%d %d %d %d].'], nParams);
        error('ParseDH:InputLengthMismatch', msg);
    end
    
    N = nParams(1);           % number of joints/links

    % Mem Alloc
    R = cell(1, N);           % Rotation Matrices
    P = cell(1, N);           % Pos Vectors

    TCurr = eye(4);           % Base-Frame Transform

    for i = 1:N
        DHRow = [AlphaList(i), AList(i), DList(i), ThetaList(i)];
        T = HomoTrans(DHRow);

        TCurr = TCurr * T;

        % Store rotation and position w.r.t. base
        R{i} = TCurr(1:3, 1:3);
        P{i} = TCurr(1:3, 4);
    end
end
