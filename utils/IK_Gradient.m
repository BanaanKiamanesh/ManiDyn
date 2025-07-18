function [q, ErrHist] = IK_Gradient(FKFun, JFun, q0, XDes, MaxIter, Tol, Alpha)
    % Function to Solve IK using Gradient Descent Method

    if nargin < 5, MaxIter = 300;  end
    if nargin < 6, Tol     = 1e-6; end
    if nargin < 7, Alpha   = 0.05; end

    q       = q0;
    ErrHist = zeros(MaxIter,1);

    for k = 1:MaxIter
        e   = XDes - FKFun(q);
        Err = norm(e);
        ErrHist(k) = Err;

        if Err < Tol
            ErrHist = ErrHist(1:k);
            return
        end

        J = JFun(q);
        q = q + Alpha * (J.' * e);
    end

    warning('IK_Gradient:NoConvergence', ...
        'Tolerance not reached in %d iterations (final err = %.3e).', ...
        MaxIter, Err);
end
