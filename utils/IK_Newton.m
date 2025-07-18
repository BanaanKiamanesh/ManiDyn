function [q, ErrHist] = IK_Newton(FKFun, JFun, q0, XDes, MaxIter, Tol, Damping)
    % Function to Solve IK using Newtons Method

    if nargin < 5, MaxIter = 100;   end
    if nargin < 6, Tol     = 1e-6;  end
    if nargin < 7, Damping = 0;     end    % 0 -> Newton / 1 -> else Damped LS

    q       = q0;
    ErrHist = zeros(MaxIter,1);

    for k = 1:MaxIter
        e   = XDes - FKFun(q);
        Err = norm(e);
        ErrHist(k) = Err;
        if Err < Tol
            ErrHist = ErrHist(1:k); return
        end

        J  = JFun(q);
        JT = J.';
        m  = size(J,1);
        if Damping == 0
            delta = pinv(J) * e;
        else
            delta = (JT / (J*JT + Damping^2*eye(m))) * e;
        end
        q = q + delta;
    end

    warning('IK_Newton:NoConvergence', ...
        'Tolerance not reached in %d iterations (final err = %.3e).', ...
        MaxIter, Err);
end
