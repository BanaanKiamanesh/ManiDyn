function [q, ErrHist] = IK_Newton(FKFun, JFun, q0, XDes, MaxIter, Tol, Damping)
    %IK_NEWTON Solves the inverse kinematics problem using Newton's method.
    %   [q, ErrHist] = IK_NEWTON(FKFun, JFun, q0, XDes) computes the joint
    %   angles `q` that achieve the desired end-effector pose `XDes`, starting
    %   from an initial guess `q0`.
    %
    %   This function uses an iterative Newton-Raphson approach, which tends
    %   to converge faster than gradient descent but requires the pseudoinverse
    %   of the Jacobian, making it more computationally intensive per iteration.
    %
    %   [q, ErrHist] = IK_NEWTON(..., MaxIter, Tol, Damping) allows specifying
    %   optional parameters for the solver.
    %
    %   Input Arguments:
    %       FKFun   - A function handle for the forward kinematics that takes a
    %                 joint vector q and returns the end-effector pose.
    %       JFun    - A function handle for the manipulator Jacobian that takes
    %                 a joint vector q and returns the Jacobian matrix.
    %       q0      - The initial guess for the joint angles (n-by-1 vector).
    %       XDes    - The desired end-effector pose (m-by-1 vector).
    %
    %   Optional Input Arguments:
    %       MaxIter - The maximum number of iterations. Default: 100.
    %       Tol     - The error tolerance for convergence. Default: 1e-6.
    %       Damping - Damping factor for the least-squares formulation. If set
    %                 to a value > 0, it uses the Damped Least Squares (DLS)
    %                 method, which can help avoid singularities. Default: 0.
    %
    %   Output Arguments:
    %       q       - The resulting joint angles (n-by-1 vector).
    %       ErrHist - A vector containing the history of the norm of the error
    %                 at each iteration.
    %
    %   Example:
    %       % Assume `fk` and `jacob` are function handles for FK and Jacobian
    %       q_initial = [0; 0];
    %       x_desired = [1.5; 0.5];
    %       [q_solution, err_hist] = IK_Newton(fk, jacob, q_initial, x_desired);
    %
    %   See also: IK_Gradient.

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
