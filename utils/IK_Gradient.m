function [q, ErrHist] = IK_Gradient(FKFun, JFun, q0, XDes, MaxIter, Tol, Alpha)
    %IK_GRADIENT Solves the inverse kinematics problem using gradient descent.
    %   [q, ErrHist] = IK_GRADIENT(FKFun, JFun, q0, XDes) computes the joint
    %   angles `q` that achieve the desired end-effector pose `XDes`, starting
    %   from an initial guess `q0`.
    %
    %   This function iteratively adjusts the joint angles to minimize the error
    %   between the current pose and the desired pose, using the gradient of
    %   the error, which is related to the manipulator Jacobian.
    %
    %   [q, ErrHist] = IK_GRADIENT(..., MaxIter, Tol, Alpha) allows specifying
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
    %       MaxIter - The maximum number of iterations. Default: 300.
    %       Tol     - The error tolerance for convergence. Default: 1e-6.
    %       Alpha   - The step size (learning rate) for the gradient descent.
    %                 Default: 0.05.
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
    %       [q_solution, error_history] = IK_Gradient(fk, jacob, q_initial, x_desired);
    %
    %   See also: IK_Newton.

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
