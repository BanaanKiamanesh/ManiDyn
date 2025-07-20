function DynPar = DynStruct(varargin)
    %DYNSTRUCT Creates a standardized structure for manipulator dynamic parameters.
    %   DynPar = DYNSTRUCT('Mass', M, 'Inertia', I, 'COM', C, 'DH', DH_STRUCT)
    %   creates a structure `DynPar` containing the parameters required for
    %   dynamic analysis.
    %
    %   Input Arguments:
    %       'Mass'    - A 1-by-n vector of the mass of each link.
    %       'Inertia' - A 1-by-n cell array, where each cell contains the 3x3
    %                   inertia tensor of a link with respect to its own frame.
    %       'COM'     - An n-by-3 matrix where each row specifies the center of
    %                   mass [x, y, z] for the corresponding link in its own frame.
    %       'DH'      - A DH parameter structure, typically created by `DHStruct`.
    %
    %   Name-Value Pair Arguments:
    %       'Length'  - (Optional) A 1-by-n vector of link lengths.
    %       'Radius'  - (Optional) A 1-by-n vector of link radii.
    %       'Fv'      - (Optional) A 1-by-n vector of viscous friction coefficients (N·m·s/rad).
    %       'Fc'      - (Optional) A 1-by-n vector of Coulomb friction coefficients (N·m).
    %
    %   Output Arguments:
    %       DynPar - A structure containing the following fields:
    %                .Mass    - (1xn double)
    %                .Inertia - (1xn cell of 3x3 double)
    %                .COM     - (nx3 double)
    %                .DH      - (struct)
    %                .Length  - (1xn double, optional)
    %                .Radius  - (1xn double, optional)
    %                .Fv      - (1xn double, optional)
    %                .Fc      - (1xn double, optional)
    %
    %   Example:
    %       % Define parameters for a single link
    %       mass = 1;
    %       inertia = eye(3);
    %       com = [0.5, 0, 0];
    %       dh = DHStruct('alpha',0,'a',1,'d',0,'theta',0,'type','r');
    %
    %       % Create the dynamics parameter structure
    %       dyn_params = DynStruct('Mass', mass, 'Inertia', {inertia}, ...
    %                              'COM', com, 'DH', dh);
    %
    %   Throws:
    %       RobotStruct:MissingField - If a required field (Mass, Inertia, COM, DH)
    %                                  is missing.
    %       RobotStruct:SizeMismatch - If input arrays have inconsistent sizes.
    %
    %   See also: ManipulatorDynamics, DHStruct.

    % Function to Construct a Robot Dynamical Parameter Struct to be Used
    % in the Dynamics Calculation

    % Input Parsing
    Parser = inputParser;
    Parser.FunctionName = 'RobotStruct';
    addParameter(Parser, 'Mass'   , [], @(x)isnumeric(x) && isvector(x));
    addParameter(Parser, 'Length' , [], @(x)isnumeric(x) && isvector(x));
    addParameter(Parser, 'Radius' , [], @(x)isnumeric(x) && isvector(x));
    addParameter(Parser, 'Fv'    , [], @(x)isnumeric(x) && isvector(x));
    addParameter(Parser, 'Fc'    , [], @(x)isnumeric(x) && isvector(x));
    addParameter(Parser, 'Inertia', [], @(x)iscell(x) && all(cellfun(@(c)isequal(size(c), [3 3]), x)));
    addParameter(Parser, 'COM'    , [], @(x)isnumeric(x) && size(x,2) == 3);
    addParameter(Parser, 'DH'     , [], @(s)isstruct(s) && all(isfield(s, {'alpha', 'a', 'd', 'theta'})));
    parse(Parser, varargin{:});
    R = Parser.Results;

    % Parameter List Length Validation
    required = {'Mass', 'Inertia', 'DH', 'COM'};
    for k = 1:numel(required)
        if isempty(R.(required{k}))
            error('RobotStruct:MissingField', 'Missing required input ''%s''.', required{k});
        end
    end

    nLinks = numel(R.Mass);
    if numel(R.Inertia) ~= nLinks
        error('RobotStruct:SizeMismatch', 'Inertia cell array must have %d elements.', nLinks);
    end
    if any([numel(R.DH.alpha), numel(R.DH.a), numel(R.DH.d), numel(R.DH.theta)] ~= nLinks)
        error('RobotStruct:SizeMismatch', 'DH lists must all have %d elements.', nLinks);
    end

    optVecs = {'Length', 'Radius'};
    optVecs = [optVecs, {'Fv', 'Fc'}];
    for k = 1:numel(optVecs)
        f = optVecs{k};
        if ~isempty(R.(f)) && numel(R.(f)) ~= nLinks
            error('RobotStruct:SizeMismatch', '%s must have %d elements.', f, nLinks);
        end
    end

    % Default zero friction if not provided
    if isempty(R.Fv)
        R.Fv = zeros(1, nLinks);
    end
    if isempty(R.Fc)
        R.Fc = zeros(1, nLinks);
    end

    % Verify COM Size (n×3)
    if size(R.COM,1) ~= nLinks
        error('RobotStruct:SizeMismatch', 'COM must have %d rows.', nLinks);
    end

    % ‌Build Struct
    DynPar = orderfields(struct( ...
        'Mass'   , R.Mass(:).', ...
        'Length' , R.Length(:).', ...
        'Radius' , R.Radius(:).', ...
        'Fv'     , R.Fv(:).', ...
        'Fc'     , R.Fc(:).', ...
        'Inertia', {R.Inertia}, ...
        'COM'    , R.COM, ...
        'DH'     , R.DH ));
end
