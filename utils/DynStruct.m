function DynPar = DynStruct(varargin)
    % Function to Construct a Robot Dynamical Parameter Struct to be Used
    % in the Dynamics Calculation

    % Input Parsing
    Parser = inputParser;
    Parser.FunctionName = 'RobotStruct';
    addParameter(Parser, 'Mass'   , [], @(x)isnumeric(x) && isvector(x));
    addParameter(Parser, 'Length' , [], @(x)isnumeric(x) && isvector(x));
    addParameter(Parser, 'Radius' , [], @(x)isnumeric(x) && isvector(x));
    addParameter(Parser, 'Inertia', [], @(x)iscell(x) && all(cellfun(@(c)isequal(size(c), [3 3]), x)));
    addParameter(Parser, 'DH'     , [], @(s)isstruct(s) && all(isfield(s, {'alpha', 'a', 'd', 'theta'})));
    parse(Parser, varargin{:});
    R = Parser.Results;

    % Parameter List Length Validation
    required = {'Mass', 'Inertia', 'DH'};
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
    for k = 1:numel(optVecs)
        f = optVecs{k};
        if ~isempty(R.(f)) && numel(R.(f))~=nLinks
            error('RobotStruct:SizeMismatch', '%s must have %d elements.', f, nLinks);
        end
    end

    % ‌Build Struct
    DynPar = orderfields(struct( ...
        'Mass'   , R.Mass(:).', ...
        'Length' , R.Length(:).', ...
        'Radius' , R.Radius(:).', ...
        'Inertia', {R.Inertia}, ...
        'DH'     , R.DH ));

end
