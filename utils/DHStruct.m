function DH = DHStruct(varargin)
    %DHSTRUCT Creates a standardized Denavit-Hartenberg (DH) parameter structure.
    %   DH = DHSTRUCT('alpha', ALPHA, 'a', A, 'd', D, 'theta', THETA, 'type', TYPE)
    %   creates a structure `DH` containing the manipulator's DH parameters.
    %   All input vectors must have the same length.
    %
    %   DH = DHSTRUCT(..., 'notation', NOTATION) specifies the DH notation used.
    %   Can be 'original' (default) or 'modified'.
    %
    %   Input Arguments:
    %       'alpha' - A 1-by-n vector of link twist angles.
    %       'a'     - A 1-by-n vector of link lengths.
    %       'd'     - A 1-by-n vector of link offsets.
    %       'theta' - A 1-by-n vector of joint angles.
    %       'type'  - A 1-by-n character array or string specifying the joint type for
    %                 each link. 'r' for revolute, 'p' for prismatic, 'f' for fixed.
    %
    %   Name-Value Pair Arguments:
    %       'notation' - The DH notation convention to be used.
    %                    'original' (default): Standard DH convention.
    %                    'modified': Modified DH convention (e.g., as in Craig's textbook).
    %
    %   Output Arguments:
    %       DH - A structure with the following fields:
    %            .alpha    - (1xn double)
    %            .a        - (1xn double)
    %            .d        - (1xn double)
    %            .theta    - (1xn double)
    %            .type     - (1xn char)
    %            .notation - (char)
    %
    %   Example:
    %       % For a 2-DOF planar robot
    %       DH = DHStruct('alpha', [0, 0], 'a', [1, 1], 'd', [0, 0], ...
    %                     'theta', [0, 0], 'type', 'rr');
    %
    %   Throws:
    %       DHStruct:SizeMismatch - If input vectors have different lengths.
    %       DHStruct:MissingField - If any of the required DH parameters are not provided.
    %       DHStruct:BadType      - If 'type' contains invalid characters.
    %       DHStruct:BadNotation  - If 'notation' is not 'original' or 'modified'.
    %
    %   See also: ManipulatorKinematics, ManipulatorDynamics, ParseDH.

    % Function to Construct a DH Parameter Struct to be Used Later

    % Input Parsing
    Parser = inputParser;
    Parser.FunctionName = 'DHStruct';
    addParameter(Parser, 'alpha'   ,         [], @(x)isnumeric(x) && isvector(x));
    addParameter(Parser, 'a'       ,         [], @(x)isvector(x));
    addParameter(Parser, 'd'       ,         [], @(x)isvector(x));
    addParameter(Parser, 'theta'   ,         [], @(x)isnumeric(x) && isvector(x));
    addParameter(Parser, 'type'    ,         [], @(x)(ischar(x) || isstring(x)) && isvector(x));
    addParameter(Parser, 'notation', 'original', @(x)(ischar(x) || isstring(x)) && isvector(x));  % ◀ default added
    parse(Parser, varargin{:});
    R = Parser.Results;


    % DH List Length Validation
    lengths = cellfun(@numel, {R.alpha, R.a, R.d, R.theta, R.type});
    filled  = lengths > 0;

    if ~all(lengths(filled) == lengths(find(filled, 1)))
        error('DHStruct:SizeMismatch', 'Provided DH arrays are not the same length.');
    end

    % Require a Complete Set
    if any(~filled)
        fn      = fieldnames(R);
        missing = strjoin(fn(~filled), ', ');
        error('DHStruct:MissingField', ['Missing DH entries: ', missing]);
    end

    % Joint-Type Sanity Check: Only {'r', 'p', 'f'}
    if ~all(ismember(lower(char(R.type)), ['r', 'p', 'f']))
        error('DHStruct:BadType', 'Joint "type" must be ''r''(revolute) or ''p''(prismatic) or ''f''(fixed) for every link.');
    end

    % DH Notation Sanity Check: Only {'original', 'modified'}
    if ~any(strcmpi(char(R.notation), {'original', 'modified'}))
        error('DHStruct:BadNotation', 'DH "notation" must be ''original''(default) or ''modified''.');
    end

    % ‌Build Struct
    DH = struct('alpha'   , R.alpha(:)'                     , ...
        'a'       , R.a(:)'                         , ...
        'd'       , R.d(:)'                         , ...
        'theta'   , R.theta(:)'                     , ...
        'type'    , squeeze(lower(char(R.type(:)'))), ...
        'notation', squeeze(lower(char(R.notation(:)'))));
end
