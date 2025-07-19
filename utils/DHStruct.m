function DH = DHStruct(varargin)
    % Function to Construct a DH Parameter Struct to be Used Later

    % Input Parsing
    Parser = inputParser;
    Parser.FunctionName = 'DHStruct';
    addParameter(Parser, 'alpha'   ,         [], @(x)isnumeric(x) && isvector(x));
    addParameter(Parser, 'a'       ,         [], @(x)isnumeric(x) && isvector(x));
    addParameter(Parser, 'd'       ,         [], @(x)isnumeric(x) && isvector(x));
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
