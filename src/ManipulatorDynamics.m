classdef ManipulatorDynamics
    properties
        Par
    end

    methods
        function obj = ManipulatorDynamics(DynPar)
            arguments
                DynPar (1, 1) struct
            end

            % Fields Check
            req = {'Mass', 'Inertia', 'DH'};
            missing = req(~isfield(DynPar, req));
            if ~isempty(missing)
                error('ManipulatorDynamics:MissingField', ...
                    'DynPar is missing required field(s): %s', ...
                    strjoin(missing, ', '));
            end

            % Link Count
            nLinks = numel(DynPar.Mass);

            % Check Inertia
            if ~iscell(DynPar.Inertia) || numel(DynPar.Inertia) ~= nLinks
                error('ManipulatorDynamics:SizeMismatch', ...
                    'Inertia must be a %d-element cell array.', nLinks);
            end

            % Check Inertia Matrix Sizes
            if ~all(cellfun(@(c)isequal(size(c), [3 3]), DynPar.Inertia))
                error('ManipulatorDynamics:BadInertia', ...
                    'Every inertia tensor must be a 3×3 matrix.');
            end

            % Check DH
            dhReq = {'alpha', 'a', 'd', 'theta', 'type'};
            if ~isstruct(DynPar.DH) || ~all(isfield(DynPar.DH, dhReq))
                dhMissing = dhReq(~isfield(DynPar.DH, dhReq));
                error('ManipulatorDynamics:BadDH', ...
                    'DH struct missing field(s): %s', ...
                    strjoin(dhMissing, ', '));
            end

            % Check DH Elements
            if any([numel(DynPar.DH.alpha), numel(DynPar.DH.a), ...
                    numel(DynPar.DH.d), numel(DynPar.DH.theta), ...
                    numel(DynPar.DH.type)] ~= nLinks)
                error('ManipulatorDynamics:SizeMismatch', ...
                    'Each DH vector must have %d elements.', nLinks);
            end

            % Check Length and Radius
            opt = {'Length', 'Radius'};
            for k = 1:numel(opt)
                if isfield(DynPar, opt{k}) && ~isempty(DynPar.(opt{k})) && ...
                        numel(DynPar.(opt{k})) ~= nLinks
                    error('ManipulatorDynamics:SizeMismatch', ...
                        '%s must have %d elements.', opt{k}, nLinks);
                end
            end

            % Store, if Everything Looks Good
            obj.Par = orderfields(DynPar);
        end

        function CalculateFK(obj)
            
        end
    end
end
