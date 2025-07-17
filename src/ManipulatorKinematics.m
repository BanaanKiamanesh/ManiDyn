classdef ManipulatorKinematics
    properties
        DH
    end

    methods
        function obj = ManipulatorKinematics(DH)
            arguments
                DH (1, 1) struct
            end

            % Check DH
            dhReq = {'alpha', 'a', 'd', 'theta', 'type'};
            if ~all(isfield(DH, dhReq))
                dhMissing = dhReq(~isfield(DH, dhReq));
                error('ManipulatorKinematics:BadDH', ...
                    'DH struct missing field(s): %s', ...
                    strjoin(dhMissing, ', '));
            end

            % Check DH Elements
            nLinks = numel(DH.alpha);
            if any([numel(DH.a), numel(DH.d), numel(DH.theta), ...
                    numel(DH.type)] ~= nLinks)
                error('ManipulatorKinematics:SizeMismatch', ...
                    'Each DH vector must have %d elements.', nLinks);
            end

            % Store, if Everything Looks Good
            obj.DH = orderfields(DH);
        end
    end
end
