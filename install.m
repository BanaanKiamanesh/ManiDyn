% INSTALL  ManiDyn installation script
% ---------------------------------------------------------
% This script sets up the ManiDyn toolbox for use in MATLAB.
%
% How to use (from ManiDyn root directory):
%   1) Temporary install (current session only):
%          >> install
%
%   2) Permanent install (adds paths & attempts to save):
%          >> install('save')
%
%   3) Remove ManiDyn paths from the current session:
%          >> install('reset')
%
% The script recursively adds all sub-folders under the ManiDyn root
% directory to the MATLAB search path, while automatically ignoring
% directories that are not required by end users (e.g. `.git`, `Trash`,
% auto-generated `codegen` folders).
%
% After installation you can test the toolbox by running the scripts in
% the `examples/` folder or the unit tests in `test/`.
%
% ---------------------------------------------------------
%  Author:  ManiDyn contributors
%  License: See LICENSE file in project root for details.
% ---------------------------------------------------------
function install(option)

    if nargin < 1
        option = "";            % default: just add paths for this session
    end

    % Locate ManiDyn root (directory where this file resides)
    rootDir = fileparts(mfilename("fullpath"));

    % Build a recursive list of all sub-directories
    allPaths = strsplit(genpath(rootDir), pathsep);

    % Exclusion patterns (folders to skip)
    skipPatterns = { [filesep '.git'], ...
        [filesep 'Trash'], ...
        [filesep 'codegen'] };

    % Filter out unwanted directories
    keepMask = true(1, numel(allPaths));
    for i = 1:numel(allPaths)
        p = allPaths{i};
        if isempty(p)
            keepMask(i) = false;              % ignore empty tokens
            continue;
        end
        for k = 1:numel(skipPatterns)
            if contains(p, skipPatterns{k})
                keepMask(i) = false;
                break;
            end
        end
    end
    userPaths = allPaths(keepMask);

    % Add selected paths to MATLAB path
    cellfun(@(p) addpath(p), userPaths);
    fprintf("[ManiDyn] Added %d folders to MATLAB path.\n", numel(userPaths));

    % Handle optional arguments
    switch lower(option)
        case 'save'
            % Attempt to persist the new path
            status = savepath;
            if status == 0
                fprintf('[ManiDyn] Path saved successfully. ManiDyn will be available in future MATLAB sessions.\n');
            else
                warning('[ManiDyn] Unable to save the MATLAB path automatically. You may need to run MATLAB with administrator privileges.');
            end
        case 'reset'
            % Remove paths added by this install
            cellfun(@(p) rmpath(p), userPaths);
            fprintf('[ManiDyn] ManiDyn paths removed from current session.\n');
    end

end % function install