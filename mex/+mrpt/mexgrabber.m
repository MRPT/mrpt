classdef mexgrabber < handle
    % mexgrabber Class for rawlog-grabber usage from Matlab.
    %
    % This class definition gives an interface to the underlying MEX functions
    % built in the private directory. It is a good practice to wrap MEX functions
    % with Matlab script so that the API is well documented and separated from
    % its C++ implementation. Also such a wrapper is a good place to validate
    % input arguments.
    %
    % See `make.m` for details.
    %
    
    properties %(Access = private)
        % Nothing yet
    end
    
    methods
        function this = mexgrabber(inifile)
            % mexgrabber Run rawlog-grabber application.
            if ~exist('inifile','var')
                error('inifile must be given');
            end
            assert(ischar(inifile));
            mexgrabber_('new', inifile);
        end
        
        function delete(~)
            % DELETE Destructor.
            mexgrabber_('delete');
        end
        
        function out = read(~)
            % READ Returns the observation
            out = mexgrabber_('read');
        end
        
        function plot(~)
            % PLOT
            % TODO yet
        end
    end
    
end