function mrpt_setup(varargin)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MRPT_SETUP adds the MRPT toolbox to MATLAB path
%
% MRPT_SETUP('FOREVER') adds the MRPT toolbox permanently
% -------------------
% Authors: Jesus Briales, Jose-Luis Blanco-Claraco
% Copyright (C) 2014 Jesus Briales
% All rights reserved.
%
% For any bugs, please contact <jesusbriales@uma.es>
%
% This file is part of the MRPT library and is made available under
% the terms of the GNU license (see the COPYING file).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p=mfilename('fullpath');
rootDir=fileparts(p);

if ~isempty(rootDir)
    addpath(genpath(rootDir));
    disp('MRPT Toolbox has been successfully installed!');
end

if nargin>0
    if strcmpi('forever',varargin{1})
	savepath;
    end
end