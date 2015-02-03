function read2DScanner(inifile)
% readScanner(inifile)
% Input:
%   inifile - full path to the 2D scanner config file
% Demonstrates how the mrpt.mexgrabber application can be used
% to read data from a 2D LIDAR scanner
%
% Examples:
% - Read through Hokuyo UTM driver:
% readCam('/home/jesus/Libs/mrpt/source/share/mrpt/config_files/rawlog-grabber/hokuyo_UTM.ini')

try
    % Create object:
    Hok = mrpt.mexgrabber(inifile);
    figure('Name','Camera window');
    obs = [];
    while isempty(obs)
        % Ensure at least one image is collected
        pause(1)
        obs = Hok.read;
    end
    
    if any(obs{end}.map.z)
        warning('Z coordinate is not zero!');
    end
    h = plot(obs{end}.map.x,obs{end}.map.y,'.k');
    while 1
        obs = Hok.read;
        if numel(obs) > 0
            set( h, 'XData', obs{end}.map.x );
            set( h, 'YData', obs{end}.map.y );
        end
        pause(0.01);
    end
catch err
    fprintf('Catched exception, Hok object is being cleared and application safely closed:\n"%s"\n',err.message);
    disp(err.stack);
    clear Cam
end