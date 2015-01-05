function readCam(inifile)
% readCam(inifile)
% Input:
%   inifile - full path to the camera config file
% Demonstrates how the mrpt.mexgrabber application can be used
% to read data from a camera
%
% Examples:
% - Read through OpenCV driver:
% readCam('/home/jesus/Libs/mrpt/source/share/mrpt/config_files/rawlog-grabber/camera_opencv.ini')
% - Read a FireWire camera (IEEE1394):
% readCam('/home/jesus/Libs/mrpt/source/share/mrpt/config_files/rawlog-grabber/camera_1394.ini')

try
    % Create object:
    Cam = mrpt.mexgrabber(inifile);
    figure('Name','Camera window');
    obs = [];
    while isempty(obs)
        % Ensure at least one image is collected
        pause(1)
        obs = Cam.read;
    end
    
    h = imshow( obs{end}.image );
    while 1
        obs = Cam.read;
        if numel(obs) > 0
            set( h, 'CData', obs{end}.image );
        end
        pause(0.01);
    end
catch err
    fprintf('Catched exception, Cam object is being cleared and application safely closed:\n"%s"\n',err.message);
    disp(err.stack);
    clear Cam
end
