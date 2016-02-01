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
% - Read a Bumblebee camera (need Flycapture and Triclops installed):
% readCam('/home/jesus/Libs/mrpt/source/share/mrpt/config_files/rawlog-grabber/camera_pgr_flycap.ini')

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
    h = showImage( obs{end} );
    while 1
        obs = Cam.read;
        if numel(obs) > 0
            updateImage(h,obs{end});
        end
        pause(0.01);
    end
catch err
    fprintf('Catched exception, Cam object is being cleared and application safely closed:\n"%s"\n',err.message);
    disp(err.stack);
    clear Cam
end
end

function h = showImage( obs )
% Create new image in current figure with imshow command
% If image is stereo imshowpair with 'montage' mode is used
switch obs.class
    case 'CObservationStereoImages'
        h = imshowpair( obs.imageL, obs.imageR, 'montage' );
    otherwise
        h = imshow( obs.image );
end
end

function updateImage( h, obs )
% Change current image CData with images in input
switch obs.class
    case 'CObservationStereoImages'
        im = imfuse(obs.imageL,obs.imageR,'montage');
    otherwise
        im = obs.image;
end
set( h, 'CData', im );
end
