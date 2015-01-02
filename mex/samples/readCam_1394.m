%RUNDATABASE Demonstrates how the example Database API can be used.

% Using a database object.
Cam = mrpt.mexgrabber('/home/jesus/config_files_rawlog-grabber/camera_1394.ini');
figure('Name','Camera window');
pause(1)
obs = Cam.read;
h = imshow( obs{end}.image );
while 1
%     tic
%     [x,y,z] = Cam.read;
%     toc
    obs = Cam.read;
%     fprintf('%d frames read\n',length(obs));
    if numel(obs) > 0
        set( h, 'CData', obs{end}.image );
    end
    pause(0.01);
end
clear Cam
