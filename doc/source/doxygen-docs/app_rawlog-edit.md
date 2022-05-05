\page  app_rawlog-edit Application: rawlog-edit

## NAME

rawlog-edit - Command-line robotic datasets (rawlogs) manipulation tool

## SYNOPSIS

   rawlog-edit  [--describe] [--undistort] [--rename-externals]
                [--stereo-rectify <SENSOR_LABEL,0.5>] [--camera-params
                <SENSOR_LABEL,file.ini>] [--sensors-pose <file.ini>]
                [--generate-3d-pointclouds] [--cut] [--export-2d-scans-txt]
                [--export-txt] [--export-rawdaq-txt] [--recalc-odometry]
                [--export-anemometer-txt] [--export-enose-txt]
                [--export-odometry-txt] [--export-imu-txt]
                [--export-gps-all] [--export-gps-txt]
                [--export-gps-gas-kml] [--export-gps-kml] [--keep-label
                <label[,label...]>] [--remove-label <label[,label...]>]
                [--list-range-bearing] [--remap-timestamps <a;b>]
                [--list-timestamps] [--list-poses] [--list-images] [--info]
                [--de-externalize] [--externalize] [-q] [-w] [--odo-D <D>]
                [--odo-KR <KR>] [--odo-KL <KL>] [--to-time <T1>]
                [--from-time <T0>] [--to-index <N1>] [--from-index <N0>]
                [--text-file-output <out.txt>] [--rectify-centers-coincide]
                [--image-size <COLSxROWS>] [--txt-externals]
                [--externals-filename-format <"${type}_${label}_%.06%f">]
                [--image-format <jpg,png,pgm,...>] [--out-dir <.>] [-p
                <mylib.so>] [-o <dataset_out.rawlog>] -i <dataset.rawlog>
                [--] [--version] [-h]

## USAGE EXAMPLES

**Quick overview of a dataset file:**

    rawlog-edit --info -i in.rawlog


**Cut the entries [1000,2000] into another rawlog file:**

    rawlog-edit --cut --from-index 1000 --to-index 2000 -i in.rawlog -o out.rawlog


**Cut the entries from the beginning up to timestamp 1281619819:**

    rawlog-edit --cut --to-time 1281619819 -i in.rawlog -o out.rawlog


**Export all suitable observations to TXT/CSV files:**

    rawlog-edit --export-txt -i in.rawlog


**Generate a Google Earth KML file with the GPS data in a dataset:**

    rawlog-edit --export-gps-kml -i in.rawlog

**Remove all observations named "REAR_LASER":**

    rawlog-edit --remove-label REAR_LASER -i in.rawlog -o out.rawlog

**Remove all observations not named "REAR_LASER":**

    rawlog-edit --keep-label REAR_LASER -i in.rawlog -o out.rawlog

**Convert all images to external storage mode:**

    rawlog-edit --externalize -i in.rawlog -o out.rawlog
    rawlog-edit --externalize --image-format jpg -i in.rawlog -o out.rawlog


## DESCRIPTION

**rawlog-edit** is a command-line application to inspect and manipulate
robotic dataset files in the "rawlog" standardized format. At least one
"operation flag" (those defined as "Op: ..." below) is required.

These are the supported arguments and operations:

   --describe
     Op: Prints a human-readable description for *all* objects in the
     dataset.

   --undistort
     Op: Undistort all images in the rawlog.


   --rename-externals
     Op: Renames all the external storage file names within the rawlog (it
     doesn't change the external files, which may even not exist).


   --stereo-rectify <SENSOR_LABEL,0.5>
     Op: creates a new set of external images for all
     CObservationStereoImages with the given SENSOR_LABEL, using the camera
     parameters stored in the observations (which must be a valid
     calibration) and with the given alpha value. Alpha can be -1 for auto,
     or otherwise be in the range [0,1] (see OpenCV's docs for
     cvStereoRectify).

     Requires: -o (or --output)

     Optional: --image-format to set image format (default=jpg), 

     --image-size to resize output images (example: --image-size 640x480)
     


   --camera-params <SENSOR_LABEL,file.ini>
     Op: change the camera intrinsic parameters of all CObservationImage
     with the given SENSOR_LABEL, with new params loaded from the given
     file, section '[CAMERA_PARAMS]' for monocular cameras, or
     '[CAMERA_PARAMS_LEFT]' and '[CAMERA_PARAMS_RIGHT]' for
     CObservationStereoImage, or '[DEPTH_CAM_PARAMS]' and
     '[INTENSITY_CAM_PARAMS]' for CObservation3DRangeScan.

     Requires: -o (or --output)


   --sensors-pose <file.ini>
     Op: batch change the poses of sensors from a rawlog-grabber-like
     configuration file that specifies the pose of sensors by their
     sensorLabel names.

     Requires: -o (or --output)


   --generate-3d-pointclouds
     Op: (re)generate the 3D pointclouds within CObservation3DRangeScan
     objects that have range data.

     Requires: -o (or --output)


   --cut
     Op: Cut a part of the input rawlog.

     Requires: -o (or --output)

     Requires: At least one of --from-index, --from-time, --to-index,
     --to-time. Use only one of the --from-* and --to-* at once.

     If only a --from-* is given, the rawlog will be saved up to its end.
     If only a --to-* is given, the rawlog will be saved from its
     beginning.


   --export-2d-scans-txt
     Op: Export 2D scans to TXT files.

     Generates two .txt files for each different sensor label of 2D scan
     observations, one with the timestamps and the other with range
     data.

     The generated .txt files will be saved in the same path than the input
     rawlog, with the same filename + each sensorLabel.

   --export-txt
     Op: Generic export observations to TXT/CSV files.

     Generates one .txt file for each different sensor label of all
     observation classes that supports the export-to-txt API.

     The generated .txt files will be saved in the same path than the input
     rawlog, as `<rawlog_filename>_<sensorLabel>.txt`.

   --export-rawdaq-txt
     Op: Export raw DAQ readings to TXT files.

     Generates one .txt file for each different sensor label + DAQ task.
     The generated .txt files will be saved in the same path than the input
     rawlog.

   --recalc-odometry
     Op: Recomputes odometry increments from new encoder-to-odometry
     constants.

     Requires: -o (or --output)

     Requires: --odo-KL, --odo-KR and --odo-D.


   --export-anemometer-txt
     Op: Export anemometer readigns to TXT files.

     Generates one .txt file for each different sensor label of an
     anemometer observation in the dataset. The generated .txt files will
     be saved in the same path than the input rawlog, with the same
     filename + each sensorLabel.

   --export-enose-txt
     Op: Export e-nose readigns to TXT files.

     Generates one .txt file for each different sensor label of an e-nose
     observation in the dataset. The generated .txt files will be saved in
     the same path than the input rawlog, with the same filename + each
     sensorLabel.

   --export-odometry-txt
     Op: Export absolute odometry readings to TXT files.

     Generates one .txt file for each different sensor label of an odometry
     observation in the dataset. The generated .txt files will be saved in
     the same path than the input rawlog, with the same filename + each
     sensorLabel.

   --export-imu-txt
     Op: Export IMU readings to TXT files.

     Generates one .txt file for each different sensor label of an IMU
     observation in the dataset. The generated .txt files will be saved in
     the same path than the input rawlog, with the same filename + each
     sensorLabel.

   --export-gps-all
     Op: Generic export all kinds of GPS/GNSS messages to separate TXT
     files.

     Generates one .txt file for each different sensor label and for each
     message type in the dataset, with a first header line describing each
     field.

   --export-gps-txt
     Op: Export GPS GPGGA messages to TXT files.

     Generates one .txt file for each different sensor label of GPS
     observations in the dataset. The generated .txt files will be saved in
     the same path than the input rawlog, with the same filename + each
     sensorLabel.

   --export-gps-gas-kml
     Op: Export GPS paths to Google Earth KML files coloured by the gas
     concentration.

     Generates one .kml file with different sections for each different
     sensor label of GPS observations in the dataset. The generated .kml
     files will be saved in the same path than the input rawlog, with the
     same filename + each sensorLabel.

   --export-gps-kml
     Op: Export GPS paths to Google Earth KML files.

     Generates one .kml file with different sections for each different
     sensor label of GPS observations in the dataset. The generated .kml
     files will be saved in the same path than the input rawlog, with the
     same filename + each sensorLabel.

   --keep-label <label[,label...]>
     Op: Remove all observations not matching the given sensor
     label(s).Several labels can be provided separated by commas.

     Requires: -o (or --output)

   --remove-label <label[,label...]>
     Op: Remove all observation matching the given sensor label(s).Several
     labels can be provided separated by commas.

     Requires: -o (or --output)

   --list-range-bearing
     Op: dump a list of all landmark observations of type
     range-bearing.

     Optionally the output text file can be changed with
     --text-file-output.

   --remap-timestamps <a;b>
     Op: Change all timestamps t replacing it with the linear map
     'a*t+b'.The parameters 'a' and 'b' must be given separated with a
     semicolon.

     Requires: -o (or --output)

   --list-timestamps
     Op: generates a list with all the observations' timestamp, sensor
     label and C++ class name.

     Optionally the output text file can be changed with
     --text-file-output.

   --list-poses
     Op: dump a list of all the poses of the observations in the
     dataset.

     Optionally the output text file can be changed with
     --text-file-output.

   --list-images
     Op: dump a list of all external image files in the dataset.

     Optionally the output text file can be changed with
     --text-file-output.

   --info
     Op: parse input file and dump information and statistics.

   --de-externalize
     Op: the opposite that --externalize: generates a monolitic rawlog file
     with all external files integrated in one.

     Requires: -o (or --output)


   --externalize
     Op: convert to external storage.

     Requires: -o (or --output)

     Optional: --image-format, --txt-externals

   -q,  --quiet
     Terse output

   -w,  --overwrite
     Force overwrite target file without prompting.

   --odo-D <D>
     Distance between left-right wheels (meters), used in
     --recalc-odometry.

   --odo-KR <KR>
     Constant from encoder ticks to meters (right wheel), used in
     --recalc-odometry.

   --odo-KL <KL>
     Constant from encoder ticks to meters (left wheel), used in
     --recalc-odometry.

   --to-time <T1>
     End time for --cut, as UNIX timestamp, optionally with fractions of
     seconds.

   --from-time <T0>
     Starting time for --cut, as UNIX timestamp, optionally with fractions
     of seconds.

   --to-index <N1>
     End index for --cut

   --from-index <N0>
     Starting index for --cut

   --text-file-output <out.txt>
     Output for a text file

   --rectify-centers-coincide
     In stereo rectification, force that both image centers after coincide
     after rectifying.

   --image-size <COLSxROWS>
     Resize output images

   --txt-externals
     When externalizing CObservation3DRangeScan objects, switched from
     binary files (default) to plain text.

   --externals-filename-format <"${type}_${label}_%.06%f">
     Format string for the command --rename-externals.(Default:
     "${type}_${label}_%.06%f"). Refer to docs for
     mrpt::obs::format_externals_filename().

   --image-format <jpg,png,pgm,...>
     External image format

   --out-dir <.>
     Output directory (used by some commands only)

   -p <mylib.so>,  --plugins <mylib.so>
     Single or comma-separated list of .so/.dll plugins to load for
     additional user-supplied classes

   -o <dataset_out.rawlog>,  --output <dataset_out.rawlog>
     Output dataset (*.rawlog)

   -i <dataset.rawlog>,  --input <dataset.rawlog>
     (required)  Input dataset (required) (*.rawlog)

   --,  --ignore_rest
     Ignores the rest of the labeled arguments following this flag.

   --version
     Displays version information and exits.

   -h,  --help
     Displays usage information and exits.

