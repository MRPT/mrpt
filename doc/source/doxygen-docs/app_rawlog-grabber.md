\page app_rawlog-grabber rawlog-grabber


Records datasets from any number of sensors into an MRPT
[rawlog](rawlog_format.html) file. Each sensor declared in an INI config file
runs its driver (any class derived from mrpt::hwdrivers::CGenericSensor:
laser scanners, cameras, RGBD cameras, GNSS receivers, IMUs, Velodyne
LiDARs, etc.) in its own thread; the main thread collects the observations of
all sensors, sorts them by timestamp, and appends them to the output file.
This is the standard way to grab datasets on a robot for later, off-line
processing with MRPT apps.

# Usage

    rawlog-grabber <config_file.ini> [-p <plugin1.so,plugin2.so,...>]

- `config_file.ini`: the config file with the sensor definitions (required).
- `-p, --plugins`: comma-separated list of plugin libraries (`.so`/`.dll`) to
  load before starting, with additional sensor drivers.
- `--version`, `-h, --help`.

Recording stops when a key is pressed in the console, or when any sensor
thread fails (e.g. the device cannot be opened). Set the environment variable
`MRPT_HWDRIVERS_VERBOSE=1` to print debug information (e.g. GNSS fix mode and
IMU angles).

# Example

Using one of the sample config files in `mrpt_data`, for a Hokuyo laser
scanner:

    rawlog-grabber config_files/rawlog-grabber/hokuyo_UTM.ini

The output is a file like `dataset_2026-01-31_10h20m30s.rawlog` in the current
directory. Camera images, if any, are stored as external files in the
directory `dataset_2026-01-31_10h20m30s_Images`.

# Configuration file

The `[global]` section holds the application settings:

| Key | Default | Description |
|---|---|---|
| `rawlog_prefix` | `dataset` | Prefix of the output file name, which may include a path. The final name is `<prefix>_<date>_<time>.rawlog`. |
| `time_between_launches` | 300 | Delay between launching consecutive sensor threads (ms). |
| `use_sensoryframes` | false | Save in the "actions-sensory frames" format (observations close in time are grouped in mrpt::obs::CSensoryFrame objects, and odometry is converted into actions) instead of the "observations-only" format. |
| `SF_max_time_span` | 0.25 | Maximum time span (s) of the observations in one sensory frame. |
| `GRABBER_PERIOD_MS` | 1000 | Period (ms) at which the main thread saves the collected observations. |
| `rawlog_GZ_compress_level` | 1 | Compression level of the output file (zstd, 1-22), or 0 for no compression (useful for high-rate 3D cameras). |

Every other section defines one sensor. Its name is the thread name, and the
section must contain:

- `driver`: the C++ class name of the sensor driver (e.g. `CHokuyoURG`,
  `CGPSInterface`, `CCameraSensor`).
- `process_rate`: rate (Hz) at which the driver is polled (mandatory).

These keys are optional and common to all drivers (see
mrpt::hwdrivers::CGenericSensor): `sensorLabel`, `max_queue_len` (default:
200) and `grab_decimation` (keep only 1 out of N observations). The rest of
the keys are specific to each driver: refer to the documentation of its class.
A section with `rawlog-grabber-ignore = true` is skipped.

The directory `config_files/rawlog-grabber/` in `mrpt_data` contains commented
sample files for most supported sensors.

# See also

- [RawLogViewer](app_RawLogViewer.html) and \ref app_rawlog-edit: inspect and
  edit the recorded datasets.
- mrpt::apps::RawlogGrabberApp: this app as a C++ class.
