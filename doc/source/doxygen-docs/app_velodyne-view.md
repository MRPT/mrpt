\page app_velodyne-view velodyne-view


Live viewer and grabber for Velodyne 3D LIDARs (VLP-16, HDL-32, HDL-64),
built on mrpt::hwdrivers::CVelodyneScanner. It shows the point cloud of each
full revolution in a 3D window, either from a live sensor on the network or
from a PCAP file recorded with Wireshark or tcpdump, and can record the data
as a rawlog (mrpt::obs::CObservationVelodyneScan and, if the sensor sends
them, mrpt::obs::CObservationGPS observations) or as PCAP files.

# Usage

    velodyne-view [options]

| Option | Description |
|---|---|
| `-o`, `--out-rawlog FILE` | Save the grabbed data to a rawlog file (observations-only format). |
| `-i`, `--in-pcap FILE` | Read data from a PCAP file instead of a live sensor. |
| `--out-pcap PREFIX` | Save all received packets to PCAP files with this name prefix. |
| `--ip-filter IP` | Only accept data from the LIDAR with this IP address. |
| `-c`, `--calib FILE` | Calibration XML file of the LIDAR (as provided by the manufacturer). |
| `-m`, `--model MODEL` | Model, to load default calibration values if no calibration file is given: `VLP16` (default), `HDL32` or `HDL64`. |
| `-n`, `--nologo` | Don't print the banner at startup. |
| `-v`, `--verbose` | Verbose debug output. |
| `-h`, `--help`, `--version` | Show help or version. |

PCAP input and output require MRPT to be built with libpcap support.

# Examples

    # Live view of a VLP-16 on the local network:
    velodyne-view

    # Live view of an HDL-32 with its calibration file, recording a dataset:
    velodyne-view -m HDL32 -c modules/mrpt_data/config_files/rawlog-grabber/velodyne_default_calib_HDL-32.xml -o dataset.rawlog

    # Play back a PCAP file:
    velodyne-view -i recording.pcap -m VLP16

# User interface

The 3D window shows the point cloud of the latest scan, the reception time of
the latest LIDAR and GPS data, and the grabbing rate.
Mouse: orbit, pan and zoom as in any MRPT 3D window. Keys:

| Key | Action |
|---|---|
| `o` / `i` | Zoom out / in |
| Space | Freeze / unfreeze the view |
| `1` / `2` | Toggle keeping the last / strongest returns in dual return mode |
| `ESC` | Quit |

# See also

- [rawlog-grabber](page_app_rawlog-grabber.html): to record Velodyne data
  together with other sensors (see `config_files/rawlog-grabber/velodyne.ini`).
- [RawLogViewer](app_RawLogViewer.html), [rawlog-edit](page_app_rawlog-edit.html)
- mrpt::hwdrivers::CVelodyneScanner, mrpt::obs::CObservationVelodyneScan
