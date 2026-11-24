\page app_gps2rawlog gps2rawlog

# gps2rawlog

Parses a raw log of the data stream of a GNSS (GPS) receiver, as saved by a
serial port terminal or any other logging tool, and saves all the recognized
messages as an MRPT dataset ([rawlog](rawlog_format.html) file). Use it to
process GNSS data recorded without MRPT, with the same parsers used by
mrpt::hwdrivers::CGPSInterface when grabbing live data.

## Usage

    gps2rawlog -i <input_file> [-o <output.rawlog>] [-w]

- `-i, --input`: input raw GNSS data file (required). It may be uncompressed,
  or gzip or zstd compressed.
- `-o, --output`: output rawlog file. Default: the input file name with the
  extension changed to `.rawlog`.
- `-w, --overwrite`: overwrite the output file if it already exists (otherwise,
  the program aborts).
- `--version`, `-h, --help`.

## Example

    gps2rawlog -i gps_session.txt

writes `gps_session.rawlog`, which can be inspected with
[RawLogViewer](app_RawLogViewer.html) or exported to text files with:

    rawlog-edit -i gps_session.rawlog --export-gps-txt

## Input / output formats

The data format is detected automatically (the `AUTO` parser of
mrpt::hwdrivers::CGPSInterface). Supported formats:

- NMEA 0183 (ASCII sentences).
- Novatel OEM6 binary frames.

The output rawlog is in the "observations-only" format: one
mrpt::obs::CObservationGPS per decoded message, with the decoded message
stored in it (see the mrpt::obs::gnss namespace for the message types).

## See also

- \ref app_rawlog-edit: export GNSS data to text or KML files, among other
  operations.
- \ref app_rawlog-grabber: to record GNSS data (and any other sensor) directly
  into rawlog files.
