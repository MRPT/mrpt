# mrpt/system/__init__.py

# Import the compiled pybind11 module
from . import _bindings as _b

# Expose symbols at package level
CTicTac = _b.CTicTac
CTimeLogger = _b.CTimeLogger
CTimeLoggerEntry = _b.CTimeLoggerEntry
CTimeLoggerSaveAtDtor = _b.CTimeLoggerSaveAtDtor
compute_CRC16 = _b.compute_CRC16
compute_CRC32 = _b.compute_CRC32
encodeBase64 = _b.encodeBase64
decodeBase64 = _b.decodeBase64
unitsFormat = _b.unitsFormat
global_profiler_enter = _b.global_profiler_enter
global_profiler_leave = _b.global_profiler_leave
global_profiler_getref = _b.global_profiler_getref

# filesystem
fileExists = _b.fileExists
directoryExists = _b.directoryExists
getTempFileName = _b.getTempFileName
getcwd = _b.getcwd
createDirectory = _b.createDirectory
deleteFile = _b.deleteFile
renameFile = _b.renameFile
extractFileName = _b.extractFileName
extractFileExtension = _b.extractFileExtension
extractFileDirectory = _b.extractFileDirectory
fileNameChangeExtension = _b.fileNameChangeExtension
fileNameStripInvalidChars = _b.fileNameStripInvalidChars
getFileSize = _b.getFileSize
toAbsolutePath = _b.toAbsolutePath
pathJoin = _b.pathJoin
filePathSeparatorsToNative = _b.filePathSeparatorsToNative

# datetime
TTimeParts = _b.TTimeParts
buildTimestampFromParts = _b.buildTimestampFromParts
buildTimestampFromPartsLocalTime = _b.buildTimestampFromPartsLocalTime
timestampToParts = _b.timestampToParts
timeDifference = _b.timeDifference
timestampAdd = _b.timestampAdd
dateTimeToString = _b.dateTimeToString
dateTimeLocalToString = _b.dateTimeLocalToString
dateToString = _b.dateToString
timeToString = _b.timeToString
timeLocalToString = _b.timeLocalToString
formatTimeInterval = _b.formatTimeInterval
intervalFormat = _b.intervalFormat

__all__ = [
    # timing
    "CTicTac",
    "CTimeLogger",
    "CTimeLoggerEntry",
    "CTimeLoggerSaveAtDtor",
    "global_profiler_enter",
    "global_profiler_leave",
    "global_profiler_getref",
    # checksums / encoding
    "compute_CRC16",
    "compute_CRC32",
    "encodeBase64",
    "decodeBase64",
    # string utils
    "unitsFormat",
    # filesystem
    "fileExists",
    "directoryExists",
    "getTempFileName",
    "getcwd",
    "createDirectory",
    "deleteFile",
    "renameFile",
    "extractFileName",
    "extractFileExtension",
    "extractFileDirectory",
    "fileNameChangeExtension",
    "fileNameStripInvalidChars",
    "getFileSize",
    "toAbsolutePath",
    "pathJoin",
    "filePathSeparatorsToNative",
    # datetime
    "TTimeParts",
    "buildTimestampFromParts",
    "buildTimestampFromPartsLocalTime",
    "timestampToParts",
    "timeDifference",
    "timestampAdd",
    "dateTimeToString",
    "dateTimeLocalToString",
    "dateToString",
    "timeToString",
    "timeLocalToString",
    "formatTimeInterval",
    "intervalFormat",
]
