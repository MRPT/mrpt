"""
Python bindings for mrpt_system
"""
from __future__ import annotations
import datetime
import typing
__all__: list[str] = ['CTicTac', 'CTimeLogger', 'CTimeLoggerEntry', 'CTimeLoggerSaveAtDtor', 'TTimeParts', 'buildTimestampFromParts', 'buildTimestampFromPartsLocalTime', 'compute_CRC16', 'compute_CRC32', 'createDirectory', 'dateTimeLocalToString', 'dateTimeToString', 'dateToString', 'decodeBase64', 'deleteFile', 'directoryExists', 'encodeBase64', 'extractFileDirectory', 'extractFileExtension', 'extractFileName', 'fileExists', 'fileNameChangeExtension', 'fileNameStripInvalidChars', 'filePathSeparatorsToNative', 'formatTimeInterval', 'getFileSize', 'getTempFileName', 'getcwd', 'global_profiler_enter', 'global_profiler_getref', 'global_profiler_leave', 'intervalFormat', 'pathJoin', 'renameFile', 'timeDifference', 'timeLocalToString', 'timeToString', 'timestampAdd', 'timestampToParts', 'toAbsolutePath', 'unitsFormat']
class CTicTac:
    """
    A high-performance stopwatch, with typical resolution of nanoseconds.
    """
    def Tac(self) -> float:
        """
        Stop the stopwatch and return the elapsed time in seconds
        """
    def Tic(self) -> None:
        """
        Start or restart the stopwatch
        """
    def __init__(self) -> None:
        """
        Create a new stopwatch and start it automatically
        """
class CTimeLogger:
    """
    A versatile "profiler" that logs the time spent within each pair of calls to enter(X)-leave(X), among other stats.
    """
    def __init__(self, enabled: bool = True, name: str = '', keep_whole_history: bool = False) -> None:
        """
        Construct a CTimeLogger
        """
    def clear(self, deep_clear: bool = False) -> None:
        """
        Resets all stats. By default (deep_clear=false), all section names are remembered (not freed) so the cost of creating upon the first next call is avoided.
        """
    def disable(self) -> None:
        """
        Disables the logger.
        """
    def dumpAllStats(self, column_width: int = 80) -> None:
        """
        Dump all stats through the COutputLogger interface.
        """
    def enable(self, enabled: bool = True) -> None:
        """
        Enables or disables the logger.
        """
    def enableKeepWholeHistory(self, enable: bool = True) -> None:
        """
        If enabled, keeps all the measured times (not only the statistics).
        """
    def enter(self, section_name: str) -> None:
        """
        Start a named section (time measurement)
        """
    def getLastTime(self, section_name: str) -> float:
        """
        Return last execution time of a section
        """
    def getMeanTime(self, section_name: str) -> float:
        """
        Return mean execution time of a section
        """
    def getName(self) -> str:
        """
        Returns the logger name.
        """
    def getStatsAsText(self, column_width: int = 80) -> str:
        """
        Dump all stats to a multi-line text string.
        """
    def isEnabled(self) -> bool:
        """
        Returns true if the logger is enabled.
        """
    def isEnabledKeepWholeHistory(self) -> bool:
        """
        Returns true if all the measured times are kept.
        """
    def leave(self, section_name: str) -> float:
        """
        End a named section and return elapsed time in seconds
        """
    def saveToCSVFile(self, csv_file: str) -> None:
        """
        Dump all stats to a Comma Separated Values (CSV) file.
        """
    def saveToMFile(self, m_file: str) -> None:
        """
        Dump all stats to a Matlab/Octave (.m) file.
        """
    def setName(self, name: str) -> None:
        """
        Sets the logger name, shown in the statistics.
        """
class CTimeLoggerEntry:
    """
    Calls enter() on construction and leave() on stop() or destruction of a CTimeLogger section.
    """
    def __init__(self, logger: CTimeLogger, section_name: str) -> None:
        """
        Scoped time logging entry
        """
    def stop(self) -> None:
        """
        Ends the timed section now.
        """
class CTimeLoggerSaveAtDtor:
    """
    A helper class to save CSV stats upon self destruction, for example, at the end of a program run.
    """
    def __init__(self, logger: CTimeLogger) -> None:
        """
        Saves the statistics of the given logger to a CSV file when this object is destroyed.
        """
class TTimeParts:
    """
    Broken-down date/time representation (UTC or local)
    """
    day: int
    day_of_week: int
    hour: int
    minute: int
    month: int
    second: float
    year: int
    def __init__(self) -> None:
        """
        Default constructor.
        """
def buildTimestampFromParts(parts: TTimeParts) -> datetime.timedelta:
    """
    Build a TTimeStamp (UTC) from a TTimeParts struct
    """
def buildTimestampFromPartsLocalTime(parts: TTimeParts) -> datetime.timedelta:
    """
    Build a TTimeStamp (local time) from a TTimeParts struct
    """
@typing.overload
def compute_CRC16(data: list[int], gen_pol: int = 32773) -> int:
    """
    Compute CRC16 checksum from a vector of bytes
    """
@typing.overload
def compute_CRC16(data: bytes, gen_pol: int = 32773) -> int:
    """
    Compute CRC16 checksum from raw bytes
    """
@typing.overload
def compute_CRC32(data: list[int], gen_pol: int = 3988292384) -> int:
    """
    Compute CRC32 checksum from a vector of bytes
    """
@typing.overload
def compute_CRC32(data: bytes, gen_pol: int = 3988292384) -> int:
    """
    Compute CRC32 checksum from raw bytes
    """
def createDirectory(dirName: str) -> bool:
    """
    Creates a directory. Returns true on success
    """
def dateTimeLocalToString(t: datetime.timedelta) -> str:
    """
    Converts a timestamp to a human-readable local date-time string
    """
def dateTimeToString(t: datetime.timedelta) -> str:
    """
    Converts a timestamp to a human-readable UTC date-time string
    """
def dateToString(t: datetime.timedelta) -> str:
    """
    Converts a timestamp to a date-only string (UTC)
    """
def decodeBase64(input: str) -> bytes:
    """
    Decode a Base64 string into raw bytes
    """
def deleteFile(fileName: str) -> bool:
    """
    Deletes a file. Returns true on success
    """
def directoryExists(dirName: str) -> bool:
    """
    Returns true if the directory exists
    """
def encodeBase64(input: bytes) -> str:
    """
    Encode a bytes-like object into a Base64 string
    """
def extractFileDirectory(filePath: str) -> str:
    """
    Extracts the directory part of a full file path
    """
def extractFileExtension(filePath: str, ignore_gz: bool = False) -> str:
    """
    Extracts the file extension (without the dot)
    """
def extractFileName(filePath: str) -> str:
    """
    Extracts just the filename from a full path (no extension, no directory)
    """
def fileExists(fileName: str) -> bool:
    """
    Returns true if the file exists on disk
    """
def fileNameChangeExtension(filename: str, newExtension: str) -> str:
    """
    Returns the filename with the extension changed
    """
def fileNameStripInvalidChars(filename: str, replace_with: str = '_') -> str:
    """
    Replaces characters not valid for a filename with a replacement char
    """
def filePathSeparatorsToNative(filePath: str) -> str:
    """
    Converts path separators to the native format
    """
def formatTimeInterval(timeSeconds: float) -> str:
    """
    Formats a time interval (seconds) as a human-readable string (e.g. '1h 23m 45s')
    """
def getFileSize(fileName: str) -> int:
    """
    Returns the size of a file in bytes
    """
def getTempFileName() -> str:
    """
    Returns the name of a proposed temporary file
    """
def getcwd() -> str:
    """
    Returns the current working directory
    """
def global_profiler_enter(func_name: str) -> None:
    """
    Starts timing a section in the global profiler.
    """
def global_profiler_getref() -> CTimeLogger:
    """
    Returns the global profiler (a CTimeLogger).
    """
def global_profiler_leave(func_name: str) -> None:
    """
    Ends timing a section in the global profiler.
    """
def intervalFormat(seconds: float) -> str:
    """
    Format a time interval in seconds to a string
    """
def pathJoin(parts: list[str]) -> str:
    """
    Joins path components, mirroring Python's os.path.join semantics
    """
def renameFile(oldFileName: str, newFileName: str) -> bool:
    """
    Renames a file. Returns true on success
    """
def timeDifference(t_first: datetime.timedelta, t_later: datetime.timedelta) -> float:
    """
    Returns the difference in seconds (t_later - t_first)
    """
def timeLocalToString(t: datetime.timedelta, secondFractionDigits: int = 6) -> str:
    """
    Converts a timestamp to a local time-only string
    """
def timeToString(t: datetime.timedelta) -> str:
    """
    Converts a timestamp to a time-only string (UTC)
    """
def timestampAdd(t: datetime.timedelta, num_seconds: float) -> datetime.timedelta:
    """
    Adds a number of seconds to a timestamp
    """
def timestampToParts(t: datetime.timedelta, localTime: bool = False) -> TTimeParts:
    """
    Decomposes a TTimeStamp into a TTimeParts struct (UTC by default)
    """
def toAbsolutePath(path: str, resolveToCanonical: bool = False) -> str:
    """
    Converts a relative path to absolute, optionally resolving symlinks
    """
def unitsFormat(val: float, nDecimalDigits: int = 2, middle_space: bool = True) -> str:
    """
    Format a value with SI metric unit prefixes (e.g., 1e3 -> '1.00 K')
    """
