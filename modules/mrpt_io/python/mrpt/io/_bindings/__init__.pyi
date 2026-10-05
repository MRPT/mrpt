"""
Python bindings for mrpt::io — file and memory streams
"""
from __future__ import annotations
import mrpt.serialization
import typing
from . import zip
__all__: list[str] = ['APPEND', 'CCompressedInputStream', 'CCompressedOutputStream', 'CFileGZInputStream', 'CFileGZOutputStream', 'CFileInputStream', 'CFileOutputStream', 'CMemoryStream', 'CStream', 'CompressionOptions', 'CompressionType', 'OpenMode', 'SeekOrigin', 'TRUNCATE', 'archiveFrom', 'detect_compression', 'file_get_contents', 'loadBinaryFile', 'loadTextFile', 'sFromBeginning', 'sFromCurrent', 'sFromEnd', 'vectorToBinaryFile', 'zip']
class OpenMode:
    """
    Members:
    
      TRUNCATE
    
      APPEND
    """
    APPEND: typing.ClassVar[OpenMode]
    TRUNCATE: typing.ClassVar[OpenMode]
    __members__: typing.ClassVar[dict[str, OpenMode]]
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: int) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: int) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class SeekOrigin:
    """
    Members:
    
      sFromBeginning
    
      sFromCurrent
    
      sFromEnd
    """
    __members__: typing.ClassVar[dict[str, SeekOrigin]]
    sFromBeginning: typing.ClassVar[SeekOrigin]
    sFromCurrent: typing.ClassVar[SeekOrigin]
    sFromEnd: typing.ClassVar[SeekOrigin]
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: int) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: int) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class CStream:
    """
    Base class of all MRPT streams (files, memory buffers, sockets, ...).
    """
    def Seek(self, offset: int, origin: SeekOrigin = ...) -> int:
        """
        Moves the read/write position by an offset relative to the given origin. Returns the new position.
        """
    def getPosition(self) -> int:
        """
        Method for getting the current cursor position, where 0 is the first byte and TotalBytesCount-1 the last one.
        """
    def getStreamDescription(self) -> str:
        """
        Returns a human-friendly description of the stream, e.g. a filename.
        """
    def getTotalBytesCount(self) -> int:
        """
        Returns the total amount of bytes in the stream.
        """
    def getline(self, arg0: str) -> bool:
        """
        Reads a text line (up to a newline character).
        """
    def read(self, count: int) -> bytes:
        """
        Read up to `count` bytes from the stream, returns bytes object
        """
    def write(self, data: bytes) -> int:
        """
        Write bytes to the stream, returns number of bytes written
        """
class CFileInputStream(CStream):
    """
    This CStream derived class allow using a file as a read-only, binary stream.
    """
    def __enter__(self) -> CFileInputStream:
        ...
    def __exit__(self, arg0: typing.Any, arg1: typing.Any, arg2: typing.Any) -> None:
        ...
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor.
        """
    @typing.overload
    def __init__(self, fileName: str) -> None:
        """
        Opens the given file for reading.
        """
    def __repr__(self) -> str:
        ...
    def checkEOF(self) -> bool:
        """
        Will be true if EOF has been already reached.
        """
    def clearError(self) -> None:
        """
        Resets stream error status bits (e.g. after an EOF)
        """
    def close(self) -> None:
        """
        Close the stream.
        """
    def fileOpenCorrectly(self) -> bool:
        """
        Returns true if the file was open without errors.
        """
    def getPosition(self) -> int:
        """
        Method for getting the current cursor position, where 0 is the first byte and TotalBytesCount-1 the last one.
        """
    def getStreamDescription(self) -> str:
        """
        Returns a human-friendly description of the stream, e.g. a filename.
        """
    def getTotalBytesCount(self) -> int:
        """
        Returns the total amount of bytes in the stream.
        """
    def is_open(self) -> bool:
        """
        Returns true if the file was open without errors.
        """
    def open(self, fileName: str) -> bool:
        """
        Open a file for reading. Returns true on success.
        """
    def readLine(self, arg0: str) -> bool:
        """
        Reads one string line from the file (until a new-line character)
        """
class CFileOutputStream(CStream):
    """
    This CStream derived class allow using a file as a write-only, binary stream.
    """
    def __enter__(self) -> CFileOutputStream:
        ...
    def __exit__(self, arg0: typing.Any, arg1: typing.Any, arg2: typing.Any) -> None:
        ...
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor.
        """
    @typing.overload
    def __init__(self, fileName: str, mode: OpenMode = ...) -> None:
        """
        Opens the given file for writing (truncate or append).
        """
    def __repr__(self) -> str:
        ...
    def close(self) -> None:
        """
        Close the stream.
        """
    def fileOpenCorrectly(self) -> bool:
        """
        Returns true if the file was open without errors.
        """
    def getPosition(self) -> int:
        """
        Method for getting the current cursor position, where 0 is the first byte and TotalBytesCount-1 the last one.
        """
    def getStreamDescription(self) -> str:
        """
        Returns a human-friendly description of the stream, e.g. a filename.
        """
    def getTotalBytesCount(self) -> int:
        """
        Method for getting the total number of bytes written to buffer.
        """
    def is_open(self) -> bool:
        """
        Returns true if the file was open without errors.
        """
    def open(self, fileName: str, mode: OpenMode = ...) -> bool:
        """
        Open a file for writing. Returns true on success.
        """
class CFileGZInputStream(CStream):
    """
    Transparently opens a compressed "gz" file and reads uncompressed data from it.
    """
    def __enter__(self) -> CFileGZInputStream:
        ...
    def __exit__(self, arg0: typing.Any, arg1: typing.Any, arg2: typing.Any) -> None:
        ...
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor: call open() before reading.
        """
    @typing.overload
    def __init__(self, fileName: str) -> None:
        """
        Opens the given gz-compressed file for reading.
        """
    def __repr__(self) -> str:
        ...
    def checkEOF(self) -> bool:
        """
        Will be true if EOF has been already reached.
        """
    def close(self) -> None:
        """
        Closes the file.
        """
    def fileOpenCorrectly(self) -> bool:
        """
        Returns true if the file was open without errors.
        """
    def filePathAtUse(self) -> str:
        """
        Returns the path of the filename passed to open(), or empty if none.
        """
    def getPosition(self) -> int:
        """
        Method for getting the current cursor position in the compressed, where 0 is the first byte and TotalBytesCount-1 the last one.
        """
    def getStreamDescription(self) -> str:
        """
        Returns a human-friendly description of the stream, e.g. a filename.
        """
    def getTotalBytesCount(self) -> int:
        """
        Method for getting the total number of compressed bytes of in the file (the physical size of the compressed file).
        """
    def is_open(self) -> bool:
        """
        Returns true if the file was open without errors.
        """
    def open(self, fileName: str) -> bool:
        """
        Open a .gz file for reading. Returns true on success.
        """
class CFileGZOutputStream(CStream):
    """
    Saves data to a file and transparently compress the data using the given compression level.
    """
    def __enter__(self) -> CFileGZOutputStream:
        ...
    def __exit__(self, arg0: typing.Any, arg1: typing.Any, arg2: typing.Any) -> None:
        ...
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor: call open() before writing.
        """
    @typing.overload
    def __init__(self, fileName: str, mode: OpenMode = ..., compressionLevel: int = 1) -> None:
        """
        Opens the given file for writing with gz compression (level 1: fastest).
        """
    def __repr__(self) -> str:
        ...
    def close(self) -> None:
        """
        Close the file.
        """
    def fileOpenCorrectly(self) -> bool:
        """
        Returns true if the file was open without errors.
        """
    def filePathAtUse(self) -> str:
        """
        Returns the path of the filename passed to open(), or empty if none.
        """
    def getPosition(self) -> int:
        """
        Method for getting the current cursor position, where 0 is the first byte and TotalBytesCount-1 the last one.
        """
    def getStreamDescription(self) -> str:
        """
        Returns a human-friendly description of the stream, e.g. a filename.
        """
    def is_open(self) -> bool:
        """
        Returns true if the file was open without errors.
        """
    def open(self, fileName: str, compress_level: int = 1, mode: OpenMode = ...) -> bool:
        """
        Open a .gz file for writing. Returns true on success.
        """
class CMemoryStream(CStream):
    """
    A stream over a memory buffer.
    """
    def Seek(self, offset: int, origin: SeekOrigin = ...) -> int:
        """
        Moves the read/write position by an offset relative to the given origin. Returns the new position.
        """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def __repr__(self) -> str:
        ...
    def clear(self) -> None:
        """
        Clears the memory buffer.
        """
    def getContents(self) -> bytes:
        """
        Return a copy of the stream buffer as a Python bytes object
        """
    def getPosition(self) -> int:
        """
        Method for getting the current cursor position, where 0 is the first byte and TotalBytesCount-1 the last one.
        """
    def getTotalBytesCount(self) -> int:
        """
        Returns the total size of the internal buffer.
        """
    def loadBufferFromFile(self, arg0: str) -> bool:
        """
        Loads the entire buffer from a file.
        """
    def saveBufferToFile(self, arg0: str) -> bool:
        """
        Saves the entire buffer to a file.
        """
class CompressionType:
    """
    Members:
    
      None_
    
      Gzip
    
      Zstd
    """
    Gzip: typing.ClassVar[CompressionType]
    None_: typing.ClassVar[CompressionType]
    Zstd: typing.ClassVar[CompressionType]
    __members__: typing.ClassVar[dict[str, CompressionType]]
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: int) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: int) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class CompressionOptions:
    """
    Compression options for output streams.
    """
    level: int
    type: CompressionType
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor.
        """
    @typing.overload
    def __init__(self, type: CompressionType, level: int = 1) -> None:
        """
        Builds compression options from a compression type and level.
        """
    def __repr__(self) -> str:
        ...
class CCompressedInputStream(CStream):
    """
    Transparently reads from a compressed file, automatically detecting the compression format from the file magic signature.
    """
    def __enter__(self) -> CCompressedInputStream:
        ...
    def __exit__(self, arg0: typing.Any, arg1: typing.Any, arg2: typing.Any) -> None:
        ...
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor: call open() before reading.
        """
    @typing.overload
    def __init__(self, fileName: str) -> None:
        """
        Opens the given file, detecting its compression format.
        """
    def __repr__(self) -> str:
        ...
    def checkEOF(self) -> bool:
        """
        Will be true if EOF has been already reached.
        """
    def close(self) -> None:
        """
        Closes the file.
        """
    def fileOpenCorrectly(self) -> bool:
        """
        Returns true if the file was opened without errors.
        """
    def filePathAtUse(self) -> str:
        """
        Returns the path of the filename passed to open(), or empty if none.
        """
    def getCompressionType(self) -> CompressionType:
        """
        Returns the detected compression type for the opened file.
        """
    def getPosition(self) -> int:
        """
        Method for getting the current cursor position in the compressed data, where 0 is the first byte and TotalBytesCount-1 the last one.
        """
    def getStreamDescription(self) -> str:
        """
        Returns a human-friendly description of the stream, e.g. a filename.
        """
    def getTotalBytesCount(self) -> int:
        """
        Method for getting the total number of compressed bytes in the file (the physical size of the file on disk).
        """
    def is_open(self) -> bool:
        """
        Returns true if the file was opened without errors.
        """
    def open(self, fileName: str) -> bool:
        """
        Open a plain, gzip or zstd file for reading. Returns true on success.
        """
class CCompressedOutputStream(CStream):
    """
    Saves data to a file with optional transparent compression.
    """
    def __enter__(self) -> CCompressedOutputStream:
        ...
    def __exit__(self, arg0: typing.Any, arg1: typing.Any, arg2: typing.Any) -> None:
        ...
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor: call open() before writing.
        """
    @typing.overload
    def __init__(self, fileName: str, mode: OpenMode = ..., options: CompressionOptions = ...) -> None:
        """
        Opens the given file for writing with the given compression options.
        """
    def __repr__(self) -> str:
        ...
    def close(self) -> None:
        """
        Close the file.
        """
    def fileOpenCorrectly(self) -> bool:
        """
        Returns true if the file was opened without errors.
        """
    def filePathAtUse(self) -> str:
        """
        Returns the path of the filename passed to open(), or empty if none.
        """
    def getCompressionType(self) -> CompressionType:
        """
        Returns the compression type being used.
        """
    def getPosition(self) -> int:
        """
        Method for getting the current cursor position in the compressed stream, where 0 is the first byte and TotalBytesCount-1 the last one.
        """
    def getStreamDescription(self) -> str:
        """
        Returns a human-friendly description of the stream, e.g. a filename.
        """
    def is_open(self) -> bool:
        """
        Returns true if the file was opened without errors.
        """
    def open(self, fileName: str, options: CompressionOptions = ..., mode: OpenMode = ...) -> bool:
        """
        Open a file for writing (default: zstd). Returns true on success.
        """
def archiveFrom(stream: CStream) -> mrpt.serialization.CArchive:
    """
    Returns a CArchive to read/write MRPT objects from/to the given stream. The stream is kept alive while the archive exists.
    """
def detect_compression(filePath: str) -> CompressionType:
    """
    Detects the compression of a file from its magic bytes (not its extension).
    """
def file_get_contents(fileName: str) -> str:
    """
    Load an entire text file as a single string. Raises on error.
    """
def loadBinaryFile(fileName: str) -> list[int] | None:
    """
    Load an entire binary file. Returns None (nullopt) on error, or a bytes-like list of uint8.
    """
def loadTextFile(fileName: str) -> list[str] | None:
    """
    Load a text file as a list of string lines. Returns None (nullopt) on error.
    """
def vectorToBinaryFile(data: bytes, fileName: str) -> bool:
    """
    Save a bytes object to a binary file. Returns False on error.
    """
APPEND: OpenMode
TRUNCATE: OpenMode
sFromBeginning: SeekOrigin
sFromCurrent: SeekOrigin
sFromEnd: SeekOrigin
