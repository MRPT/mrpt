"""
Python bindings for mrpt_serialization
"""
from __future__ import annotations
import mrpt.rtti
import typing
__all__: list[str] = ['CArchive', 'CExceptionEOF', 'CSerializable', 'bytesToObject', 'objectToBytes']
class CExceptionEOF(EOFError):
    """
    End of stream reached while reading an object (an EOFError).
    """
class CSerializable:
    """
    The virtual base class which provides a unified interface for all persistent objects in MRPT.
    """
    def GetRuntimeClass(self) -> mrpt.rtti.TRuntimeClassId:
        """
        Returns information about the class of an object in runtime.
        """
class CArchive:
    """
    Serializes and deserializes MRPT objects to/from a stream.
    """
    def ReadDouble(self) -> float:
        """
        Reads a double value.
        """
    def ReadInt(self) -> int:
        """
        Reads an int value.
        """
    @typing.overload
    def ReadObject(self) -> CSerializable:
        """
        Reads an MRPT object from the stream. Raises EOFError at the end of the stream.
        """
    @typing.overload
    def ReadObject(self, obj: CSerializable) -> None:
        """
        Reads an MRPT object from the stream into an existing object, which must be of the same class.
        """
    def WriteObject(self, arg0: CSerializable) -> None:
        """
        Writes an MRPT object to the stream.
        """
    def __repr__(self) -> str:
        ...
def bytesToObject(arg0: bytes) -> CSerializable:
    """
    Converts a Python bytes object back into an MRPT object.
    """
def objectToBytes(arg0: CSerializable) -> bytes:
    """
    Converts an MRPT object to a Python bytes object.
    """
