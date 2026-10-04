"""
Python bindings for mrpt_serialization
"""
from __future__ import annotations
import mrpt.rtti
import typing
__all__: list[str] = ['CArchive', 'CExceptionEOF', 'CSerializable', 'bytesToObject', 'objectToBytes']
class CExceptionEOF(EOFError):
    pass
class CSerializable:
    def GetRuntimeClass(self) -> mrpt.rtti.TRuntimeClassId:
        ...
class CArchive:
    def ReadDouble(self) -> float:
        ...
    def ReadInt(self) -> int:
        ...
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
