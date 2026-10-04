"""
mrpt-serialization Python API.
"""
from __future__ import annotations
import mrpt as mrpt
from mrpt.serialization._bindings import CArchive as CArchive
from mrpt.serialization._bindings import CExceptionEOF as CExceptionEOF
from mrpt.serialization._bindings import CSerializable as CSerializable
from mrpt.serialization._bindings import bytesToObject as bytesToObject
from mrpt.serialization._bindings import objectToBytes as objectToBytes
from . import _bindings
__all__: list = ['CSerializable', 'CArchive', 'CExceptionEOF', 'archiveFrom', 'objectToBytes', 'bytesToObject']
def __getattr__(name):
    ...
