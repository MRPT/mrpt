"""
mrpt-serialization Python API.
"""

# CSerializable inherits CObject (mrpt.rtti); ensure it is registered first.
import mrpt.rtti  # noqa: F401

from . import _bindings as _b

# ----- Classes -----
CSerializable = _b.CSerializable
CArchive = _b.CArchive

# ----- Global Functions -----
# These wrap ObjectToOctetVector and OctetVectorToObject for Python 'bytes'
objectToBytes = _b.objectToBytes
bytesToObject = _b.bytesToObject

# ----- Optional __all__ -----
__all__ = [
    "CSerializable",
    "CArchive",
    "objectToBytes",
    "bytesToObject",
]
