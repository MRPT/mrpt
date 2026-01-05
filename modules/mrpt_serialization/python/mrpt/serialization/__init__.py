"""
mrpt-serialization Python API.
"""

# Import the compiled pybind11 module
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
