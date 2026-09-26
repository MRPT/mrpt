"""
mrpt-serialization Python API.
"""

# CSerializable inherits CObject (mrpt.rtti); ensure it is registered first.
import mrpt.rtti  # noqa: F401

from . import _bindings as _b

# ----- Classes -----
CSerializable = _b.CSerializable
CArchive = _b.CArchive
CExceptionEOF = _b.CExceptionEOF  # subclass of EOFError

# ----- Global Functions -----
# These wrap ObjectToOctetVector and OctetVectorToObject for Python 'bytes'
objectToBytes = _b.objectToBytes
bytesToObject = _b.bytesToObject

# ----- Optional __all__ -----


def __getattr__(name):
    # archiveFrom() needs mrpt.io streams, and mrpt.io imports this module:
    # resolve it lazily to avoid an import cycle.
    if name == "archiveFrom":
        from mrpt.io import archiveFrom
        return archiveFrom
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


__all__ = [
    "CSerializable",
    "CArchive",
    "CExceptionEOF",
    "archiveFrom",
    "objectToBytes",
    "bytesToObject",
]
