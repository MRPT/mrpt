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


__all__ = [
    "CTicTac",
    "CTimeLogger",
    "CTimeLoggerEntry",
    "CTimeLoggerSaveAtDtor",
    "compute_CRC16",
    "compute_CRC32",
    "encodeBase64",
    "decodeBase64",
    "unitsFormat",
]
