# mrpt/core/__init__.py

"""
mrpt-core Python API.
"""

# Import the compiled pybind11 module
from . import _bindings as _b

# ----- abs_diff functions -----
abs_diff_int = _b.abs_diff_int
abs_diff_long = _b.abs_diff_long
abs_diff_float = _b.abs_diff_float
abs_diff_double = _b.abs_diff_double

# ----- bits_math -----
deg2rad = _b.deg2rad
rad2deg = _b.rad2deg

# ----- reverse_bytes -----
reverse_bytes_u16 = _b.reverse_bytes_u16
reverse_bytes_u32 = _b.reverse_bytes_u32
reverse_bytes_u64 = _b.reverse_bytes_u64
reverse_bytes_i16 = _b.reverse_bytes_i16
reverse_bytes_i32 = _b.reverse_bytes_i32
reverse_bytes_i64 = _b.reverse_bytes_i64

# ----- Stringifyable -----
Stringifyable = _b.Stringifyable

# ----- Clock -----
Clock = _b.Clock

# ----- WorkerThreadsPool -----
WorkerThreadsPool = _b.WorkerThreadsPool

# ----- Optional __all__ for `from mrpt.core import *` -----
__all__ = [
    # abs_diff
    "abs_diff_int",
    "abs_diff_long",
    "abs_diff_float",
    "abs_diff_double",
    # bits_math
    "deg2rad",
    "rad2deg",
    # reverse_bytes
    "reverse_bytes_u16",
    "reverse_bytes_u32",
    "reverse_bytes_u64",
    "reverse_bytes_i16",
    "reverse_bytes_i32",
    "reverse_bytes_i64",
    # classes
    "Stringifyable",
    "Clock",
    "WorkerThreadsPool",
]
