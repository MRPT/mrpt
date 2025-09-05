"""
mrpt-config Python API.
"""

# Import the compiled pybind11 module
from . import _bindings as _b

# ----- Classes -----
CConfigFileBase = _b.CConfigFileBase
CConfigFile = _b.CConfigFile
CConfigFileMemory = _b.CConfigFileMemory
CLoadableOptions = _b.CLoadableOptions

# ----- Global Functions -----
config_parser = _b.config_parser
MRPT_SAVE_NAME_PADDING = _b.MRPT_SAVE_NAME_PADDING
MRPT_SAVE_VALUE_PADDING = _b.MRPT_SAVE_VALUE_PADDING

# ----- Optional __all__ for `from mrpt.config import *` -----
__all__ = [
    # Classes
    "CConfigFileBase",
    "CConfigFile",
    "CConfigFileMemory",
    "CLoadableOptions",
    # Functions
    "config_parser",
    "MRPT_SAVE_NAME_PADDING",
    "MRPT_SAVE_VALUE_PADDING",
]
