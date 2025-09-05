"""
mrpt-rtti Python API.
"""

# Import the compiled pybind11 module
from . import _bindings as _b

# ----- Classes -----
CObject = _b.CObject
TRuntimeClassId = _b.TRuntimeClassId

# ----- Global Functions -----
registerClass = _b.registerClass
registerClassCustomName = _b.registerClassCustomName
getAllRegisteredClasses = _b.getAllRegisteredClasses
getAllRegisteredClassesChildrenOf = _b.getAllRegisteredClassesChildrenOf
findRegisteredClass = _b.findRegisteredClass
registerAllPendingClasses = _b.registerAllPendingClasses
classFactory = _b.classFactory
ptr_cast = _b.ptr_cast

# ----- Optional __all__ for `from mrpt.rtti import *` -----
__all__ = [
    # Classes
    "CObject",
    "TRuntimeClassId",
    # Functions
    "registerClass",
    "registerClassCustomName",
    "getAllRegisteredClasses",
    "getAllRegisteredClassesChildrenOf",
    "findRegisteredClass",
    "registerAllPendingClasses",
    "classFactory",
    "ptr_cast",
]
