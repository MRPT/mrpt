"""
mrpt-rtti Python API.
"""

# Import the compiled pybind11 module
from . import _bindings as _b


def _check_numpy_compatibility():
    """Warns if NumPy is too new for the pybind11 the bindings were built with.

    pybind11 < 2.12 does not support NumPy >= 2: passing NumPy arrays to MRPT
    functions then crashes. This happens when a pip-installed NumPy 2
    shadows the system one.
    """
    try:
        from importlib.metadata import version

        numpy_major = int(version("numpy").split(".")[0])
    except Exception:
        return
    if numpy_major >= 2 and tuple(_b._pybind11_version[:2]) < (2, 12):
        import warnings

        warnings.warn(
            "MRPT Python bindings were built with pybind11 "
            f"{'.'.join(map(str, _b._pybind11_version))}, which does not support "
            f"NumPy {version('numpy')}: passing NumPy arrays to MRPT will crash. "
            "Use NumPy < 2 (e.g. the system package) or rebuild MRPT with "
            "pybind11 >= 2.12.",
            RuntimeWarning,
            stacklevel=3,
        )


_check_numpy_compatibility()

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
]
