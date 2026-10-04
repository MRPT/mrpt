"""
mrpt-rtti Python API.
"""
from __future__ import annotations
from mrpt.rtti._bindings import CObject as CObject
from mrpt.rtti._bindings import TRuntimeClassId as TRuntimeClassId
from mrpt.rtti._bindings import classFactory as classFactory
from mrpt.rtti._bindings import findRegisteredClass as findRegisteredClass
from mrpt.rtti._bindings import getAllRegisteredClasses as getAllRegisteredClasses
from mrpt.rtti._bindings import getAllRegisteredClassesChildrenOf as getAllRegisteredClassesChildrenOf
from mrpt.rtti._bindings import registerAllPendingClasses as registerAllPendingClasses
from mrpt.rtti._bindings import registerClass as registerClass
from mrpt.rtti._bindings import registerClassCustomName as registerClassCustomName
from . import _bindings
__all__: list = ['CObject', 'TRuntimeClassId', 'registerClass', 'registerClassCustomName', 'getAllRegisteredClasses', 'getAllRegisteredClassesChildrenOf', 'findRegisteredClass', 'registerAllPendingClasses', 'classFactory']
def _check_numpy_compatibility():
    """
    Warns if NumPy is too new for the pybind11 the bindings were built with.
    
        pybind11 < 2.12 does not support NumPy >= 2: passing NumPy arrays to MRPT
        functions then crashes. This happens when a pip-installed NumPy 2
        shadows the system one.
        
    """
