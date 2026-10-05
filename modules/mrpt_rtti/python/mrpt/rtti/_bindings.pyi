"""
Python bindings for mrpt_rtti
"""
from __future__ import annotations
import typing
__all__: list[str] = ['CObject', 'TRuntimeClassId', 'classFactory', 'findRegisteredClass', 'getAllRegisteredClasses', 'getAllRegisteredClassesChildrenOf', 'registerAllPendingClasses', 'registerClass', 'registerClassCustomName']
class CObject:
    """
    Virtual base to provide a compiler-independent RTTI system.
    """
    def GetRuntimeClass(self) -> TRuntimeClassId:
        """
        Returns information about the class of an object in runtime.
        """
class TRuntimeClassId:
    """
    Runtime type information of an MRPT class: name, base class and factory.
    """
    def createObject(self) -> CObject:
        """
        Creates a new object of this class (default constructed), or returns None for virtual classes.
        """
    @typing.overload
    def derivedFrom(self, arg0: TRuntimeClassId) -> bool:
        """
        Returns true if this class derives from the given one.
        """
    @typing.overload
    def derivedFrom(self, arg0: str) -> bool:
        """
        Returns true if this class derives from the class with the given name.
        """
    def getBaseClass(self) -> TRuntimeClassId:
        """
        Gets the base class runtime id.
        """
    @property
    def className(self) -> str:
        """
        Name of the class
        """
def classFactory(arg0: str) -> CObject:
    """
    Creates an object given by its registered name.
    """
def findRegisteredClass(className: str, allow_ignore_namespace: bool = True) -> TRuntimeClassId:
    """
    Returns the class with the given name, or None if not registered.
    """
def getAllRegisteredClasses() -> list[TRuntimeClassId]:
    """
    Returns all the registered classes.
    """
def getAllRegisteredClassesChildrenOf(arg0: TRuntimeClassId) -> list[TRuntimeClassId]:
    """
    Like getAllRegisteredClasses(), but filters the list to only include children clases of a given base one.
    """
def registerAllPendingClasses() -> None:
    """
    Registers all pending classes (normally done automatically).
    """
def registerClass(arg0: TRuntimeClassId) -> None:
    """
    Register a class into the MRPT internal list of "CObject" descendents.
    """
def registerClassCustomName(arg0: str, arg1: TRuntimeClassId) -> None:
    """
    Registers a class under an additional name (for backwards-compatible deserialization).
    """
