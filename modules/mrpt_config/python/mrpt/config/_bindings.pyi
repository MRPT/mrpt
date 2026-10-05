"""
Python bindings for mrpt_config
"""
from __future__ import annotations
import typing
__all__: list[str] = ['CConfigFile', 'CConfigFileBase', 'CConfigFileMemory', 'CLoadableOptions', 'MRPT_SAVE_NAME_PADDING', 'MRPT_SAVE_VALUE_PADDING', 'config_parser']
class CConfigFileBase:
    """
    Base class of configuration files (INI-like or YAML), with sections and key/value pairs.
    """
    def clear(self) -> None:
        """
        Empties the config file.
        """
    def getAllKeys(self, arg0: str) -> list[str]:
        """
        Returns a list with all keys in a section.
        """
    def getAllSections(self) -> list[str]:
        """
        Returns a list with all section names.
        """
    def getContentAsYAML(self) -> str:
        """
        Returns content as a YAML block.
        """
    def keyExists(self, arg0: str, arg1: str) -> bool:
        """
        Checks if a key exists in a section.
        """
    def read_bool(self, section: str, name: str, defValue: bool, failIfNotFound: bool = False) -> bool:
        """
        Reads a bool value with an optional default value.
        """
    def read_double(self, section: str, name: str, defValue: float, failIfNotFound: bool = False) -> float:
        """
        Reads a double value with an optional default value.
        """
    def read_float(self, section: str, name: str, defValue: float, failIfNotFound: bool = False) -> float:
        """
        Reads a float value with an optional default value.
        """
    def read_int(self, section: str, name: str, defValue: int, failIfNotFound: bool = False) -> int:
        """
        Reads an integer value with an optional default value.
        """
    def read_string(self, section: str, name: str, defValue: str = '', failIfNotFound: bool = False) -> str:
        """
        Reads a string value with an optional default value.
        """
    def read_string_first_word(self, section: str, name: str, defValue: str = '', failIfNotFound: bool = False) -> str:
        """
        Reads the first word of a string value with an optional default value.
        """
    def read_uint32_t(self, section: str, name: str, defValue: int, failIfNotFound: bool = False) -> int:
        """
        Reads a 32-bit unsigned integer value with an optional default value.
        """
    def read_uint64_t(self, section: str, name: str, defValue: int, failIfNotFound: bool = False) -> int:
        """
        Reads a 64-bit unsigned integer value with an optional default value.
        """
    def sectionExists(self, arg0: str) -> bool:
        """
        Checks if a section exists.
        """
    def setContentFromYAML(self, arg0: str) -> None:
        """
        Sets content from a YAML block.
        """
    @typing.overload
    def write(self, section: str, name: str, value: float, name_padding_width: int = -1, value_padding_width: int = -1, comment: str = '') -> None:
        """
        Writes a numeric value into the given section and key.
        """
    @typing.overload
    def write(self, section: str, name: str, value: str, name_padding_width: int = -1, value_padding_width: int = -1, comment: str = '') -> None:
        """
        Writes a string value into the given section and key.
        """
class CLoadableOptions:
    """
    This is a virtual base class for sets of options than can be loaded from and/or saved to configuration plain-text files.
    """
    def dumpToConsole(self) -> None:
        """
        Dumps options to the console.
        """
    def dumpToString(self) -> str:
        """
        Returns the options as human-readable text, as dumpToConsole() prints them.
        """
    def loadFromConfigFile(self, source: CConfigFileBase, section: str) -> None:
        """
        Loads options from a configuration file section.
        """
    def loadFromConfigFileName(self, arg0: str, arg1: str) -> None:
        """
        Loads options directly from a file name.
        """
    def saveToConfigFile(self, target: CConfigFileBase, section: str) -> None:
        """
        Saves options to a configuration file section.
        """
    def saveToConfigFileName(self, arg0: str, arg1: str) -> None:
        """
        Saves options directly to a file name.
        """
class CConfigFile(CConfigFileBase):
    """
    This class allows loading and storing values and vectors of different types from ".ini" files easily.
    """
    @typing.overload
    def __init__(self, arg0: str) -> None:
        """
        Constructor for a file.
        """
    @typing.overload
    def __init__(self) -> None:
        """
        Empty constructor.
        """
    def discardSavingChanges(self) -> None:
        """
        Discards saving changes to the physical file.
        """
    def getAssociatedFile(self) -> str:
        """
        Returns the associated file name.
        """
    def setFileName(self, arg0: str) -> None:
        """
        Associates the object with a file.
        """
    def writeNow(self) -> None:
        """
        Writes changes to the physical file immediately.
        """
class CConfigFileMemory(CConfigFileBase):
    """
    A configuration file kept in memory, initialized from a text string.
    """
    @typing.overload
    def __init__(self) -> None:
        """
        Empty constructor.
        """
    @typing.overload
    def __init__(self, arg0: list[str]) -> None:
        """
        Constructor with a list of strings.
        """
    @typing.overload
    def __init__(self, arg0: str) -> None:
        """
        Constructor with a single string.
        """
    def getContent(self) -> str:
        """
        Returns the current content.
        """
    @typing.overload
    def setContent(self, arg0: list[str]) -> None:
        """
        Sets content from a list of strings.
        """
    @typing.overload
    def setContent(self, arg0: str) -> None:
        """
        Sets content from a single string.
        """
def MRPT_SAVE_NAME_PADDING() -> int:
    """
    Default padding for names.
    """
def MRPT_SAVE_VALUE_PADDING() -> int:
    """
    Default padding for values.
    """
def config_parser(arg0: str) -> str:
    """
    Parses a configuration document.
    """
