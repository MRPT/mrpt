"""
Python bindings for mrpt_containers
"""
from __future__ import annotations
import typing
__all__: list[str] = ['YAML']
class YAML:
    """
    Powerful YAML/JSON container for nested structured data.
    
    **Thread safety:** Not thread-safe. Concurrent reads are safe when no writer
    is active; any concurrent write requires external synchronization (same policy
    as std::map / std::vector).
    """
    @staticmethod
    def from_dict(d: dict) -> YAML:
        """
        Build a YAML map node from a Python dict (recursive)
        """
    @staticmethod
    def from_file(arg0: str) -> YAML:
        """
        Parse YAML/JSON from file
        """
    @staticmethod
    def from_list(lst: list) -> YAML:
        """
        Build a YAML sequence node from a Python list (recursive)
        """
    @staticmethod
    def from_string(arg0: str) -> YAML:
        """
        Parse YAML/JSON from string
        """
    def __contains__(self, key: str) -> bool:
        """
        Support Python 'in' operator for map keys
        """
    def __delitem__(self, key: typing.Any) -> None:
        """
        Delete a map entry by key or sequence element by index
        """
    def __getitem__(self, key: typing.Any) -> YAML:
        """
        Access child node by string key (map) or int index (sequence)
        """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def __iter__(self) -> typing.Any:
        """
        Iterate over map keys (for maps) or child nodes (for sequences)
        """
    def __len__(self) -> int:
        ...
    def __repr__(self) -> str:
        ...
    def __setitem__(self, key: typing.Any, value: typing.Any) -> None:
        ...
    def __str__(self) -> str:
        ...
    def append(self, value: typing.Any) -> None:
        """
        Append any Python value (str/int/float/bool/dict/list) to a sequence
        """
    def as_bool(self) -> bool:
        """
        Extract scalar value as bool
        """
    def as_float(self) -> float:
        """
        Extract scalar value as float
        """
    def as_int(self) -> int:
        """
        Extract scalar value as int
        """
    def as_str(self) -> str:
        """
        Extract scalar value as string
        """
    def clear(self) -> None:
        """
        Clear all contents
        """
    def empty(self) -> bool:
        """
        True if node is empty
        """
    def get_bool(self, key: str, default: bool = False) -> bool:
        """
        Get bool value for key, with optional default
        """
    def get_float(self, key: str, default: float = 0.0) -> float:
        """
        Get float value for key, with optional default
        """
    def get_int(self, key: str, default: int = 0) -> int:
        """
        Get int value for key, with optional default
        """
    def get_str(self, key: str, default: str = '') -> str:
        """
        Get string value for key, with optional default
        """
    def has(self, arg0: str) -> bool:
        """
        Check if a key exists in a map node
        """
    def isMap(self) -> bool:
        """
        True if this node is a map (dict-like)
        """
    def isNullNode(self) -> bool:
        """
        True if this node is null/empty
        """
    def isScalar(self) -> bool:
        """
        True if this node holds a scalar value
        """
    def isSequence(self) -> bool:
        """
        True if this node is a sequence (list-like)
        """
    def items(self) -> list:
        """
        Return list of (key, YAML) pairs
        """
    def keys(self) -> list[str]:
        """
        Return list of map keys
        """
    def push_back(self, value: float) -> None:
        """
        Append a numeric value to a sequence node
        """
    def push_back_str(self, value: str) -> None:
        """
        Append a string value to a sequence node
        """
    def save_to_file(self, fileName: str) -> None:
        """
        Save YAML to a file
        """
    def size(self) -> int:
        """
        Number of elements
        """
    def to_dict(self) -> typing.Any:
        """
        Recursively convert a map node to a Python dict
        """
    def to_list(self) -> typing.Any:
        """
        Recursively convert a sequence node to a Python list
        """
    def to_string(self) -> str:
        """
        Dump YAML to string
        """
    def values(self) -> list:
        """
        Return list of map values as YAML nodes
        """
