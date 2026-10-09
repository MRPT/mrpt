"""
Python bindings for mrpt_core
"""
from __future__ import annotations
import datetime
import typing
__all__: list[str] = ['Clock', 'Stringifyable', 'WorkerThreadsPool', 'abs_diff_double', 'abs_diff_float', 'abs_diff_int', 'abs_diff_long', 'deg2rad', 'format', 'format1d', 'format1s', 'from_string_double', 'from_string_int', 'get_env', 'get_env_double', 'get_env_int', 'rad2deg', 'reverse_bytes_i16', 'reverse_bytes_i32', 'reverse_bytes_i64', 'reverse_bytes_u16', 'reverse_bytes_u32', 'reverse_bytes_u64']
class Stringifyable:
    """
    Interface for classes whose state can be represented as a human-friendly text.
    """
    def asString(self) -> str:
        """
        Returns a human-friendly textual description of the object.
        """
class Clock:
    """
    C++11-clock that is compatible with MRPT TTimeStamp representation.
    """
    @staticmethod
    def fromDouble(seconds: float) -> datetime.timedelta:
        """
        Convert double seconds to mrpt::Clock::time_point
        """
    @staticmethod
    def now() -> datetime.timedelta:
        """
        Current time as mrpt::Clock::time_point
        """
    @staticmethod
    def nowDouble() -> float:
        """
        Current time in seconds (double)
        """
    @staticmethod
    def toDouble(time_point: datetime.timedelta) -> float:
        """
        Convert mrpt::Clock::time_point to double seconds
        """
class WorkerThreadsPool:
    """
    A simple and efficient thread pool for parallel task execution.
    """
    def __init__(self, num_threads: int) -> None:
        """
        Creates a pool with the given number of worker threads.
        """
    def clear(self) -> None:
        """
        Stops all worker threads and clears the pool.
        """
    def enqueue(self, func: typing.Callable) -> None:
        """
        Enqueues a task for execution by a worker thread.
        """
    def pendingTasks(self) -> int:
        """
        Returns the number of tasks waiting in the queue.
        """
    def resize(self, arg0: int) -> None:
        """
        Adds worker threads to the pool.
        """
    def size(self) -> int:
        """
        Returns the number of worker threads in the pool.
        """
    @property
    def name(self) -> str:
        """
        Pool name property
        """
    @name.setter
    def name(self, arg1: str) -> None:
        ...
def abs_diff_double(arg0: float, arg1: float) -> float:
    """
    Absolute difference (double)
    """
def abs_diff_float(arg0: float, arg1: float) -> float:
    """
    Absolute difference (float)
    """
def abs_diff_int(arg0: int, arg1: int) -> int:
    """
    Absolute difference (int)
    """
def abs_diff_long(arg0: int, arg1: int) -> int:
    """
    Absolute difference (long)
    """
def deg2rad(arg0: float) -> float:
    """
    Degrees to radians (double)
    """
def format(fmt: str, *args) -> str:
    """
    Identity overload — returns the string as-is. Use Python f-strings for formatting instead.
    """
def format1d(fmt: str, val: float) -> str:
    """
    Printf-style format with one double argument
    """
def format1s(fmt: str, val: str) -> str:
    """
    Printf-style format with one string argument
    """
def from_string_double(s: str, default_value: float = 0.0, throw_on_error: bool = False) -> float:
    """
    Parse string to double; returns default_value on failure unless throw_on_error is True
    """
def from_string_int(s: str, default_value: int = 0, throw_on_error: bool = False) -> int:
    """
    Parse string to int; returns default_value on failure unless throw_on_error is True
    """
def get_env(var_name: str, default_val: str = '') -> str:
    """
    Read an environment variable as string; returns default_val if not set
    """
def get_env_double(var_name: str, default_val: float = 0.0) -> float:
    """
    Read an environment variable as double; returns default_val if not set
    """
def get_env_int(var_name: str, default_val: int = 0) -> int:
    """
    Read an environment variable as int; returns default_val if not set
    """
def rad2deg(arg0: float) -> float:
    """
    Radians to degrees (double)
    """
def reverse_bytes_i16(arg0: int) -> int:
    """
    Reverse endianness for int16
    """
def reverse_bytes_i32(arg0: int) -> int:
    """
    Reverse endianness for int32
    """
def reverse_bytes_i64(arg0: int) -> int:
    """
    Reverse endianness for int64
    """
def reverse_bytes_u16(arg0: int) -> int:
    """
    Reverse endianness for uint16
    """
def reverse_bytes_u32(arg0: int) -> int:
    """
    Reverse endianness for uint32
    """
def reverse_bytes_u64(arg0: int) -> int:
    """
    Reverse endianness for uint64
    """
