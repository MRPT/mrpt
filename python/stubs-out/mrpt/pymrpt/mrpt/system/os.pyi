from typing import overload

def _strcmp(str1: str, str2: str) -> int: ...
def _strcmpi(str1: str, str2: str) -> int: ...
def _strncmp(str: str, subStr: str, count: int) -> int: ...
def _strnicmp(str: str, subStr: str, count: int) -> int: ...
@overload
def getch() -> int: ...
@overload
def getch() -> int: ...
@overload
def kbhit() -> bool: ...
@overload
def kbhit() -> bool: ...
def memcpy(dest: capsule, destSize: int, src: capsule, copyCount: int) -> None: ...
def strcat(dest: str, destSize: int, source: str) -> str: ...
def strcpy(dest: str, destSize: int, source: str) -> str: ...
