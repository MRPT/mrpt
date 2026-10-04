"""
gzip compression of memory blocks and files
"""
from __future__ import annotations
import typing
__all__: list[str] = ['compress_gz_data_block', 'compress_gz_file', 'decompress_gz_data_block', 'decompress_gz_file']
def compress_gz_data_block(data: bytes, compress_level: int = 9) -> bytes:
    """
    Compress a bytes object into a gzip-format bytes object (level 0-9).
    """
def compress_gz_file(file_path: str, data: bytes, compress_level: int = 9) -> bool:
    """
    Write a bytes object into a gzip file. Returns False on error.
    """
def decompress_gz_data_block(data: bytes) -> bytes:
    """
    Decompress a gzip-format bytes object. Data not in gzip format is returned unmodified.
    """
def decompress_gz_file(file_path: str) -> bytes | None:
    """
    Read a gzip file (or a plain file, unmodified) into bytes. Returns None on error.
    """
