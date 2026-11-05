"""C++ SDK shipped inside the MRPT wheel: headers, shared libraries and CMake
config files, so other wheels can build C++ code against this MRPT build.

With scikit-build-core, ``find_package(mrpt_poses)`` (or any other module)
works out of the box, since this package is registered as a ``cmake.prefix``
entry point. Other build systems can pass ``get_cmake_prefix_path()`` in
``CMAKE_PREFIX_PATH``.

C++ extensions built this way must use the same pybind11 version as this
wheel (``mrpt.sdk.PYBIND11_VERSION``) to share Python types with it.
"""

from pathlib import Path

from ._build_info import MRPT_VERSION, PYBIND11_VERSION  # noqa: F401

_ROOT = Path(__file__).resolve().parent


def get_cmake_prefix_path() -> str:
    """Directory to add to CMAKE_PREFIX_PATH to find the MRPT CMake packages."""
    return str(_ROOT)


def get_include() -> str:
    """Directory with the MRPT C++ headers."""
    return str(_ROOT / "include")


def get_library_dir() -> str:
    """Directory with the MRPT shared libraries."""
    return str(_ROOT / "lib")

