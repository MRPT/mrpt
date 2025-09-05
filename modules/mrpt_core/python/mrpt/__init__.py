#!/usr/bin/env python3

# mrpt/__init__.py
"""
MRPT Python API.
"""

# Extend the path for a namespace package. This allows Python to find
# sub-packages residing in different directories, for example, if they
# are installed by different build systems (e.g., colcon).
__path__ = __import__("pkgutil").extend_path(__path__, __name__)

# List of optional MRPT modules to import.
# Each import is wrapped in a try/except block to ensure the main 'mrpt'
# package loads even if some sub-packages are not present.

try:
    from . import core
except ImportError:
    print("Note: Optional MRPT module 'core' not found. Some functionality will be unavailable.")

try:
    from . import config
except ImportError:
    print("Note: Optional MRPT module 'config' not found. Some functionality will be unavailable.")

try:
    from . import rtti
except ImportError:
    print("Note: Optional MRPT module 'rtti' not found. Some functionality will be unavailable.")

try:
    from . import system
except ImportError:
    print("Note: Optional MRPT module 'system' not found. Some functionality will be unavailable.")

try:
    from . import viz
except ImportError:
    print("Note: Optional MRPT module 'viz' not found. Some functionality will be unavailable.")
