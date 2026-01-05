#!/usr/bin/env python3
"""
MRPT Python API (Namespace Root)
"""
import pkgutil
import importlib
import os

# 1. This is the "magic" line for PEP 420 namespace packages.
# It allows 'mrpt' to be spread across multiple 'site-packages' directories.
__path__ = pkgutil.extend_path(__path__, __name__)

# 2. Modules we expect to find in different install prefixes
MRPT_MODULES = [
    "core",
    "config",
    "rtti",
    "system",
    "serialization",
    "math",
    "poses",
    "containers",
    "viz",
]

# Set this to True via environment variable if you need deep debugging
DEBUG_MRPT_LOAD = os.environ.get("PYMRPT_DEBUG_LOAD", "0") == "1"

for module_name in MRPT_MODULES:
    try:
        # Import absolutely to ensure we hit the merged namespace correctly
        module = importlib.import_module(f"mrpt.{module_name}")
        globals()[module_name] = module
    except ImportError as e:
        # We only print if it's a 'real' error (like a missing dependency or type error)
        # and ignore cases where the module simply isn't installed.
        if DEBUG_MRPT_LOAD:
            print(f"[MRPT Debug] Failed to load '{module_name}': {e}")
    except Exception as e:
        print(f"Error initializing MRPT module '{module_name}': {e}")

# Clean up temporary variables
del pkgutil, importlib, module_name
