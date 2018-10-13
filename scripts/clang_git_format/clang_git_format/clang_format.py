import difflib
from distutils import spawn
import glob
import os
import sys
import threading
import subprocess

from config import (CLANG_FORMAT_VERSION, CLANG_FORMAT_SHORT_VERSION, PROGNAME)

from utils import (
    callo,
    get_clang_format_from_linux_cache,
    get_clang_format_from_darwin_cache, )

import logging
logger = logging.getLogger("clang-format")


class ClangFormat(object):
    """Class encapsulates finding a suitable copy of clang-format,
    and linting/formating an individual file
    """

    def __init__(self, clang_path, cache_dir):
        """Initializataion method.

        """
        self.clang_path = None
        self.clang_format_progname_ext = ""

        if sys.platform == "win32":
            self.clang_format_progname_ext += ".exe"

        # Check the clang-format the user specified
        if clang_path is not None:
            if os.path.isfile(clang_path):
                self.clang_path = clang_path

        # Check the users' PATH environment variable now
        if self.clang_path is None:
            # Check for various versions staring with binaries with version
            # specific suffixes in the user's path
            programs = [
                PROGNAME + "-" + CLANG_FORMAT_VERSION,
                PROGNAME + "-" + CLANG_FORMAT_SHORT_VERSION,
                PROGNAME,
            ]

            if sys.platform == "win32":
                for i in range(len(programs)):
                    programs[i] += '.exe'

            for program in programs:
                self.clang_path = spawn.find_executable(program)

                if self.clang_path:
                    if not self._validate_version():
                        self.clang_path = None
                    else:
                        break

        # If Windows, try to grab it from Program Files
        # Check both native Program Files and WOW64 version
        if sys.platform == "win32":
            programfiles = [
                os.environ["ProgramFiles"],
                os.environ["ProgramFiles(x86)"],
            ]

            for programfile in programfiles:
                win32bin = os.path.join(programfile,
                                        "LLVM\\bin\\clang-format.exe")
                if os.path.exists(win32bin):
                    self.clang_path = win32bin
                    break

        # Have not found it yet, download it from the web
        if self.clang_path is None:
            if not os.path.isdir(cache_dir):
                os.makedirs(cache_dir)

            self.clang_path = os.path.join(cache_dir,
                                           PROGNAME + "-" + CLANG_FORMAT_VERSION
                                           + self.clang_format_progname_ext)


            # Download a new version if the cache is empty or stale
            if not os.path.isfile(self.clang_path) \
                    or not self._validate_version():

                logger.warn("Haven't found a valid clang version in PATH. "
                            "Downloading a valid version...")

                if sys.platform.startswith("linux"):
                    get_clang_format_from_linux_cache(self.clang_path)
                elif sys.platform == "darwin":
                    get_clang_format_from_darwin_cache(self.clang_path)
                else:
                    logger.error("clang-format.py does not support "
                                 "downloading clang-format " +
                                 " on this platform, please install "
                                 "clang-format " + CLANG_FORMAT_VERSION)

        # Validate we have the correct version
        # We only can fail here if the user specified a clang-format binary and
        # it is the wrong version
        if not self._validate_version():
            logger.error("Exiting because of previous warning.")
            sys.exit(1)

        self.print_lock = threading.Lock()

    def _validate_version(self):
        """Validate clang-format is the expected version
        """
        cf_version = callo([self.clang_path, "--version"])

        # JLBC: Disable version checks and just use the version we find:
        return True

    def _lint(self, file_name, print_diff):
        """Check the specified file has the correct format
        """
        with open(file_name, 'rb') as original_text:
            original_file = original_text.read()

        # Get formatted file as clang-format would format the file
        formatted_file = callo([self.clang_path, "--style=file", file_name])

        if original_file != formatted_file:
            if print_diff:
                original_lines = original_file.splitlines()
                formatted_lines = formatted_file.splitlines()
                result = difflib.unified_diff(original_lines, formatted_lines)

                # Take a lock to ensure diffs do not get mixed when printed to
                # the screen
                with self.print_lock:
                    logger.error("Found diff for " + file_name)
                    logger.info("To fix formatting errors, run %s "
                                "--style=file -i %s" % (self.clang_path,
                                                        file_name))
                    for line in result:
                        logger.info(line.rstrip())

            return False

        return True

    def lint(self, file_name):
        """Check the specified file has the correct format
        """
        return self._lint(file_name, print_diff=True)

    def format_func(self, file_name):
        """Update the format of the specified file
        """
        if self._lint(file_name, print_diff=False):
            return True

        # Update the file with clang-format
        formatted = not subprocess.call(
            [self.clang_path, "--style=file", "-i", file_name])

        # Version 3.8 generates files like foo.cpp~RF83372177.TMP when it
        # formats foo.cpp on Windows, we must clean these up
        if sys.platform == "win32":
            glob_pattern = file_name + "*.TMP"
            for fglob in glob.glob(glob_pattern):
                os.unlink(fglob)

        return formatted
