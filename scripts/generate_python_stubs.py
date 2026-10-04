#!/usr/bin/env python3
"""Regenerates the Python type stubs (.pyi) of all MRPT Python modules with
pybind11-stubgen, writing them next to each module's __init__.py
(modules/mrpt_<name>/python/mrpt/<name>/), along with a PEP 561 py.typed
marker. The stubs are committed to git: they are the source of the Python API
reference docs, and CI checks they are up to date.

Usage (after building all modules, from the repository root):

    pip install pybind11-stubgen==3.0.0
    . install/setup.bash
    scripts/generate_python_stubs.py [module ...]

Stubs depend on the pybind11 version used to build the bindings: CI
regenerates them with the version of its Linux job, so use those stubs (or the
"python-stubs" artifact of a failed CI check) if your local ones differ.
"""

import glob
import os
import re
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def python_modules():
    """Map: python module name (e.g. "poses") -> its package dir in the sources."""
    mods = {}
    for init in sorted(glob.glob(os.path.join(ROOT, "modules", "mrpt_*", "python", "mrpt", "*", "__init__.py"))):
        pkg_dir = os.path.dirname(init)
        mods[os.path.basename(pkg_dir)] = pkg_dir
    return mods


def clean_stub(path):
    """Removes private module-level names (package code variables, build
    details such as the pybind11 version) and makes re-exports explicit."""
    with open(path, encoding="utf-8") as f:
        lines = f.readlines()
    lines = [
        # Explicit re-export ("X as X"), as type checkers expect in stubs:
        re.sub(r"^(from mrpt\.\w+\._bindings import )(\w+)$", r"\1\2 as \2", line)
        for line in lines
        if not re.match(r"^(__warningregistry__|_(?!_)\w*) *[:=]", line)
    ]
    with open(path, "w", encoding="utf-8") as f:
        f.writelines(lines)


UNNAMED_MARK = "  # unnamed C++ type"


def replace_unnamed_types(path):
    """Makes annotations valid and independent of the pybind11 version:
    pybind11-stubgen writes "..." for C++ types with no Python binding, which
    becomes typing.Any with a mark, so scripts/check_python_docstrings.py can
    list them; C++ fixed-size arrays become plain lists."""
    with open(path, encoding="utf-8") as f:
        text = f.read()
    lines = []
    for line in text.split("\n"):
        if re.match(r"\s*(def |\w+: )", line):
            # C++ fixed-size arrays as plain lists, the only form every
            # pybind11 version can be reduced to. Older pybind11 writes
            # "list[T[3]]"; newer, an Annotated[] needing pybind11-stubgen:
            line = re.sub(r"\blist\[([\w.]+)\[\d+\]\]", r"list[\1]", line)
            line = re.sub(r"\b([\w.]+)\[\d+\]", r"list[\1]", line)
            line = re.sub(r"typing\.Annotated\[(.+?), pybind11_stubgen\.typing_ext\.\w+\([^)]*\)\]", r"\1", line)
            new = re.sub(r"(: |-> |\[)\.\.\.(?=[,)=\]:]|$)", r"\1typing.Any", line)
            if new != line:
                line = new + UNNAMED_MARK
        lines.append(line)
    new_text = "\n".join(lines)
    if "pybind11_stubgen." not in new_text.replace("import pybind11_stubgen.typing_ext\n", ""):
        new_text = new_text.replace("import pybind11_stubgen.typing_ext\n", "")
    if new_text != text:
        if not re.search(r"^import typing$", new_text, re.MULTILINE):
            new_text = new_text.replace("from __future__ import annotations\n", "from __future__ import annotations\nimport typing\n", 1)
        with open(path, "w", encoding="utf-8") as f:
            f.write(new_text)


def use_public_names(mods, selected):
    """Rewrites "mrpt.X._bindings.Name" as "mrpt.X.Name" in the stubs of the
    selected modules, for the names that the public package re-exports."""
    exported = {}
    for name, pkg_dir in mods.items():
        init = os.path.join(pkg_dir, "__init__.pyi")
        if os.path.isfile(init):
            text = open(init, encoding="utf-8").read()
            exported[name] = set(re.findall(r"^from mrpt\.\w+\._bindings import (\w+)", text, re.MULTILINE))

    def repl(m):
        if m.group(2) in exported.get(m.group(1), ()):
            return "mrpt.{}.{}".format(m.group(1), m.group(2))
        return m.group(0)

    for name in selected:
        for stub in glob.glob(os.path.join(mods[name], "**", "*.pyi"), recursive=True):
            text = open(stub, encoding="utf-8").read()
            new = re.sub(r"\bmrpt\.(\w+)\._bindings\.(\w+)\b", repl, text)
            # Imports no longer needed:
            for mod in set(re.findall(r"^import mrpt\.(\w+)\._bindings$", new, re.MULTILINE)):
                if "mrpt.{}._bindings.".format(mod) not in new:
                    new = re.sub(r"^import mrpt\.{}\._bindings$".format(mod), "import mrpt." + mod, new, flags=re.MULTILINE)
            if new != text:
                with open(stub, "w", encoding="utf-8") as f:
                    f.write(new)


def main():
    mods = python_modules()
    selected = sys.argv[1:] or list(mods)
    failed = []
    with tempfile.TemporaryDirectory() as tmp:
        for name in selected:
            if name not in mods:
                sys.exit("Unknown module '{}'. Available: {}".format(name, " ".join(mods)))
            print("Generating stubs for mrpt.{}...".format(name), flush=True)
            r = subprocess.run(
                [
                    sys.executable,
                    "-m",
                    "pybind11_stubgen",
                    "--numpy-array-remove-parameters",
                    "-o",
                    tmp,
                    "mrpt." + name,
                ],
                env=dict(os.environ, PYTHONWARNINGS="ignore"),
            )
            out_dir = os.path.join(tmp, "mrpt", name)
            if r.returncode != 0 or not os.path.isdir(out_dir):
                failed.append(name)
                continue
            for stub in glob.glob(os.path.join(out_dir, "**", "*.pyi"), recursive=True):
                clean_stub(stub)
                replace_unnamed_types(stub)

            pkg_dir = mods[name]
            for old in glob.glob(os.path.join(pkg_dir, "**", "*.pyi"), recursive=True):
                os.remove(old)
            shutil.copytree(out_dir, pkg_dir, dirs_exist_ok=True)
            open(os.path.join(pkg_dir, "py.typed"), "w").close()

    use_public_names(mods, [n for n in selected if n not in failed])

    if failed:
        sys.exit("Failed to generate stubs for: " + " ".join(failed))


if __name__ == "__main__":
    main()
