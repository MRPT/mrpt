#!/usr/bin/env python3
"""Lists the classes, functions, methods and properties of the MRPT Python
bindings with no docstring, and the signatures where pybind11 could not name a
C++ type (typing.Any marked as "unnamed C++ type" in the stubs), by parsing the committed .pyi stubs
(see scripts/generate_python_stubs.py). No MRPT build is needed.

Usage:
    scripts/check_python_docstrings.py [--summary] [module ...]
"""

import argparse
import ast
import glob
import os

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

# Special methods whose meaning is obvious without a docstring:
SELF_EXPLANATORY = {
    "__repr__",
    "__str__",
    "__eq__",
    "__ne__",
    "__lt__",
    "__le__",
    "__gt__",
    "__ge__",
    "__hash__",
    "__len__",
    "__iter__",
    "__next__",
    "__getitem__",
    "__setitem__",
    "__contains__",
    "__enter__",
    "__exit__",
    "__getstate__",
    "__setstate__",
    "__copy__",
    "__deepcopy__",
    "__index__",
    "__int__",
    "__float__",
    "__bool__",
    "__add__",
    "__iadd__",
    "__sub__",
    "__isub__",
    "__mul__",
    "__imul__",
    "__neg__",
    "__array__",
    "__dir__",
}


def stub_files(modules):
    files = sorted(glob.glob(os.path.join(ROOT, "modules", "mrpt_*", "python", "mrpt", "*", "**", "*.pyi"), recursive=True))
    if modules:
        files = [f for f in files if f.split(os.sep + "mrpt" + os.sep)[-1].split(os.sep)[0] in modules]
    return files


def is_enum(cls):
    return any(isinstance(n, (ast.Assign, ast.AnnAssign)) and "__members__" in ast.unparse(n) for n in cls.body)


def check_file(path):
    """Returns (undocumented, unnamed_types): lists of qualified names."""
    rel = os.path.relpath(path, ROOT)
    pkg = "mrpt." + rel.split(os.sep + "mrpt" + os.sep)[-1].split(os.sep)[0]
    text = open(path, encoding="utf-8").read()
    lines = text.split("\n")
    tree = ast.parse(text)
    undocumented = []
    unnamed = []

    def visit(nodes, prefix):
        for node in nodes:
            if isinstance(node, ast.ClassDef):
                qual = prefix + node.name
                if not ast.get_docstring(node) and not is_enum(node):
                    undocumented.append(qual)
                if not is_enum(node):
                    visit(node.body, qual + ".")
            elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                qual = prefix + node.name
                sig_end = node.body[0].lineno - 1 if node.body else node.lineno
                if any("# unnamed C++ type" in line for line in lines[node.lineno - 1 : sig_end]):
                    unnamed.append(qual)
                if node.name in SELF_EXPLANATORY:
                    continue
                if node.name.startswith("_") and node.name != "__init__":
                    continue
                # Property setters share the getter docstring:
                if any(isinstance(d, ast.Attribute) and d.attr in ("setter", "deleter") for d in node.decorator_list):
                    continue
                if not ast.get_docstring(node) and qual not in undocumented:
                    undocumented.append(qual)

    visit(tree.body, pkg + ".")
    return undocumented, sorted(set(unnamed))


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--summary", action="store_true", help="only print the counts per module")
    parser.add_argument("modules", nargs="*", help="Python module names (e.g. poses maps)")
    args = parser.parse_args()

    files = stub_files(args.modules)
    if not files:
        raise SystemExit("No .pyi stubs found: run scripts/generate_python_stubs.py first.")

    total_undoc = 0
    total_unnamed = 0
    for path in files:
        undocumented, unnamed = check_file(path)
        total_undoc += len(undocumented)
        total_unnamed += len(unnamed)
        if not undocumented and not unnamed:
            continue
        print("{}: {} undocumented, {} with unnamed C++ types".format(os.path.relpath(path, ROOT), len(undocumented), len(unnamed)))
        if args.summary:
            continue
        for name in undocumented:
            print("  no docstring: " + name)
        for name in unnamed:
            print("  unnamed type: " + name)
    print("Total: {} undocumented, {} with unnamed C++ types".format(total_undoc, total_unnamed))


if __name__ == "__main__":
    main()
