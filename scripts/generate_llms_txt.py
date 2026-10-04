#!/usr/bin/env python3
"""Generates llms.txt and llms-full.txt (https://llmstxt.org/) for the docs
root: an overview of MRPT for AI agents and tools, with links to the Markdown
version of the docs pages.

- llms.txt: how to find/link/import MRPT, modules with their key classes
  (those most used in the examples), the Python packages and the example index.
- llms-full.txt: also every public C++ class (with its Doxygen brief, if the
  Doxygen XML output exists), every Python class and every example.

Invoked by doc/Makefile after Doxygen (it reads doc/source/xml-dir/).
"""

import ast
import collections
import glob
import os
import re
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import generate_rst_docs_examples as gen  # noqa: E402

ROOT = gen.ROOT
DOC_SRC = gen.DOC_SRC
BASE = gen.DOCS_BASE_URL
XML_DIR = os.path.join(DOC_SRC, "xml-dir")
KEY_CLASSES_PER_MODULE = 8

# Modules with no C++ API of interest to users:
SKIP_MODULES = {"mrpt_common", "mrpt_data", "mrpt_imgui_vendor", "mrpt_libapps_cli", "mrpt_libapps_gui"}

INTRO = """# MRPT

> The Mobile Robot Programming Toolkit (MRPT) provides portable C++17 libraries,
> with Python bindings, for mobile robotics: SE(2)/SE(3) geometry and
> uncertainty, metric maps (point clouds, occupancy grids, voxels), SLAM and
> localization, Bayesian filtering, motion planning and reactive navigation,
> sensor drivers and datasets, and 3D visualization. BSD-3-Clause license.
> These docs are for MRPT {version}.

Key facts for writing code with MRPT 3.x:

- Each library is an independent CMake (colcon) package `mrpt_<module>`:
  `find_package(mrpt_poses REQUIRED)` and link the target `mrpt::mrpt_poses`.
  Its dependencies are found and linked transitively.
- Headers are `#include <mrpt/<module>/<Class>.h>`, declaring `mrpt::<module>::<Class>`.
  Classes deriving from `mrpt::rtti::CObject` have `::Ptr` / `::ConstPtr`
  (`std::shared_ptr`) aliases and a static `Create(...)` factory.
- Python: one package per module, e.g. `from mrpt.poses import CPose3D`
  (Debian/Ubuntu packages `python3-mrpt-<module>`), with type stubs.
- MRPT 3.x renamed packages, targets and namespaces of 2.x (e.g. 3D scene
  classes moved from `mrpt::opengl` to `mrpt::viz`); code written for 2.x
  needs porting: see the porting guide below.
- Example data files (datasets, config files) are in the `mrpt_data` package.
"""


def md(page):
    return BASE + page + ".md"


def module_briefs():
    """{"mrpt_poses": "SE(2)/SE(3) poses..."} from doxygen-docs/lib_mrpt_*.md"""
    briefs = {}
    for f in glob.glob(os.path.join(DOC_SRC, "doxygen-docs", "lib_mrpt_*.md")):
        lines = open(f, encoding="utf-8").read().splitlines()
        m = re.match(r"\\defgroup (mrpt_\w+)_grp", lines[0]) if lines else None
        if not m:
            continue
        paragraph = []
        for line in lines[1:]:
            if not line.strip():
                if paragraph:
                    break
                continue
            paragraph.append(line.strip())
        briefs[m.group(1)] = " ".join(paragraph).rstrip(".,") + "."
    return briefs


def cpp_modules():
    mods = []
    for d in sorted(glob.glob(os.path.join(ROOT, "modules", "mrpt_*"))):
        name = os.path.basename(d)
        if name not in SKIP_MODULES and os.path.isdir(os.path.join(d, "include")):
            mods.append(name)
    return mods


def doxygen_classes():
    """{"mrpt_poses": [("mrpt::poses::CPose3D", "brief", "class_mrpt_poses_CPose3D")]}"""
    classes = collections.defaultdict(list)
    modules = set(cpp_modules())
    index = os.path.join(XML_DIR, "index.xml")
    if not os.path.isfile(index):
        return classes
    for compound in ET.parse(index).getroot().findall("compound"):
        if compound.get("kind") not in ("class", "struct"):
            continue
        name = compound.findtext("name")
        if not name.startswith("mrpt::") or "<" in name:
            continue
        xml_file = os.path.join(XML_DIR, compound.get("refid") + ".xml")
        try:
            cdef = ET.parse(xml_file).getroot().find("compounddef")
        except (ET.ParseError, FileNotFoundError):
            continue
        if cdef.get("prot") != "public":
            continue
        location = cdef.find("location")
        header = location.get("file", "") if location is not None else ""
        m = re.match(r"mrpt/(\w+)/", header)  # paths relative to include/
        if not m or "mrpt_" + m.group(1) not in modules:
            continue
        brief = " ".join("".join(cdef.find("briefdescription").itertext()).split())
        page = "{}_{}".format(compound.get("kind"), name.replace("::", "_"))
        classes["mrpt_" + m.group(1)].append((name, brief, page))
    return classes


def python_packages():
    """{"poses": [("CPose3D", "docstring summary"), ...]} from the .pyi stubs."""
    pkgs = {}
    for init in sorted(glob.glob(os.path.join(ROOT, "modules", "mrpt_*", "python", "mrpt", "*", "__init__.pyi"))):
        pkg_dir = os.path.dirname(init)
        name = os.path.basename(pkg_dir)
        docs = {}
        for stub in glob.glob(os.path.join(pkg_dir, "**", "*.pyi"), recursive=True):
            for node in ast.parse(open(stub, encoding="utf-8").read()).body:
                if isinstance(node, ast.ClassDef):
                    doc = (ast.get_docstring(node) or "").strip().split("\n")[0]
                    docs[node.name] = "" if doc.startswith("Members:") else doc
        names = []
        for node in ast.parse(open(init, encoding="utf-8").read()).body:
            if isinstance(node, ast.ImportFrom) and node.module and "_bindings" in node.module:
                names += [a.asname or a.name for a in node.names if a.name in docs]
        pkgs[name] = [(n, docs[n]) for n in names]
    return pkgs


def key_classes(examples):
    """Classes most used in the C++ examples, per module."""
    counts = collections.Counter(c for ex in examples if ex["language"] == "cpp" for c in ex["classes"])
    per_module = collections.defaultdict(list)
    for cls, _ in counts.most_common():
        parts = cls.split("::")
        module = "mrpt_" + parts[1] if len(parts) == 3 else "mrpt_core"
        if len(per_module[module]) < KEY_CLASSES_PER_MODULE:
            per_module[module].append(cls)
    return per_module


def package_description(mod):
    text = open(os.path.join(ROOT, "modules", mod, "package.xml"), encoding="utf-8").read()
    m = re.search(r"<description>(.*?)</description>", text, re.DOTALL)
    return " ".join(m.group(1).split()).rstrip(".") + "." if m else ""


def example_line(ex):
    req = " (requires: {})".format(", ".join(ex["requires"])) if ex["requires"] else ""
    lang = "C++" if ex["language"] == "cpp" else "Python"
    summary = ex["summary"].replace("``", "`")
    return "- [{} ({})]({}): {}{}".format(ex["name"], lang, md(ex["page"]), summary, req)


def write(path, text):
    with open(path, "w", encoding="utf-8") as f:
        f.write(text.rstrip("\n") + "\n")


def main():
    version = open(os.path.join(ROOT, "modules", "mrpt_common", "package.xml"), encoding="utf-8").read()
    version = re.search(r"<version>([\d.]+)</version>", version).group(1)
    examples = gen.collect_cpp() + gen.collect_py()
    briefs = module_briefs()
    keys = key_classes(examples)
    py = python_packages()
    dox = doxygen_classes()

    head = INTRO.format(version=version)
    head += """
## Getting started

- [Installing MRPT]({install}): binary packages for Ubuntu/Debian, ROS and Windows.
- [Building from sources]({compiling})
- [Using MRPT from CMake]({cmake})
- [Porting code from MRPT 2.x to 3.x]({port3})
- [Tutorials]({tutorials})
- [Applications]({apps}): ready-to-use programs (SLAM, rawlog tools, viewers...).
""".format(
        install=md("download-mrpt"),
        compiling=md("compiling"),
        cmake=md("mrpt_from_cmake"),
        port3=md("page_porting_mrpt3"),
        tutorials=md("tutorials"),
        apps=md("applications"),
    )

    modules_short = "\n## C++ modules\n\n"
    modules_full = "\n## C++ modules\n"
    for mod in cpp_modules():
        short = mod[len("mrpt_") :]
        if mod in briefs:
            brief = briefs[mod]
            page = md("group_{}_grp".format(mod))
        else:
            brief = package_description(mod)
            page = "https://github.com/MRPT/mrpt/tree/develop/modules/" + mod
        key = keys.get(mod, [])
        line = "- [{}]({}): {}".format(mod, page, brief)
        if short in py:
            line += " Python: `mrpt.{}`.".format(short)
        if key:
            line += " Key classes: " + ", ".join("`{}`".format(k) for k in key) + "."
        modules_short += line + "\n"

        modules_full += "\n### {}\n\n{}\n\n".format(mod, brief)
        modules_full += "- Docs: {}\n- CMake: `find_package({} REQUIRED)`, target `mrpt::{}`\n".format(page, mod, mod)
        if short in py:
            modules_full += "- Python: `import mrpt.{}` ({})\n".format(short, md("python_api/mrpt/{}/index".format(short)))
        if dox.get(mod):
            modules_full += "\nC++ classes:\n\n"
            for name, cbrief, cpage in sorted(dox[mod]):
                modules_full += "- [`{}`]({}){}\n".format(name, md(cpage), ": " + cbrief if cbrief else "")
        if py.get(short):
            modules_full += "\nPython classes (`mrpt.{}`):\n\n".format(short)
            for name, doc in py[short]:
                modules_full += "- `{}`{}\n".format(name, ": " + doc if doc else "")

    python = """
## Python API

- [Python API reference]({ref}): one page per package, generated from the type stubs.
- [Python examples]({ex})
""".format(ref=md("python_api"), ex=md("python_examples"))

    ex_short = """
## Examples

- [examples.json]({json}): machine-readable index of all C++ and Python
  examples: summary, modules, classes used, and whether they need a GUI,
  hardware or an external dataset.
- [C++ examples]({cpp}): {ncpp} examples, by category.
- [Python examples]({py}): {npy} examples, by category.
""".format(
        json=BASE + "examples.json",
        cpp=md("examples"),
        py=md("python_examples"),
        ncpp=sum(1 for e in examples if e["language"] == "cpp"),
        npy=sum(1 for e in examples if e["language"] == "python"),
    )
    ex_full = ex_short + "\n" + "\n".join(example_line(e) for e in examples) + "\n"

    optional = """
## Optional

- [llms-full.txt]({full}): this file plus all C++ and Python classes and all examples.
- [C++ API index]({api})
- [Changelog]({changelog})
- [Source code](https://github.com/MRPT/mrpt)
""".format(full=BASE + "llms-full.txt", api=md("doxygen-index"), changelog=md("page_changelog"))

    os.makedirs(gen.EXTRA_DIR, exist_ok=True)
    write(os.path.join(gen.EXTRA_DIR, "llms.txt"), head + modules_short + python + ex_short + optional)
    write(os.path.join(gen.EXTRA_DIR, "llms-full.txt"), head + python + modules_full + ex_full)


if __name__ == "__main__":
    main()
