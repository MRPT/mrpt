#!/usr/bin/env python3
"""Generates the docs pages of all C++ and Python examples, the galleries
listing them (examples.rst, python_examples.rst), and examples.json, a
machine-readable index of all examples for tools and AI agents.
Invoked automatically by doc/Makefile.

Sources of information for each example:
- C++ (mrpt_examples_cpp/<name>/): the first paragraph of README.md is its
  summary.
- Python (mrpt_examples_py/<name>.py): the first line of the script docstring
  is its summary, and <name>.out, if present, its expected output.
- doc/source/images/<name>_screenshot.* is its thumbnail (optional). Examples
  without one get a placeholder thumbnail of their category.
- MRPT modules and classes used are detected from #include lines (C++) and
  imports (Python).
- Optional metadata, in one line of README.md (C++) or the script (Python):
      <!-- mrpt-example: requires=hardware,gui; tags=lidar,icp -->
      # mrpt-example: requires=dataset; video=<youtube_id>
  Keys: "requires" (gui, hardware, dataset; overrides the auto-detected
  value), "tags" (free keywords) and "video" (YouTube video id).
"""

import ast
import glob
import json
import os
import re

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
CPP_DIR = os.path.join(ROOT, "mrpt_examples_cpp")
PY_DIR = os.path.join(ROOT, "mrpt_examples_py")
MODULES_DIR = os.path.join(ROOT, "modules")
DOC_SRC = os.path.join(ROOT, "doc", "source")
OUT_MD_DIR = os.path.join(DOC_SRC, "doxygen-docs")
IMAGES_DIR = os.path.join(DOC_SRC, "images")
EXTRA_DIR = os.path.join(DOC_SRC, "_extra")

DOCS_BASE_URL = "https://docs.mrpt.org/reference/latest/"
GITHUB_BASE_URL = "https://github.com/MRPT/mrpt/blob/develop/"

# Gallery sections: (title, placeholder thumbnail, name prefixes or modules)
CATEGORIES = [
    ("3D graphics and GUIs", "graphics", ["opengl", "gui", "imgui", "viz"]),
    ("Maps", "maps", ["maps"]),
    ("SLAM, Bayesian filtering and graph-SLAM", "slam", ["slam", "bayes", "graphslam"]),
    ("Navigation and graph search", "nav", ["nav", "graphs", "kinematics"]),
    ("Math, poses and geometry", "math", ["math", "poses", "topography", "random", "tfest", "expr"]),
    ("Images", "img", ["img"]),
    ("Sensors, observations and datasets", "sensors", ["obs", "hwdrivers", "kitti", "rgbd"]),
    (
        "Core, containers, I/O and system",
        "core",
        [
            "core",
            "containers",
            "config",
            "io",
            "serialization",
            "rtti",
            "typemeta",
            "system",
            "comms",
        ],
    ),
]
OTHER_CATEGORY = ("Other", "core", [])

# For Python scripts not named "mrpt_<module>_example.py", the category comes
# from the first of these modules that the script imports:
PY_MODULE_PRIORITY = [
    "slam",
    "bayes",
    "nav",
    "graphs",
    "maps",
    "hwdrivers",
    "comms",
    "gui",
    "opengl",
    "viz",
    "obs",
    "img",
    "topography",
    "tfest",
    "kinematics",
    "poses",
    "math",
]

REQUIRES_VALUES = ["gui", "hardware", "dataset"]

CSS = """.. raw:: html

   <style>
   .mrpt-example-thumb { height: 150px !important; object-fit: cover; }
   .mrpt-gallery .sd-card-title { font-size: 0.9em; overflow-wrap: anywhere; }
   .mrpt-gallery .sd-card-text { font-size: 0.9em; }
   </style>

"""

CPP_HEADER = """.. _examples:

===============
C++ examples
===============

The source code for all these C++ examples can be found
under `MRPT/mrpt_examples_cpp <https://github.com/MRPT/mrpt/tree/develop/mrpt_examples_cpp>`_.
They are built along with MRPT when ``MRPT_BUILD_EXAMPLES`` is enabled in CMake.

Python examples are `here <python_examples.html>`_. A machine-readable index
of all examples is available in `examples.json <examples.json>`_.

"""

PY_HEADER = """.. _python_examples:

=================
Python examples
=================

The source code for all these Python examples can be found
under `MRPT/mrpt_examples_py <https://github.com/MRPT/mrpt/tree/develop/mrpt_examples_py>`_.
Run them with ``python3 <script>.py`` once the MRPT Python bindings are
installed (e.g. ``python3-mrpt`` packages, or ``source install/setup.bash``
after building from sources).

See also the :ref:`python_api`. C++ examples are `here <examples.html>`_.
A machine-readable index of all examples is available in
`examples.json <examples.json>`_.

"""

CONTENTS = """.. contents:: Categories
   :local:
   :depth: 1

"""


def existing_modules():
    return {
        d[len("mrpt_") :]
        for d in os.listdir(MODULES_DIR)
        if d.startswith("mrpt_") and os.path.isdir(os.path.join(MODULES_DIR, d))
    }


MODULES = existing_modules()


def unique(seq):
    return list(dict.fromkeys(seq))


def parse_metadata(text):
    """Parses the optional "mrpt-example: key=v1,v2; key2=v" line."""
    meta = {}
    m = re.search(r"mrpt-example:\s*(.*?)\s*(?:-->)?\s*$", text, re.MULTILINE)
    if not m:
        return meta
    for item in m.group(1).split(";"):
        if "=" not in item:
            continue
        key, value = item.split("=", 1)
        meta[key.strip()] = [v.strip() for v in value.split(",") if v.strip()]
    for r in meta.get("requires", []):
        if r not in REQUIRES_VALUES:
            raise ValueError("Unknown 'requires' value '{}' in: {}".format(r, m.group(0)))
    return meta


def strip_metadata(text):
    return re.sub(r"^.*mrpt-example:.*\n?", "", text, flags=re.MULTILINE).rstrip() + "\n"


def screenshot(name):
    files = sorted(glob.glob(os.path.join(IMAGES_DIR, name + "_screenshot.*")))
    return os.path.basename(files[0]) if files else None


def category_of(key):
    for cat in CATEGORIES:
        if key in cat[2]:
            return cat
    return OTHER_CATEGORY


def placeholder(cat):
    return "example_placeholder_{}.svg".format(cat[1])


def md_to_rst_inline(text):
    """Converts a one-paragraph markdown text into plain reStructuredText."""
    text = re.sub(r"!\[[^\]]*\]\([^)]*\)", "", text)  # images
    text = re.sub(r"\[([^\]]*)\]\([^)]*\)", r"\1", text)  # links
    text = re.sub(r"\\ref\s+\S+", "", text)
    text = re.sub(r"(?<!`)`([^`]+)`(?!`)", r"``\1``", text)
    return text.strip()


def first_paragraph(text):
    paragraph = []
    for line in text.strip().splitlines():
        if not line.strip():
            break
        paragraph.append(line.strip())
    return " ".join(paragraph)


# ---------------------------------------------------------------------------
# C++ examples
# ---------------------------------------------------------------------------
def cpp_source_file(example_dir):
    if os.path.isfile(os.path.join(example_dir, "main.cpp")):
        return "main.cpp"
    cpps = sorted(glob.glob(os.path.join(example_dir, "*.cpp")))
    return os.path.basename(cpps[0]) if cpps else None


def cpp_modules_and_classes(sources):
    modules = []
    classes = []
    for m in re.finditer(r"#\s*include\s*[<\"]mrpt/(\w+)/([\w/]+)\.h[>\"]", sources):
        module, header = m.group(1), m.group(2)
        if module not in MODULES:
            continue
        modules.append("mrpt_" + module)
        if "/" not in header and header[0].isupper():
            ns = "mrpt::" if module == "core" else "mrpt::{}::".format(module)
            classes.append(ns + header)
    return sorted(set(modules)), sorted(set(classes))


def collect_cpp():
    examples = []
    for example_dir in sorted(glob.glob(os.path.join(CPP_DIR, "*", ""))):
        example_dir = example_dir.rstrip(os.sep)
        name = os.path.basename(example_dir)
        src_file = cpp_source_file(example_dir)
        if not src_file:
            continue
        readme = os.path.join(example_dir, "README.md")
        readme_text = open(readme, encoding="utf-8").read() if os.path.isfile(readme) else ""
        sources = "".join(
            open(f, encoding="utf-8", errors="replace").read()
            for f in sorted(glob.glob(os.path.join(example_dir, "*.cpp")))
            + sorted(glob.glob(os.path.join(example_dir, "*.h")))
        )
        meta = parse_metadata(readme_text)
        modules, classes = cpp_modules_and_classes(sources)
        requires = meta.get("requires")
        if requires is None:
            requires = ["gui"] if {"mrpt_gui", "mrpt_imgui"} & set(modules) else []
        examples.append(
            {
                "name": name,
                "language": "cpp",
                "summary": md_to_rst_inline(first_paragraph(readme_text)),
                "readme": strip_metadata(readme_text) if readme_text else "",
                "category": category_of(name.split("_")[0]),
                "modules": modules,
                "classes": classes,
                "requires": requires,
                "tags": meta.get("tags", []),
                "video": (meta.get("video") or [None])[0],
                "screenshot": screenshot(name),
                "source": "mrpt_examples_cpp/{}/{}".format(name, src_file),
                "page": "page_{}".format(name),
            }
        )
    return examples


def write_cpp_page(ex):
    with open(os.path.join(OUT_MD_DIR, "example-" + ex["name"] + ".md"), "w", encoding="utf-8") as f:
        f.write("\\page {} Example: {}\n".format(ex["name"], ex["name"]))
        if ex["readme"]:
            f.write("\n" + ex["readme"] + "\n")
        if ex["video"]:
            f.write("\n\\htmlonly\n{}\n\\endhtmlonly\n".format(video_iframe(ex["video"])))
        if ex["screenshot"]:
            f.write("\n![{} screenshot]({})\n".format(ex["name"], ex["screenshot"]))
        f.write("\n" + info_line(ex, markdown=True) + "\n\n")
        f.write("C++ example source code:\n")
        f.write("\\include {}\n".format(ex["source"][len("mrpt_examples_cpp/") :]))


# ---------------------------------------------------------------------------
# Python examples
# ---------------------------------------------------------------------------
def py_modules_and_classes(code):
    modules = []
    classes = []
    aliases = {}  # local name -> MRPT module
    for node in ast.walk(ast.parse(code)):
        if isinstance(node, ast.Import):
            for a in node.names:
                parts = a.name.split(".")
                if parts[0] == "mrpt" and len(parts) > 1:
                    modules.append(parts[1])
                    if a.asname:
                        aliases[a.asname] = parts[1]
        elif isinstance(node, ast.ImportFrom) and node.module:
            parts = node.module.split(".")
            if parts[0] == "mrpt" and len(parts) > 1:
                modules.append(parts[1])
                classes += ["mrpt.{}.{}".format(parts[1], a.name) for a in node.names if a.name[0].isupper()]
        elif isinstance(node, ast.Attribute) and node.attr[0].isupper():
            # mrpt.X.Name or alias.Name
            base = ast.unparse(node.value).split(".")
            if base[0] == "mrpt" and len(base) == 2:
                classes.append("mrpt.{}.{}".format(base[1], node.attr))
            elif len(base) == 1 and base[0] in aliases:
                classes.append("mrpt.{}.{}".format(aliases[base[0]], node.attr))
    modules = [mod for mod in unique(modules) if mod in MODULES]
    return modules, sorted(set(classes))


def py_category(name, modules):
    m = re.match(r"mrpt_(\w+)_example$", name)
    if m:
        return category_of(m.group(1))
    for mod in PY_MODULE_PRIORITY:
        if mod in modules:
            return category_of(mod)
    return category_of(modules[0]) if modules else OTHER_CATEGORY


def collect_py():
    examples = []
    for script in sorted(glob.glob(os.path.join(PY_DIR, "*.py"))):
        name = os.path.splitext(os.path.basename(script))[0]
        code = open(script, encoding="utf-8").read()
        doc = ast.get_docstring(ast.parse(code)) or ""
        meta = parse_metadata(code)
        modules, classes = py_modules_and_classes(code)
        requires = meta.get("requires")
        if requires is None:
            requires = ["gui"] if "gui" in modules else []
        out_file = os.path.join(PY_DIR, name + ".out")
        doc_lines = doc.strip().splitlines()
        examples.append(
            {
                "name": name,
                "language": "python",
                "summary": md_to_rst_inline(doc_lines[0]) if doc_lines else "",
                "description": "\n".join(doc_lines[1:]).strip(),
                "category": py_category(name, modules),
                "modules": ["mrpt." + m for m in modules],
                "classes": classes,
                "requires": requires,
                "tags": meta.get("tags", []),
                "video": (meta.get("video") or [None])[0],
                "screenshot": screenshot(name),
                "output": os.path.basename(out_file) if os.path.isfile(out_file) else None,
                "source": "mrpt_examples_py/{}.py".format(name),
                "page": "pymrpt_example_{}".format(name),
            }
        )
    return examples


def write_py_page(ex):
    name = ex["name"]
    title = "Python example: {}.py".format(name)
    rst = ".. _pyexample_{}:\n\n{}\n{}\n{}\n\n".format(name, "=" * len(title), title, "=" * len(title))
    rst += ex["summary"] + "\n\n"
    if ex["video"]:
        rst += ".. raw:: html\n\n   " + video_iframe(ex["video"]) + "\n\n"
    if ex["screenshot"]:
        rst += ".. image:: images/{}\n   :width: 90%\n\n".format(ex["screenshot"])
    rst += info_line(ex, markdown=False) + "\n\n"
    rst += ".. literalinclude:: ../../{}\n   :language: python\n   :linenos:\n\n".format(ex["source"])
    if ex["output"]:
        rst += "Output:\n\n.. literalinclude:: ../../mrpt_examples_py/{}\n   :language: text\n\n".format(
            ex["output"]
        )
    with open(os.path.join(DOC_SRC, ex["page"] + ".rst"), "w", encoding="utf-8") as f:
        f.write(rst)


# ---------------------------------------------------------------------------
# Common
# ---------------------------------------------------------------------------
def video_iframe(video_id):
    return (
        '<iframe width="560" height="315" src="https://www.youtube.com/embed/{}" '
        'title="YouTube video player" frameborder="0" allowfullscreen></iframe>'
    ).format(video_id)


def info_line(ex, markdown):
    code = "`{}`" if markdown else "``{}``"
    parts = ["Modules: " + ", ".join(code.format(m) for m in ex["modules"])] if ex["modules"] else []
    if ex["requires"]:
        parts.append("Requires: " + ", ".join(ex["requires"]))
    return " | ".join(parts)


def card(ex):
    image = ex["screenshot"] or placeholder(ex["category"])
    badges = " ".join(":bdg-secondary:`{}`".format(r) for r in ex["requires"])
    lines = [
        "   .. grid-item-card:: {}".format(ex["name"]),
        "      :link: {}".format(ex["page"]),
        "      :link-type: doc",
        "      :shadow: md",
        "      :img-top: images/{}".format(image),
        "      :class-img-top: mrpt-example-thumb",
        "",
        "      " + ex["summary"],
    ]
    if badges:
        lines += ["", "      " + badges]
    return "\n".join(lines) + "\n\n"


def gallery_rst(header, examples):
    sections = {}
    for ex in examples:
        sections.setdefault(ex["category"][0], []).append(ex)

    rst = header + CSS + CONTENTS
    for cat in CATEGORIES + [OTHER_CATEGORY]:
        title = cat[0]
        if title not in sections:
            continue
        rst += "{}\n{}\n\n".format(title, "-" * len(title))
        rst += ".. grid:: 1 2 3 3\n   :gutter: 3\n   :class-container: mrpt-gallery\n\n"
        # Examples with a screenshot first:
        for ex in sorted(sections[title], key=lambda e: e["screenshot"] is None):
            rst += card(ex)
        rst += "\n"

    rst += ".. toctree::\n  :hidden:\n  :maxdepth: 1\n\n"
    rst += "".join("  {}.rst\n".format(ex["page"]) for ex in examples)
    return rst


def json_entry(ex):
    image = ex["screenshot"]
    return {
        "name": ex["name"],
        "language": ex["language"],
        "category": ex["category"][0],
        "summary": ex["summary"].replace("``", "`"),
        "modules": ex["modules"],
        "classes": ex["classes"],
        "requires": ex["requires"],
        "tags": ex["tags"],
        "source": GITHUB_BASE_URL + ex["source"],
        "doc": DOCS_BASE_URL + ex["page"] + ".html",
        "screenshot": DOCS_BASE_URL + image if image else None,
    }


def main():
    for f in glob.glob(os.path.join(OUT_MD_DIR, "example-*.md")):
        os.remove(f)
    for f in glob.glob(os.path.join(DOC_SRC, "pymrpt_example_*.rst")):
        os.remove(f)

    cpp = collect_cpp()
    py = collect_py()

    for ex in cpp:
        write_cpp_page(ex)
    for ex in py:
        write_py_page(ex)

    with open(os.path.join(DOC_SRC, "examples.rst"), "w", encoding="utf-8") as f:
        f.write(gallery_rst(CPP_HEADER, cpp))
    with open(os.path.join(DOC_SRC, "python_examples.rst"), "w", encoding="utf-8") as f:
        f.write(gallery_rst(PY_HEADER, py))

    os.makedirs(EXTRA_DIR, exist_ok=True)
    with open(os.path.join(EXTRA_DIR, "examples.json"), "w", encoding="utf-8") as f:
        json.dump(
            {
                "description": "Index of MRPT C++ and Python examples. "
                "requires: gui = opens a window, hardware = needs a sensor or device, "
                "dataset = needs a dataset not shipped with MRPT.",
                "examples": [json_entry(ex) for ex in cpp + py],
            },
            f,
            indent=1,
        )
        f.write("\n")


if __name__ == "__main__":
    main()
