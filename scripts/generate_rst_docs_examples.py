#!/usr/bin/env python3
"""Generates the doxygen page of each C++ example (from its README.md,
screenshot and source file), and the examples.rst gallery listing them all.
Invoked automatically by doc/Makefile.

The first paragraph of each example README.md is used as its description in
the gallery, and doc/source/images/<example>_screenshot.* as its thumbnail.
"""

import glob
import os
import re

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
EXAMPLES_DIR = os.path.join(ROOT, "mrpt_examples_cpp")
OUT_MD_DIR = os.path.join(ROOT, "doc", "source", "doxygen-docs")
OUT_RST = os.path.join(ROOT, "doc", "source", "examples.rst")
IMAGES_DIR = os.path.join(ROOT, "doc", "source", "images")

# Gallery sections, by example name prefix:
CATEGORIES = [
    ("3D graphics and GUIs", ["opengl", "gui", "imgui"]),
    ("Maps", ["maps"]),
    ("SLAM, Bayesian filtering and graph-SLAM", ["slam", "bayes", "graphslam"]),
    ("Navigation and graph search", ["nav", "graphs"]),
    ("Math, poses and geometry", ["math", "poses", "topography", "random"]),
    ("Images", ["img"]),
    ("Sensors, observations and datasets", ["obs", "hwdrivers", "kitti", "rgbd"]),
    (
        "Core, containers, I/O and system",
        ["core", "containers", "io", "serialization", "rtti", "typemeta", "system", "comms"],
    ),
]
OTHER_CATEGORY = "Other"

HEADER = """.. _examples:

===============
C++ examples
===============

The source code for all these C++ examples can be found
under `MRPT/mrpt_examples_cpp <https://github.com/MRPT/mrpt/tree/develop/mrpt_examples_cpp>`_.
They are built along with MRPT when ``MRPT_BUILD_EXAMPLES`` is enabled in CMake.

Python examples are `here <python_examples.html>`_.

.. raw:: html

   <style>
   .mrpt-example-thumb { height: 150px !important; object-fit: cover; }
   .mrpt-gallery .sd-card-title { font-size: 0.9em; overflow-wrap: anywhere; }
   </style>

.. contents:: Categories
   :local:
   :depth: 1

"""


def source_file(example_dir):
    if os.path.isfile(os.path.join(example_dir, "main.cpp")):
        return "main.cpp"
    cpps = sorted(glob.glob(os.path.join(example_dir, "*.cpp")))
    return os.path.basename(cpps[0]) if cpps else None


def screenshot(name):
    files = sorted(glob.glob(os.path.join(IMAGES_DIR, name + "_screenshot.*")))
    return os.path.basename(files[0]) if files else None


def summary(readme_text):
    """First paragraph of a README, as plain reStructuredText."""
    paragraph = []
    for line in readme_text.strip().splitlines():
        if not line.strip():
            break
        paragraph.append(line.strip())
    text = " ".join(paragraph)
    text = re.sub(r"!\[[^\]]*\]\([^)]*\)", "", text)  # images
    text = re.sub(r"\[([^\]]*)\]\([^)]*\)", r"\1", text)  # links
    text = re.sub(r"\\ref\s+\S+", "", text)
    text = re.sub(r"(?<!`)`([^`]+)`(?!`)", r"``\1``", text)
    return text.strip()


def category(name):
    prefix = name.split("_")[0]
    for title, prefixes in CATEGORIES:
        if prefix in prefixes:
            return title
    return OTHER_CATEGORY


def write_example_page(name, src_file, readme_text, image):
    with open(os.path.join(OUT_MD_DIR, "example-" + name + ".md"), "w", encoding="utf-8") as f:
        f.write("\\page {} Example: {}\n".format(name, name))
        if readme_text:
            f.write("\n" + readme_text + "\n")
        if image:
            f.write("\n![{} screenshot]({})\n".format(name, image))
        f.write("C++ example source code:\n")
        f.write("\\include {}/{}\n".format(name, src_file))


def card(name, desc, image):
    lines = [
        "   .. grid-item-card:: {}".format(name),
        "      :link: page_{}".format(name),
        "      :link-type: doc",
        "      :shadow: md",
        "      :img-top: images/{}".format(image),
        "      :class-img-top: mrpt-example-thumb",
        "",
        "      " + desc,
        "",
    ]
    return "\n".join(lines) + "\n"


def main():
    for f in glob.glob(os.path.join(OUT_MD_DIR, "example-*.md")):
        os.remove(f)

    sections = {}
    names = []
    for example_dir in sorted(glob.glob(os.path.join(EXAMPLES_DIR, "*", ""))):
        example_dir = example_dir.rstrip(os.sep)
        name = os.path.basename(example_dir)
        src_file = source_file(example_dir)
        if not src_file:
            continue
        readme = os.path.join(example_dir, "README.md")
        readme_text = open(readme, encoding="utf-8").read() if os.path.isfile(readme) else ""
        image = screenshot(name)
        write_example_page(name, src_file, readme_text, image)
        names.append(name)
        sections.setdefault(category(name), []).append((name, summary(readme_text), image))

    rst = HEADER
    for title in [c[0] for c in CATEGORIES] + [OTHER_CATEGORY]:
        if title not in sections:
            continue
        rst += "{}\n{}\n\n".format(title, "-" * len(title))
        # Examples with a screenshot, as cards:
        with_image = [e for e in sections[title] if e[2]]
        if with_image:
            rst += ".. grid:: 1 2 3 3\n   :gutter: 3\n   :class-container: mrpt-gallery\n\n"
            for name, desc, image in with_image:
                rst += card(name, desc, image)
            rst += "\n"
        # The rest, as a list:
        for name, desc, image in sections[title]:
            if not image:
                rst += "- :doc:`{} <page_{}>`: {}\n".format(name, name, desc)
        rst += "\n"

    rst += ".. toctree::\n  :hidden:\n  :maxdepth: 1\n\n"
    rst += "".join("  page_{}.rst\n".format(n) for n in names)

    with open(OUT_RST, "w", encoding="utf-8") as f:
        f.write(rst)


if __name__ == "__main__":
    main()
