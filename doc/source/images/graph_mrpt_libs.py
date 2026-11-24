#!/usr/bin/env python3
"""Writes graph_mrpt_libs.dot: the dependencies between MRPT modules, from
their package.xml files (run graph_mrpt_libs_build.sh to render it)."""

import glob
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
MODULES = os.path.join(HERE, "..", "..", "..", "modules")
SKIP = {"mrpt_common", "mrpt_data", "mrpt_imgui_vendor"}
GROUP = {"mrpt_libapps_cli": "mrpt_apps", "mrpt_libapps_gui": "mrpt_apps"}


def deps_of(pkg_xml):
    txt = open(pkg_xml, encoding="utf-8").read()
    deps = re.findall(r"<(?:build_)?depend>(mrpt_[a-z_]+)</", txt)
    return sorted(set(deps) - SKIP)


def main():
    graph = {}
    for pkg_xml in glob.glob(os.path.join(MODULES, "mrpt_*", "package.xml")):
        name = os.path.basename(os.path.dirname(pkg_xml))
        if name not in SKIP:
            graph[name] = deps_of(pkg_xml)
    lines = ["digraph MRPT_LIBS {", '  node [shape=box, style=filled, fillcolor="#D0D0D0"];']
    for name in sorted(graph):
        label = name + (" (.h)" if name == "mrpt_typemeta" else "")
        url = "group_{}_grp.html".format(GROUP.get(name, name))
        lines.append('  {} [label="{}", URL="{}"];'.format(name, label, url))
        for d in graph[name]:
            if d in graph and d != name:
                lines.append("  {} -> {};".format(name, d))
    lines.append("}")
    with open(os.path.join(HERE, "graph_mrpt_libs.dot"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
