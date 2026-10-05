"""Renders as plain text the doxyrest links to C++ members with no page of
their own: doxyrest is configured to document public members only
(PROTECTION_FILTER in doxyrest-config.lua) and to skip empty macros, but
public declarations and comments may still refer to those. The hidden
members are read from the Doxygen XML output; any other unresolved reference
keeps producing a warning.
"""

import glob
import os
import xml.etree.ElementTree as ET

XML_DIR = "xml-dir"


def hidden_member_ids(xml_dir):
    """Ids of the Doxygen items that doxyrest does not give a page to."""
    hidden = set()
    for path in glob.glob(os.path.join(xml_dir, "*.xml")):
        try:
            root = ET.parse(path).getroot()
        except ET.ParseError:
            continue
        for compound in root.iter("compounddef"):
            compound_hidden = compound.get("prot", "public") != "public"
            if compound_hidden:
                hidden.add(compound.get("id"))
            for member in compound.iter("memberdef"):
                if compound_hidden or member.get("prot", "public") != "public" or member.get("kind") == "define":
                    hidden.add(member.get("id"))
                    hidden.update(v.get("id") for v in member.iter("enumvalue"))
    # Sphinx labels are lowercase:
    return {i.lower() for i in hidden if i}


def load_hidden_members(app):
    app.env.mrpt_hidden_doxids = hidden_member_ids(os.path.join(app.srcdir, XML_DIR))


def plain_text_for_hidden_members(app, env, node, contnode):
    target = node.get("reftarget", "")
    if node.get("refdomain") != "std" or not target.startswith("doxid-"):
        return None
    if target[len("doxid-"):].lower() in getattr(env, "mrpt_hidden_doxids", ()):
        return contnode
    return None


def setup(app):
    app.connect("builder-inited", load_hidden_members)
    app.connect("missing-reference", plain_text_for_hidden_members)
    return {"parallel_read_safe": True, "parallel_write_safe": True}
