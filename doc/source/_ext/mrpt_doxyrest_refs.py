"""Renders as plain text the doxyrest links to C++ members with no page of
their own: doxyrest is configured to document public members only
(PROTECTION_FILTER in doxyrest-config.lua), but public declarations and
comments may still refer to protected or private ones. Other unresolved
references keep producing warnings.
"""


def plain_text_for_hidden_members(app, env, node, contnode):
    if node.get("refdomain") == "std" and node.get("reftarget", "").startswith("doxid-"):
        return contnode
    return None


def setup(app):
    app.connect("missing-reference", plain_text_for_hidden_members)
    return {"parallel_read_safe": True, "parallel_write_safe": True}
