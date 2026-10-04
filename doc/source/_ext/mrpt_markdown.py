"""Tweaks for the Markdown version of the docs (sphinx-markdown-builder):
code blocks are written as plain text, since doxyrest fills the C++
declarations with highlighting and cross-reference nodes that the Markdown
translator would otherwise drop or turn into link soup.
"""

from docutils import nodes
from sphinx_markdown_builder.translator import MarkdownTranslator


class MrptMarkdownTranslator(MarkdownTranslator):
    def visit_literal_block(self, node):
        language = node.get("language", "")
        if not language and "highlight" in node["classes"]:
            language = "cpp"  # doxyrest ref-code-block
        text = node.astext().rstrip("\n")
        if text:
            self.add("```{}\n{}\n```".format(language, text), prefix_eol=2, suffix_eol=2)
        raise nodes.SkipNode

    def visit_target(self, node):
        # No "<a id=...>" anchors: noise for readers of the Markdown pages.
        pass


def setup(app):
    app.set_translator("markdown", MrptMarkdownTranslator)
    return {"parallel_read_safe": True, "parallel_write_safe": True}
