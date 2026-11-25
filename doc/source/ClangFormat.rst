==============================
Code formatting (clang-format)
==============================

All C++ sources in MRPT (``modules/``, ``apps/``, ``mrpt_examples_cpp/``, except
vendored code under ``3rdparty/`` directories) are formatted with
**clang-format 14**, using the `.clang-format <https://github.com/MRPT/mrpt/blob/develop/.clang-format>`_
file at the repository root (Google-based style, 100 columns).

A CI job checks every pull request, so format your changes before pushing:

.. code-block:: bash

   # Format the files you changed:
   clang-format-14 -i path/to/file.cpp path/to/file.h
   # Or check/format the whole codebase:
   scripts/clang_format_codebase.sh --check
   scripts/clang_format_codebase.sh

Use exactly version 14: other versions format some constructs differently. If
your distribution doesn't ship it, ``pipx install clang-format==14.0.6`` does
(this is what the CI job uses).

Editor integration (format on save) is available for most editors and IDEs, see
the `clang-format documentation <https://clang.llvm.org/docs/ClangFormat.html>`_.
Don't forget to also read the :doc:`MRPT_Coding_Style`.

Tips
----

- To keep a block untouched (e.g. a hand-aligned table), wrap it in
  ``// clang-format off`` and ``// clang-format on``.
- Prefer ``/** ... */`` Doxygen comments on the line before a declaration over
  long trailing ``//!<`` comments, which clang-format would wrap.
