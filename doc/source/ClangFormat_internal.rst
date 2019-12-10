=========================================
ClangFormat - Developer Instructions
=========================================

This file primarily outlines the procedure of formatting the entire MRPT
codebase with ClangFormat in case this is needed again, or in case we upgrade to
a more recent version of ClangFormat.

Notes on formatting the codebase 
--------------------------------------------------------

At present (Dec-2019) we use ``clang-format-8``.

ClangFormat doesn't go well with `//` comments or `\code` doxygen blocks. If
there are cases where the linter returns with an error, correct the occurrences
manually. Usually a manual reflow of the comments is needed. For `\code` blocks
you can also keep them as is with `// clang-format [on|off]` directives.

It is advised to set the `AlignTrailingComments` config var to false, as it
keeps reindenting comments between successive ClangFormat runs.


Porting to a new ClangFormat version
----------------------------------------

- Change the version specified in
    `clang_git_format/config.py <https://github.com/MRPT/mrpt/blob/master/scripts/clang_git_format/clang_git_format/config.py>`_.
- Rerun the `clang_format_codebase.sh script <https://github.com/MRPT/mrpt/blob/master/scripts/clang_format_codebase.sh>`_.
