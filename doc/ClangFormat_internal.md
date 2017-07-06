# ClangFormat - Developer Instructions

This file primarily outlies the procedure of formatting the entire MRPT codebase
with ClangFormat in case this is needed again, or in case we upgrade to a more
recent version of ClangFormat.

## Notes on formatting the codebase - clang-format 3.8

ClangFormat doesn't go well with `//` comments or `\code` doxygen blocks. If
there are cases where the linter returns with an error, correct the occurrences
manually. Usually a manual reflow of the comments is needed. For `\code` blocks
you can also keep them as is with `// clang-format [on|off]` directives.

- It is advised to set the `AlignTrailingComments` config var to false, as it
keeps reindenting comments between successive ClangFormat runs.


## Porting to a new ClangForamt version

Change occurrences of old version in `doc/ClangFormat.md`




