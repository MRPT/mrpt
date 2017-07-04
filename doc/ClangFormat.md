# MRPT codebase to comply with ClangFormat

The following instructions offer a summary of the formatting/style changes
applied in <TODO Add date>,
as well as handy solutions for solving formatting problems and for making this
transition as smooth as possible for all MRPT developers.

## Line-wrapping

It has been decided that MRPT code is to be wrapped at **80** lines. This has
the following consequences:

- Long inline doxygen comments `//!< ...` are to be wrapped up and indented at
    the same line. This is not most likely not what the developer wants. To get
    around this replace them standard `/** ... */` comments or any other
    alternative outlined in the [doxygen
    docs](https://www.stack.nl/~dimitri/doxygen/manual/docblocks.html).
    For bulk reformatting of these comments one may use `sed` as in [this
    script](https://github.com/bergercookie/clang_git_format/blob/master/scripts/convert_inline_doxygen_comments.sh).

    **WARNING:** The `/**...*/` snippet should be placed on the **previous**
    line from the variable, directive that is for. This is **opposite** to the
    convention for the `//!<...` commenting which is placed to the right of the
    variable/directive.

- Long inline standard C comments `// ...` haven't been replaced prior to
    running clang-format in the codebase. If these exceeded the chars limit they
    were wrapped in the next line, which, again, isn't that the original
    developer would have wanted. However, they don't affect either the doxygen
    documentation (they aren't parsed at all) or the code compilation, so they
    are left (wrapped) for now. If something similar is encountered, please
    manually fix the comments (e.g., put them prior to the directive they are
    meant for) and make a PR.


## Updating branches prior to reformatting changes

In case you are working based on a commit prior to the [major reformatting
change](TODO add link to commit) you 'll have to comply with the formatting
changes prior to making your PR to the repository (otherwise the corresponding
travis test fails). You have two options on this:

- Use the reformat_branch functionality of clang_git_format script
    TODO - Add details
- Apply clang_format manually to the files that you have modified and make a
    merge to the [upstream repo](https://github.com/MRPT/mrpt).

    TODO - Add details

The **first option** is strongly suggested, but if that fails, resort to the
second.

## IDEs

TODO

## TODOs

- [ ] TODO - update the links
- [ ] Talk about the reformat_branch
- [ ] Solve all TODOs (how meta! :-) )
