=========================================
MRPT codebase to comply with ClangFormat
=========================================

The following instructions offer a summary of the formatting/style changes
applied on July 7th 2017, as well as handy solutions for solving formatting
problems and for making this transition as smooth as possible.

As of Dec 2019, we use clang-format-8.

Line-wrapping
--------------

MRPT code is to be wrapped at **80** lines. This has the following consequences:

- Long(er) inline doxygen comments `//!< ...` are to be automatically wrapped
    up and indented at the same line. This is most likely not what the developer
    wants. To get around this replace them with the standard `/** ... */`
    comments or any other alternative outlined in the [doxygen
    docs](https://www.stack.nl/~dimitri/doxygen/manual/docblocks.html).

    For bulk reformatting of these comments you may use `sed` as in [this
    script](https://github.com/bergercookie/clang_git_format/blob/master/scripts/convert_inline_doxygen_comments.sh).

    **WARNING:** The `/**...*/` snippet should be placed on the **previous**
    line from the variable, directive that is for. This is **opposite** to the
    convention for the `//!<...` commenting which is placed to the right of the
    variable/directive.

- Long inline standard C comments `// ...` weren't  been replaced prior to
    running clang-format in the codebase. If these exceeded the chars limit,
    they were wrapped *in the next line*, which, again, isn't that the original
    developer would have wanted. However, they don't affect either the doxygen
    documentation (they aren't parsed at all) or the code compilation, so they
    are left as they are (wrapped) for now. If you encounter such a case, please
    manually fix the comments (e.g., put them prior to the directive they are
    meant for) and make a PR.


Comply new code with ClangFormat
------------------------------------

The ClangFormat configuration that MRPT uses can be found
`here <https://github.com/MRPT/mrpt/blob/master/.clang-format>`_. To check
whether your existing code complies with this, you can run the check_style.sh
script from the MRPT root.

```sh
TASK=lint_all travis/check_style.sh
```

The same script is also used to validate that your changes comply, when you make
a Pull-Request to the MRPT upstream repo.

To format your changes according to the `.clang-format` configuration, either:

- Run `clang-format` directly specifying the location of the modified files
    (beware to use the ClangFormat version specified in
    `clang_git_format/config.py <https://github.com/MRPT/mrpt/blob/master/scripts/clang_git_format/clang_git_format/config.py>`_:

- Run ClangFormat on the whole codebase. To do that use the
    [clang_format_codebase.sh
    script](https://github.com/MRPT/mrpt/blob/master/scripts/clang_format_codebase.sh)
    from the MRPT root directory. Since the MRPT codebase is already formatted,
    (assuming that your work off a commit after the codebase reformat) only your
    changes are going to be modified. Do check the results of ClangFormat before
    applying for a patch.

IDEs
------------------------------------

A very important step in the "post-reformat era" is to stick around and
integrate ClangFormat into the daily routine for MRPT development. To that end
there exist plugins, for your editor of choice, that facilitate applying the
ClangFormat configuration while concurrently modifying/adding code.

- `Vim <https://github.com/rhysd/vim-clang-format>`_

    A sample Vim configuration is given in `this snippet <https://gist.github.com/bergercookie/9a2e96e19733b32ca55b8e2940eaba2c>`_.

- `Visual Studio 2017 <https://marketplace.visualstudio.com/items?itemName=HansWennborg.ClangFormat>`_
- `Visual Studio Code <https://marketplace.visualstudio.com/items?itemName=xaver.clang-format>`_
- `Emacs <https://llvm.org/svn/llvm-project/cfe/trunk/tools/clang-format/clang-format.el>`_
- `Sublime <https://github.com/rosshemsley/SublimeClangFormat>`_
- `Atom <https://atom.io/packages/formatter-clangformat>`_
- `XCode <https://github.com/mapbox/XcodeClangFormat>`_

The plugin of your choice should:

- Format code based on the `.clang-format` file placed in the MRPT root
    directory.
- Use the ClangFormat version especified at the top of this page. This is 
because part of the configuration directives are not compatible with 
previous ClangFormat versions.

Refer to the docs of the corresponding plugin for tweaking its behavior.
Also, do check the `MRPT Coding style <https://github.com/MRPT/mrpt/blob/master/doc/MRPT_Coding_Style.md>`_
to minimize the content that ClangFormat modifies

Past discussion links
------------------------------------

- `Discussion issue <https://github.com/MRPT/mrpt/issues/520>`_
- `Pull-Request <https://github.com/MRPT/mrpt/pull/556>`_
- `Pull-Request #2 <https://github.com/MRPT/mrpt/pull/559>`_

