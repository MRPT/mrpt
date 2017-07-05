# MRPT codebase to comply with ClangFormat

The following instructions offer a summary of the formatting/style changes
applied on <TODO Add date>,
as well as handy solutions for solving formatting problems and for making this
transition as smooth as possible.

## Line-wrapping

MRPT code is to be wrapped at **80** lines. This has the following consequences:

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

- Long inline standard C comments `// ...` weren't  been replaced prior to
    running clang-format in the codebase. If these exceeded the chars limit,
    they were wrapped in the next line, which, again, isn't that the original
    developer would have wanted. However, they don't affect either the doxygen
    documentation (they aren't parsed at all) or the code compilation, so they
    are left (wrapped) for now. If you encounter such a case, please
    manually fix the comments (e.g., put them prior to the directive they are
    meant for) and make a PR.


## Updating branches prior to reformatting changes

In case you are working based on a commit prior to the [major reformatting
change](TODO add link to commit) you 'll have to comply with the formatting
changes prior to making your PR to the repository (otherwise the corresponding
travis test fails). You have two options on this:

- Use the reformat_branch functionality of clang_git_format script. We assume
    that the, already reformatted branch you are making the PR to is called
    `mrpt-2.0-devel`. We also assume that you are working on a separate branch
    named `feature`. If that's not the case:
    1. Make a `feature` branch at your latest commit and check that out:

      ```sh
      git checkout -b feature
      ```
    2. Pull the changes from the `upstream/mrpt-2.0-devel` branch:

    ```sh
    git fetch upstream mrpt-2.0-devel
    # WARNING: This overwrites whatever separate work you have in mrpt-2.0-devel
    # branch. You should have your work in a separate *feature* branch.
    git checkout mrpt-2.0-devel
    git reset --hard upstream/mrpt-2.0-devel
    ```

    The procedure is expained in detail in [this
    article](https://engineering.mongodb.com/post/succeeding-with-clangformat-part-3-persisting-the-change)
    (watch the arrow direction. Also `clang_format.py reformat-branch T R`
    should instead be `clang_format.py reformat-branch P R`).

    Checkout your `feature` branch again and run clang_format.py script as
    follows:

    ```sh
    git checkout feature # Make sure you are in feature branch
    ./scripts/clang_git_format/format_code.py  -g . \
        --reformat_branch <TODO COMMIT_ID_MRPT_REFORMAT> mrpt-2.0-devel mrpt-2.0-devel"
    ```

    If everything went according to plan, a new branch named
    `feature-reformatted` should be rebased on top of mrpt-2.0-devel branch. Now
    all that's left is to make a pull-request from the rebased branch.

    TODO - Revisit these instructions

- Apply clang_format manually to the files that you have modified and make a
    merge to the [upstream repo](https://github.com/MRPT/mrpt).

    TODO - Add details


## Comply new code with ClangFormat

The clang-format configuration that MRPT uses can be found [here](TODO - Add
link to .clang-format at MRPT root). To check whether your existing code
complies with this, you can run the check_style.sh script from the MRPT root.

To format your changes according to the `.clang-format` configuration, either:

- Run `clang-format` directly specifying the location of the modified files
    (beware to use the ClangFormat version specified in
    [clang_git_format/config.py](TODO - Add link to config file):

- Run `clang_git_format --lint-files` on the whole codebase. Since the MRPT
    codebase is already formatted, (assuming that your work off a commit after
    the codebase reformat) only your changes are going to be modified.  use the
    [clang_format_codebase.sh script](TODO - Add link) from the MRPT root
    directory.

    ```sh
    TASK=LINT_ALL travis/check_style.sh
    ```

To make sure that your changes comply with ClangFormat, run the
[check_style.sh script](TODO - Add link) from the MRPT root directory. The
`check_style.sh` script is also used to validate that your changes comply, when
you make a Pull-Request to the MRPT upstream repo.


## IDEs

TODO

## TODOs

- [X] Add travis test file
- [X] Enable to lint one's work offlien - prior to PR/commit
- [ ] TODO - update the links
- [ ] Talk about the reformat_branch
- [ ] Solve all TODOs (how meta! :-) )
