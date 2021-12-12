Utility scripts to ease packaging for Debian, Ubuntu, Windows binaries, etc.
------------------------------------------------------------------------------

The main entry point in this directory is:

- `packaging/make_release.sh`: Generates the `*.tar.gz` and `*.zip` source
   code packages. Refer to comments in
   [packaging/make_release.sh](packaging/make_release.sh) for the full list of
   tasks done.

The script includes the git submodules that should go into packages, and removes
those that are not intended to be shipped within Debian packages but which we
keep into the git repo for the convenience of (mainly) Windows users.

Read more here: [MRPT release check-list](https://docs.mrpt.org/reference/latest/make_a_mrpt_release.html) ([page source code](../doc/source/make_a_mrpt_release.rst)).

The code actually generating nightly (and stable) PPA releases lives [here](https://github.com/MRPT/mrpt-ubuntu-ppa-packages) (browse the different branches).
