Utility scripts to ease packaging for Debian, Ubuntu, Windows binaries, etc.
------------------------------------------------------------------------------

The main entry points in this directory are:

- `packaging/prepare_debian.sh`: Generates the `*.orig.tar.gz` tarballs and
   sign them.
- `packaging/prepare_release.sh`: Generates the `*.tar.gz` and `*.zip` source
   code packages.

Both above include the git submodules that should go into packages, and removes
those that are not intended to be shipped within Debian packages but which we
keep into the git repo for the convenience of (mainly) Windows users.

Read more here: [MRPT release check-list](https://docs.mrpt.org/reference/latest/make_a_mrpt_release.html) ([page source code](../doc/source/make_a_mrpt_release.rst)).
