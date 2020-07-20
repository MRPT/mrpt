.. _make_a_mrpt_release:

=============================
MRPT release check-list
=============================

These notes are mostly for myself (J.L. Blanco), but hopefully they'll be
useful to someone else maintaining MRPT in the future... ;-)


1) Create packages of sources
---------------------------------

- Go to MRPT dir.
- Edit ``doc/source/doxygen-docs/changelog.md`` and set the release date.
- ``bash scripts/build_docs.sh``
- Do the final ``git commit`` before the release.
- ``bash scripts/prepare_release.sh``
- ``bash scripts/prepare_debian.sh``

The packages are in ``$(HOME)/mrpt_release`` and ``$(HOME)/mrpt_debian``

Now for windows binary packages:
(see also the script: MRPT/scripts/automated_build_msvc_binary_package.bat)
- Extract mrpt-x.y.z.zip
- Run the script: automated_create_all_windows_MSVC_MinGW_build_dirs.bat

Note: Since 2020, Windows binary package generation is also automated
via AppVeyor CI.

2) DEBIAN PACKAGE
--------------------

- Go to mrpt_debian/debian
- Edit changelog
- Build package:

.. code-block: bash

   cd ~/mrpt_debian/
   gpg --armor --detach-sign  mrpt_*.tar.xz
   cd mrpt-*
   debuild -S -sa
   cd ..
   lintian *.changes

3) Test build in Debian Unstable
---------------------------------------

- ``sudo ARCH=amd64 DIST=sid pbuilder --update``
- ``sudo ARCH=amd64 DIST=sid pbuilder --build *.dsc``

Test all binary pkgs with: ``lintian *.deb``
