.. _make_a_mrpt_release:

=============================
MRPT release check-list
=============================

These notes are mostly for myself (J.L. Blanco), but hopefully they'll be
useful to someone else maintaining MRPT in the future... ;-)


1) Generate source code packages
-----------------------------------

- Go to MRPT dir.
- Edit ``doc/source/doxygen-docs/changelog.md`` and set the release date.
- Do the final ``git commit`` before the release.
- ``bash packaging/prepare_release.sh``
- ``bash packaging/prepare_debian.sh``

The packages are in ``$(HOME)/mrpt_release`` and ``$(HOME)/mrpt_debian``

Now for windows binary packages:
(see also the script: MRPT/scripts/automated_build_msvc_binary_package.bat)
- Extract mrpt-x.y.z.zip
- Run the script: automated_create_all_windows_MSVC_MinGW_build_dirs.bat

Note: Since 2020, Windows binary package generation is also automated
via AppVeyor CI.

2) Create a new Debian package
--------------------------------

As of Oct/2021, we switched to gbp with:

- Upstream repository (source code): https://github.com/mrpt/mrpt
- gbp repository: https://salsa.debian.org/robotics-team/mrpt

Instructions:

1) Make sure of generating the `xxx.orig.tar.gz` file first with:

.. code-block:: bash

   cd MRPT_SOURCE_ROOT
   packaging/prepare_debian.sh

This should have generated these files:

- `xx`: orig tarball.
- `xx`: signature.

Now, we have to integrate it into the gbp repo.

2) Go to the directory where the mrpt gbp repo is cloned.

3) Integrate the new release with:

.. code-block:: bash

   cd MRPT_GBP_REPO
   gbp import-orig ~/mrpt_debian/mrpt_*.orig.tar.gz
   # JL: What else???


(JL: What about this old code, now removed from prepare_debian.sh ?)

.. code-block:: bash

   # Export signing pub key:
   mkdir debian/upstream/ || true
   gpg --export --export-options export-minimal --armor > debian/upstream/signing-key.asc


(JL: And this?)

.. code-block:: bash

   # Add AUTHORS file, referenced in d/copyright
   cp $MRPTSRC/AUTHORS debian/


Was:

.. code-block:: bash

   cd ~/mrpt_debian/mrpt-*
   debuild -S -sa
   cd ..
   lintian *.changes




3) Test build in Debian Unstable
---------------------------------------

.. code-block:: bash

   sudo ARCH=amd64 DIST=sid pbuilder --update
   sudo ARCH=amd64 DIST=sid pbuilder --build *.dsc
   cd /var/cache/pbuilder/sid-amd64/result/
   lintian -I *.deb
