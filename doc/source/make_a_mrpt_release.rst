.. _make_a_mrpt_release:

=============================
MRPT release check-list
=============================

These notes are mostly for myself (J.L. Blanco), but hopefully they'll be
useful to someone else maintaining MRPT in the future... ;-)


1) Generate source code packages
-----------------------------------

- Go to MRPT git cloned repository.
- Make sure of being in branch `develop`.
- Edit ``doc/source/doxygen-docs/changelog.md`` and set the release date.
- Do the final ``git commit`` before the release.
- When all CI tests pass:

.. code-block:: bash

   git checkout master
   git merge develop
   git tag --sign X.Y.Z # release new version
   bash packaging/prepare_release.sh  # TODO: Merge prepare_debian.sh here!!

- The source tarballs are now in ``$(HOME)/mrpt_release``.
- Windows binaries will be automatically generated from AppVeyor CI, wait for them. 
- Create a new GitHub release from the tag.
- Attach the tarball, pgp signature, and Windows installer.
- Release done! :-) 


(JL: Recover this old code into the new prepare_release.sh ?)

.. code-block:: bash

   # Export signing pub key:
   mkdir debian/upstream/ || true
   gpg --export --export-options export-minimal --armor > debian/upstream/signing-key.asc


2) Create a new Debian package
--------------------------------

As of Oct/2021, we switched to gbp with:

- Upstream repository (source code): https://github.com/mrpt/mrpt
- gbp repository: https://salsa.debian.org/robotics-team/mrpt

Instructions:

1) Make sure of having generated and uploaded to the GitHub release the `xxx.tar.gz` and its PGP signature.

2) Go to the directory where the mrpt gbp repo is cloned.

3) Integrate the new release with:

.. code-block:: bash

   cd MRPT_GBP_REPO
   git checkout master
   gbp import-orig --uscan --pristine-tar

       gbp:info: Importing '/home/jlblanco/mrpt_debian/mrpt_2.4.0.orig.tar.gz' to branch 'upstream'...
       gbp:info: Source package is mrpt
       gbp:info: Upstream version is 2.4.0
       gbp:info: Replacing upstream source on 'master'
       gbp:info: Successfully imported version 2.4.0 of /home/jlblanco/mrpt_debian/mrpt_2.4.0.orig.tar.gz

   # update the files in debian/ (copyright, *.install) as needed, commit them and run:
   gbp dch -R -c
   gbp push


(JL: And this?)

.. code-block:: bash

   # Add AUTHORS file, referenced in d/copyright
   cp $MRPTSRC/AUTHORS debian/


3) Test build in Debian Unstable
---------------------------------------

.. code-block:: bash

   gbp buildpackage --git-pbuilder --git-postbuild='lintian $GBP_CHANGES_FILE'
