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
   bash packaging/prepare_release.sh
   # If everything seems OK, push it:
   # **WARNING**: This command makes the release public!
   git push master --tags

- The source tarballs are now in ``$(HOME)/mrpt_release``.
- Windows binaries will be automatically generated from AppVeyor CI, wait for them.
- Create a new GitHub release from the tag.
- Attach the tarball, pgp signature, and Windows installer.
- Release done! :-)


2) Create a new Debian package
--------------------------------

As of MRPT 2.4.0 (Oct/2021), we switched to gbp with:

- Upstream repository (source code): https://github.com/mrpt/mrpt
- gbp repository: https://salsa.debian.org/robotics-team/mrpt

Instructions:

1) Make sure of having generated and uploaded to the GitHub release the
   `xxx.tar.gz` and its PGP signature (part "1" above).

2) Go to the directory where the mrpt gbp repo is cloned and integrate the
   new release with:

.. code-block:: bash

   cd /PATH/TO/MRPT_GBP_REPO
   git checkout master

   # Download new release from GitHub releases:
   gbp import-orig --uscan --pristine-tar
   # **Or** from a local copy (for dry-runs before actual releases):
   #gbp import-orig --pristine-tar /PATH/TO/MRPT.X.Y.Z.tar.gz

   # update the files in debian/ (copyright, *.install) as needed, commit them and run:
   gbp dch -R -c --git-author --distribution=unstable

   # Test build (if changes are significant):
   # (Create cowbuilder image upon first usage first)
   gbp buildpackage --git-pbuilder --git-dist=unstable --git-postbuild='lintian $GBP_CHANGES_FILE'

   # Push:
   gbp push


If at some point a new PGP signature is used to sign upstream (MRPT GitHub repo,
not Debian) source packages, remember updating the PGP public key within the
gbp repository with:

.. code-block:: bash

   # Export signing pub key:
   git checkout master
   mkdir debian/upstream/ || true
   gpg --export --export-options export-minimal --armor > debian/upstream/signing-key.asc
