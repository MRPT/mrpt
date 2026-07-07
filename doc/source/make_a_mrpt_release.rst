.. _make_a_mrpt_release:

=============================
MRPT release check-list
=============================

These notes are mostly for myself (J.L. Blanco), but hopefully they'll be
useful to someone else maintaining MRPT in the future... ;-)

.. note::

   These instructions reflect the **MRPT 3.x** modular, catkin-style
   workflow, where each module under ``modules/*`` (and each app under
   ``apps/*``) carries its own ``package.xml`` (with its own ``<version>``)
   and its own ``CHANGELOG.rst``. All modules are released in lockstep with
   one shared version number. There is also a tool,
   ``packaging/release.py``, that automates most of the steps below,
   pausing for confirmation before any irreversible action (pushing,
   merging to ``master``, publishing the GitHub release). See
   `Automating the release`_ below.

   For the legacy MRPT 2.x process (single ``doc/source/doxygen-docs/changelog.md``,
   no per-module ``CHANGELOG.rst``/``package.xml`` versions), see the
   ``2.x`` git branch history of this same file.


1) Generate, review and commit the changelogs
------------------------------------------------

- Go to the MRPT git cloned repository.
- Make sure of being on branch ``develop`` and that it is up to date and clean.
- Make sure CI is green on ``develop``.
- Before touching any version number, generate a **"Forthcoming"** section
  in every module/app's ``CHANGELOG.rst``, summarizing commits since the
  last tag, using ``catkin_generate_changelog`` (also from the
  ``catkin_pkg`` package). Run it from the repository root:

.. code-block:: bash

   cd MRPT_ROOT
   git checkout develop && git pull
   catkin_generate_changelog
   # Answer "y" per-package (or pass -y for non-interactive) to (re)write
   # each modules/*/CHANGELOG.rst and apps/*/CHANGELOG.rst "Forthcoming"
   # section.

- **Review and hand-edit** the generated "Forthcoming" sections before
  committing: consolidate/merge duplicate or near-duplicate lines, drop
  noise (raw "Merge pull request ..." commit bodies, internal-only commits
  such as pure code-coverage/test additions), and fix wording so each
  module's changelog reads as a coherent summary for end users.
- Commit the reviewed changelogs **separately**, before any version bump:

.. code-block:: bash

   git add -u  # or list the changed CHANGELOG.rst files explicitly
   git commit -m "Update changelogs for upcoming release"

  This keeps changelog wording/review as its own reviewable commit,
  decoupled from the automatic version-bump commit created in the next
  step.


2) Bump module versions
------------------------------------------------

- Bump the ``<version>`` of **all** modules/apps and finalize the
  already-reviewed ``CHANGELOG.rst`` "Forthcoming" sections (renaming them
  to the new dated section, e.g. ``3.0.5 (2026-06-21)``) using
  ``catkin_prepare_release`` (provided by the ``catkin_pkg`` Python
  package: ``pip install catkin_pkg`` or ``apt install
  python3-catkin-pkg``). Run it from the repository root so it discovers
  every ``package.xml`` under ``modules/*`` and ``apps/*``:

.. code-block:: bash

   cd MRPT_ROOT
   git checkout develop && git pull
   catkin_prepare_release --bump patch --no-push
   # Use --bump minor / --bump major instead of patch as needed, or
   # --version X.Y.Z to set an explicit version.

  This command will:

  - Move each module's "Forthcoming" ``CHANGELOG.rst`` section into a new
    dated section (e.g. ``3.0.5 (2026-06-21)``), summarizing commits
    touching that module's directory since the last tag.
  - Bump ``<version>`` in every ``package.xml``.
  - Create **one commit** with the message being the new version (e.g.
    ``3.0.5``) on top of ``develop``.
  - Create an **annotated, unsigned** git tag with that same version name.

  Inspect the diff before continuing; hand-edit any ``CHANGELOG.rst``
  entries that need polishing (e.g. merging duplicate lines, fixing
  wording), then amend the commit if needed.

- If you need to re-sign the tag (recommended for releases), delete the
  unsigned one and recreate it signed:

.. code-block:: bash

   git tag -d X.Y.Z
   git tag --sign X.Y.Z


3) Merge into master and push
-------------------------------

.. code-block:: bash

   git checkout master && git pull
   git merge develop
   # **WARNING**: pushing makes the new commits and tag public:
   git push origin master
   git push origin X.Y.Z


4) Generate the source code packages
--------------------------------------

- Stay on ``master``, at the just-created tag.

.. code-block:: bash

   bash packaging/make_release.sh

- This produces, under ``$HOME/mrpt_release/``:

  - ``mrpt-X.Y.Z.tar.gz`` (Unix line endings) — the file used by Debian's
    ``uscan`` / ``gbp import-orig --uscan``, so its name and tag **must**
    match the ``debian/watch`` pattern in the Debian packaging repo.
  - ``mrpt-X.Y.Z.zip`` (DOS line endings, for Windows users).
  - ``mrpt-X.Y.Z.tar.gz.asc`` — detached GPG signature of the ``.tar.gz``.

- After generating the packages, clean the working tree:

.. code-block:: bash

   git clean -fd


5) Create the GitHub release and attach assets
-------------------------------------------------

- Aggregate the per-module ``CHANGELOG.rst`` entries for the new version
  into release notes, then create the release and upload the tarball, zip,
  and signature with the GitHub CLI (``gh``):

.. code-block:: bash

   gh release create X.Y.Z \
       --title "Release of vX.Y.Z" \
       --notes-file /path/to/aggregated_notes.md \
       "$HOME/mrpt_release/mrpt-X.Y.Z.tar.gz" \
       "$HOME/mrpt_release/mrpt-X.Y.Z.tar.gz.asc" \
       "$HOME/mrpt_release/mrpt-X.Y.Z.zip"

- Windows binaries are produced automatically by CI (AppVeyor/GitHub
  Actions); attach them to the same release once they are ready, if not
  already wired to upload directly.
- Release done! :-)


Automating the release
-------------------------

Steps 2-5 above are scripted end-to-end by ``packaging/release.py``. It
runs the same commands documented here, in the same order, and stops to
ask for an explicit "yes" before each action that is hard to reverse or
publicly visible (pushing ``develop``'s tag/commit, merging and pushing
``master``, and creating/publishing the GitHub release). Run it from the
repository root:

.. note::

   Step 1 (generate, review and commit the changelogs) is **not**
   automated: it calls ``catkin_prepare_release`` non-interactively
   (``-y``), which would otherwise generate "Forthcoming" changelog
   entries straight from raw commit messages with no chance to review or
   consolidate them. Always do step 1 by hand first, commit it, and only
   then run ``packaging/release.py``.

.. code-block:: bash

   python3 packaging/release.py --bump patch

Pass ``--dry-run`` to print every command without executing anything, or
``--version X.Y.Z`` to pick an explicit version instead of bumping. See
``python3 packaging/release.py --help`` and the script's own docstring for
details and the full list of preconditions it checks (clean tree, correct
branch, CI status is **not** checked automatically — verify it yourself
first).

A fully unattended (`-y`/non-interactive) release is intentionally not
offered: merging to ``master`` and publishing a GitHub release are public,
hard-to-reverse actions, so a human confirmation gate is kept at those two
points even when every other step is automated.


6) Create a new Debian package
--------------------------------

As of MRPT 2.4.0 (Oct/2021), we switched to gbp with:

- Upstream repository (source code): https://github.com/mrpt/mrpt
- gbp repository: https://salsa.debian.org/robotics-team/mrpt

Instructions:

1) Make sure of having generated and uploaded to the GitHub release the
   `xxx.tar.gz` and its PGP signature (part "4" and "5" above).

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
   # (Create cowbuilder image upon first usage first, then run the command below
   #  inside of:
   #   DIST=sid BINDMOUNTS=/home/xxx/xxx/ sudo pbuilder login --save-after-login
   # )
   gbp buildpackage
   # or from the host:
   # gbp buildpackage --git-pbuilder --git-postbuild='lintian $GBP_CHANGES_FILE'

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
