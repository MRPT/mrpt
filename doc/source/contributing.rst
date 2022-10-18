.. _contributing:

==============
Contributing
==============

.. toctree::
  :maxdepth: 2

  MRPT_Coding_Style.rst
  mrpt_git_branches.rst
  ClangFormat.rst
  ClangFormat_internal.rst
  make_a_mrpt_release.rst


Did you find a bug in MRPT?
-----------------------------
* **Ensure it is not already** reported by searching:
  
  * The `issues list <https://github.com/MRPT/mrpt/issues>`_,
  * The `mailing list <https://groups.google.com/forum/#!forum/mrpt-users>`_ and
  * `Solved questions at stackoverflow.com <http://stackoverflow.com/search?q=mrpt>`_

Do you want to contribute new code to MRPT?
---------------------------------------------
* Please, **read carefully** the `C++ coding style for MRPT <MRPT_Coding_Style.html>`_.
* Make sure to be familiar with Git, branches, etc. A starting tutorial can be found `here <http://git-scm.com/docs/gittutorial>`_
* Fork in GitHub.
* Recommended: create a new branch with a descriptive name and work on it instead of the default `develop` branch.
* Send commits to your fork as needed:

  * Commit often.
  * Small commits with a proper description are preferred.
  
* **Verify that your code builds**. It will anyway be tested upon pull request by our CI.
* Most changes are relevant enough to be shown in the `doc/source/doxygen-docs/changelog.md <https://github.com/MRPT/mrpt/blob/master/doc/source/doxygen-docs/changelog.md>`_. Modify it as well to reflect what is new.
* Open a pull-request.
