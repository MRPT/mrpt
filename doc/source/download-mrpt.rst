.. _downloadmrpt:

##############
Download MRPT
##############

.. contents:: :local:


Source code
-------------

Get the latest development version from the official
`MRPT GitHub repository <https://github.com/MRPT/mrpt/>`_ with:

.. code-block:: bash

   git clone https://github.com/MRPT/mrpt.git --recursive

Next step: :ref:`compiling`

Debian/Ubuntu official repositories
---------------------------------------

Install the `official version <https://packages.ubuntu.com/source/jammy/mrpt>`_ for your distribution with:

.. code-block:: bash

   sudo apt install libmrpt-dev mrpt-apps


.. note::
   Versions in `official repositories <https://packages.ubuntu.com/source/jammy/mrpt>`_
   may be quite outdated. It is strongly
   recommended to use the PPAs (read below) or build from sources instead.


Debian/Ubuntu PPA
----------------------

Last **stable release** (`mrpt-stable PPA status <https://launchpad.net/~joseluisblancoc/+archive/ubuntu/mrpt-stable>`_) (from the ``master`` branch), for Ubuntu >=18.04:

.. code-block:: bash

   sudo add-apt-repository ppa:joseluisblancoc/mrpt-stable   # master (stable releases) branch
   sudo apt install libmrpt-dev mrpt-apps

**Nightly builds** (`mrpt-nightly PPA status <https://launchpad.net/~joseluisblancoc/+archive/ubuntu/mrpt>`_) (from the ``develop`` branch) for Ubuntu >=18.04:

.. code-block:: bash

   sudo add-apt-repository ppa:joseluisblancoc/mrpt   # develop branch
   sudo apt install libmrpt-dev mrpt-apps


Windows installers
--------------------

Executables (.exes and .dlls) and development libraries (.hs and .libs) included:

   - `Nightly builds <https://github.com/MRPT/mrpt/releases/tag/Windows-nightly-builds>`_
