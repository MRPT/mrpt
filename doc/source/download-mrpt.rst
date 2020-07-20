.. _downloadmrpt:

##############
Download MRPT
##############

.. contents:: :local:


Source code
-------------

Get the latest development version with:

.. code-block:: bash

   git clone https://github.com/MRPT/mrpt.git

Next step: :ref:`compiling`

Debian/Ubuntu official repositories
---------------------------------------

Install the official version for your distribution with:

.. code-block:: bash

   sudo apt install libmrpt-dev mrpt-apps


.. note::
   Versions in official repositories may be quite outdated. It is strongly
   recommended to use the PPAs (read below) or build from sources instead.


Debian/Ubuntu PPA
----------------------

Last **stable release** (from the ``master`` branch), for Ubuntu >=18.04:

.. code-block:: bash

   sudo add-apt-repository ppa:joseluisblancoc/mrpt-stable   # master (stable releases) branch
   sudo apt install libmrpt-dev mrpt-apps

**Nightly builds** (from the ``develop`` branch) for Ubuntu >=18.04:

.. code-block:: bash

   sudo add-apt-repository ppa:joseluisblancoc/mrpt   # develop branch
   sudo apt install libmrpt-dev mrpt-apps

Last **stable release**, for Ubuntu 16.04 LTS Xenial (EOL: April 2021):

.. code-block:: bash

   # Install pre-requisites (** ONLY FOR Ubuntu 16.04 Xenial **)
   sudo add-apt-repository ppa:ubuntu-toolchain-r/test # gcc-7 Backport
   sudo add-apt-repository ppa:joseluisblancoc/mrpt-stable-xenial
   sudo apt-get update
   sudo apt-get install libmrpt-dev mrpt-apps

**Nightly builds**, for Ubuntu 16.04 LTS Xenial (EOL: April 2021):

.. code-block:: bash

   # Install pre-requisites (** ONLY FOR Ubuntu 16.04 Xenial **)
   sudo add-apt-repository ppa:ubuntu-toolchain-r/test # gcc-7 Backport
   sudo add-apt-repository ppa:joseluisblancoc/mrpt-unstable-xenial
   sudo apt-get update
   sudo apt-get install libmrpt-dev mrpt-apps


Windows installers
--------------------

Executables (.exes and .dlls) and development libraries (.hs and .libs) included:

   - `Last stable version <https://bintray.com/mrpt/mrpt-win-binaries/MRPT-nightly-builds/win64-stable>`_
   - `Nightly builds <https://bintray.com/mrpt/mrpt-win-binaries/MRPT-nightly-builds/win64-develop>`_
