.. _tutorial_lib_layout:

############################
MRPT libraries tree layout
############################

.. note::
    This page describes the internal structure of MRPT sources, which is
    intended for MRPT developers/contributors. More often, users will be
    interested in writing their own programs that use MRPT, as described here:

    :ref:`mrpt_from_cmake`
.. contents:: :local:


1. Tree overview
----------------

The MRPT source tree looks like this:

-  MRPT

   -  apps: \ *Applications*
   -  libs: \ *C++ libraries*

      -  ...

The part of interest here is the "libs" directory, where there are a
number of **libraries** or **modules**, all with a common structure, which is
described here.

2. Name
-------

The name of all the libraries must be \ **"mrpt-<name>"**, where <name>
is a lowercase word describing the module. Examples are "slam", "math",
or "vision". Note that the directories under "MRPT/libs/" are named
"<name>", \ **not** "mrpt-<name>", while the library itself to be
generated after building will be mrpt-<name>.dll (or .so for UNIX
systems).

3. Tree outline
---------------

Each "module" mrpt-<name> must have, at least, these files and
directories:

-  MRPT

   -  libs: \ *C++ libraries*

      -  **name**

         -  src

            -  **``precomp_hdr.cpp``**\ (*This file must only contain
               a:* ``#include <mrpt/name.h> ``)
            -  registerAllClasses.cpp \ *(optional: used to register all
               ```CSerializable`` <http://www.mrpt.org/tutorials/programming/serialization/>`__\ objects
               so they can be de-serialized without knowing their type
               in advance)*
            -  *(the rest of source files)*

         -  include

            -  mrpt

               -  **name**

                  -  **link\_pragmas.h**
                  -  *(the rest of include files)*

               -  **``name.h``** (*this will be the precompiled
                  header:* ``#include <mrpt/name.h> ``)

         -  CMakeLists.txt

For template contents for each of the required files, please take a look
at some of the simplest libraries, like ``mrpt-topography``
under ``MRPT/libs/topography``. Note that this structure keeps the
header files of each library separately, so the user of MRPT must
specify the dependencies in his/her CMake configuration file, as
described here: :ref:`mrpt_from_cmake`

4. The *CMakeLists.txt* file
------------------------------

If there are no complex conditional compiling options or any other
complications, declaring an MRPT library is as simple as writing just
the following in its ``CMakeListst.txt`` file:

.. code-block:: cmake

  #---------------------------------------------
  # Macro declared in "DeclareMRPTLib.cmake":
  #---------------------------------------------
  define_mrpt_lib(
  # Lib name
  name
  # Dependencies
  mrpt-math
  ....
  )

Notice that files matching the patterns ``*_LIN.cpp`` and ``*_WIN.cpp`` are
automatically ignored when building under Windows or Linux/MacOS, respectively.
For the usage of more advanced features, please refer to DeclareMRPTLib.cmake or
the scripts for the existing libraries.

5. Unit testing
------------------------------
The MRPT CMake scripts automatically recognize those files under the src
directory with the pattern ``*_unittest.cpp`` as files for unit testing.
MRPT uses the Google’s gtest library to perform these tests.

To see how to invoke the regular unit tests, or how to run them automatically
under ``gdb`` or ``valgrind``, see: :ref:`unit_testing`.

Note that the ``*_unittest.cpp`` files are not included in the "mrpt-<name>”
library, but a new target ``test-<name>`` is automatically created which is
invoked when doing ``make test`` or ``make test_legacy``.
A typical content of a unit testing file is (see existing tests for more examples):


.. code-block:: cpp

  #include <mrpt/slam.h>
  #include <gtest/gtest.h>


  TEST(MyTestCaseName, ThisTestName)
  {
  double err;
  // Do whatever...
  EXPECT_EQ(0, err ) << "Differences were found when checking ...\n";
  }
