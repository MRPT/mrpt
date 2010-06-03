
  ARIA
  MobileRobots Advanced Robotics Interface for Applications

  Version 2.5.1

Copyright 2002,2003,2004,2005 
ActivMedia Robotics, LLC. All rights reserved.

Copyright 2006, 2007
MobileRobots Inc. All rights reserved.

Please read this document for important details about using ARIA.

   Contents
   ========
*  Introduction
*  Documentation
*  Licenses and Sharing
*  ARIA Package
*  Files of Note
*  Compiling programs that use ARIA
   *  Windows
   *  Linux
      *  Using ARIA's Makefiles
      *  Using your own Makefile or other build system
      *  Learning more about using Linux
   *  Learning more about C++
*  Using ARIA from Python and Java
*  Simulator

Welcome to the MobileRobots Advanced Robotics Interface for Applications (ARIA).
ARIA is an object-oriented, application programming interface (API) for
MobileRobots' (and ActivMedia) line of intelligent mobile robots, including
Pioneer 2/3 DX and AT, PeopleBot, PowerBot, and AmigoBot mobile robots.
Written in the C++ language, ARIA is client-side software for easy,
high-performance access to and management of the robot server, as well
to the many accessory robot sensors and effectors, and useful utilities. 
   
ARIA can be run multi- or single-threaded, using its own wrapper
around Linux pthreads and WIN32 threads.  Use ARIA in many different
ways, from simple command-control of the robot server for direct-drive
navigation, to development of higher-level intelligent actions (aka
behaviors).   Add your own features and modifications to ARIA: It's 
open-source!

The ARIA package includes both source code and pre-build libraries
and example programs. These libraries and programs were build with
GCC 3.4 if on Linux, and MS Visual Studio .NET 2003 (7.1) if on
Windows.  Using these compilers for development is recommended. If
you use a different compiler, you must rebuild the ARIA libraries.

See below for more information about building programs with ARIA
on Windows and Linux and using the Windows and Linux development tools.


Documentation and Help
======================

Follow the INSTALL text instructions to install ARIA on your Linux or 
Windows workstation. System requirements are included in the INSTALL 
text.

ARIA includes a full API Reference Manual in HTML format. This manual,
Aria-Reference.html (and in the docs/ directory), includes documentation 
about each of the classes and methods in ARIA, as well as a comprehensive 
overview describing how to get stated understanding and using ARIA.  In 
addition, ARIA includes several example programs in the examples/ and 
advanced/ directories, and the header files and source code are included
in include/ and src/ as well.  

The ArNetworking library has its own reference manual,
ArNetworking-Reference.html in the ArNetworking subdirectory, and examples
in ArNetworking/examples.

If you plan on using the Java or Python wrapper libraries, see the 
javaExamples, pythonExamples, ArNetworking/javaExamples and
ArNetworking/pythonExamples directories for important information in
README files, and example programs. You should also read the Aria Reference
manual for general information about Aria -- the API in the wrapper libraries
are almost identical to the C++ API.

If you have any problems or questions using ARIA or your robot, the 
MobileRobots support site provides:
 
  * A FAQ (Frequently Asked Questions) list, at <http://robots.mobilerobots.com/FAQ.html>
  * The Aria-Users mailing list, where you can discuss Aria with other users and 
    MobileRobots software developers:
      * Search the archives at <http://robots.mobilerobots.com/archives/aria-users/threads.html>
      * Join the list at <http://robots.mobilerobots.com/archives/aria-info.html>


License and Sharing
===================

ARIA is released under the GNU Public License, which means that if you
distribute any work which uses ARIA, you must distribute the entire 
source code to that work.  Read the included LICENSE text for details.
We open-sourced ARIA under GPL not only for your convenience, but also 
so that you will share your enhancements to the software.  If you wish 
your enhancements to make it into the ARIA baseline, you will need to 
assign the copyright on those changes to MobileRobots, contact 
lafary@mobilerobots.com with these changes or with questions about this.

Accordingly, please do share your work, and please sign up for the 
exclusive ARIA-users@activmedia.com newslist so that you can benefit 
from others' work, too.

ARIA may instead be licensed for proprietary, closed source applications. 
Contact sales@mobilerobots.com for details.

For answers to frequently asked questions about what the GPL allows
and requires, see <http://www.gnu.org/licenses/gpl-faq.html>


The ARIA Package
================

Aria/:
  docs      The API Reference Manual: Extensive documentation of all of ARIA
  examples  ARIA examples -- a good place to start; see examples/README.txt
  include   ARIA header (*.h) files
  src       ARIA source code (*.cpp) files
  lib       Libraries (.lib files for Windows, .so files for Linux) 
  bin       Contains Windows binaries and DLLs.
  params    Robot definition (parameter) files ("p3dx.p", for example)
  tests     Test files, somewhat esoteric but useful during ARIA development
  utils     Utility commands, not generally needed
  advanced  Advanced demos, not for the faint of heart (or ARIA novice)
  python    Python wrapper package
  java      Java wrapper package

Aria/ArNetworking/:  (A library used to facilitate network communication)
  docs      API Reference Manual for ArNetworking 
  examples  ArNetworking examples
  include   ArNetworking header (*.h) files, of course
  src       ArNetworking source (*.cpp) files
  python    Python wrapper package
  java      Java wrapper package
  

Other ARIA Files of Note
========================

LICENSE.txt   GPL license; agree to this to use ARIA
Aria.sln      MSVC++ solution for building ARIA and the 'demo' program
examples/examples.sln MSVC++ solution for building all of the examples
Makefile      Linux makefile for building ARIA and examples
Makefile.dep  Linux dependency file
INSTALL.txt   Step-wise instructions for installing ARIA
README.txt    This file; also see READMEs in advanced/, examples/, and tests/


Compiling a new file(s)
====================

Windows
-------

In order to compile ARIA and the examples, you can use Microsoft 
Visual C++ version 7.1 (2003) or version 8 (2005), however ARIA comes 
prebuilt for version 7.1 only. 

Visual Studio c++ 8 (2005) is not compatible with  previous versions, 
so to use version 8, you must rebuild all ARIA libraries. You must also 
install the seperate Platform SDK, and add its include and lib directories 
to VC8's includes and libraries directories in Options. 

Note: If you must use an older version, you must rebuild the ARIA 
libraries, but MobileRobots only supports Visual Studio .NET 2003.

Open the solution file you want to add your new project to (using
"File->Open Workspace"), e.g. Aria.sln, or make a new solution 
(using "File->New->Blank Solution").

Select "File->New->Project", select a console application.  Click the button to
add to exisiting solution, give it a name. (Delete that name from the 
'location' field if you want to avoid using a subdirectory.)

Now you must configure your project to find ARIA. These instructions
assume a default installation; if you installed ARIA elsewhere you
must use different paths. If you keep your Visual Studo project within
Aria's directory, you can also use relative paths (e.g. "..\lib" for the
library path).

* From the Project Menu, choose to edit your projects Properties. Or
  right-click on the project in the Solution Explorer and choose Properties.

* Change the "Configuration" (upper left of the dialog) menu to "All
  Configurations".

* Click on the "General" section

  * Set "Output Files" to "C:\Program Files\MobileRobots\Aria\bin" 
    (Aria's bin directory, where Aria.dll etc. are).  Or, if you want your
    program saved in a different directory, you must put Aria's bin directory
    in your system PATH environment variable, or copy Aria.dll.

  * You may want to change Intermediate Files. A useful value is
    "obj\$(ConfigurationName)";  $(ConfigurationName) will be either "Debug" 
    or "Release".

  * Set "Use Managed Extensions" to "no".

* Click on the "Link" or "Linker" section.

  * Click on the "Input" subsection.

    * To "Additional Library Path" add "C:\Program Files\MobileRobots\Aria\lib".

    * Under "Object/Library Modules", at the beginning of the line, insert
      "Aria.lib winmm.lib wsock32.lib".  Also include ArNetworking.lib if
      you are using ArNetworking.

* Click on the "C++" section.

  * Click on the "General" subsection

    * To "Additional Include Directories" add 
      "C:\Program Files\MobileRobots\Aria\include".

   * Click on the "Code Generation" subsection

     * Change the "Configuration" menu (upper left of the window) to "Debug".

       * Under the "Use run-time library" pulldown select "Debug
         Multithreaded".

     * Change the "Configuration" menu (upper left of the screen) to "Release".

       * Under the "Use run-time library" pulldown select "Multithreaded".

     * Change the "Configuration" menu back to "All Configurations"


Linux
-----

On GNU/Linux two tools are used: the compiler (GCC), which compiles
and/or links a single file, and Make, which is used to manage 
building multiple files and their dependencies.  If you need to
deal with multiple files, you can copy a Makefile and modify it, 
or create a new one.

Note: In all packages for RedHat and Debian, the shared libraries
were build with G++ 3.4. You must use G++ 3.4 or a compatible version to
build programs that link against it, or rebuild ARIA using your 
preferrend G++ version. G++ 3.4 has been preinstalled on robot on-board 
computers with RedHat 7.3 in /usr/local/gcc-3.4.1, and it has been configured
as the default gcc and g++ compilers on Debian systems as well. (On other
Debian systems, the default g++ and gcc compiler versions may be changed
using the 'update-alternatives' command). You may need to install it 
yourself on your own system if not already present. When compiling
ARIA or its examples, you may also temporarily override the C++ compiler
command ('g++' by default) by setting the CXX environment variable before 
or while running make. E.g.: "make CXX=g++-3.4" or "export CXX=g++-3.4; make".

More information and documentation about GCC is available at 
<http://gcc.gnu.org>, and in the gcc 'man' page.  More information 
about Make is at <http://www.gnu.org/software/make/>. Several graphical 
IDE applications are available for use with GCC, though none are 
currently supported by MobileRobots.



# Using ARIA's Makefiles on Linux: #

The ARIA Makefile is able to build any program consisting 
of one source code file in the 'examples', 'tests', or 'advanced' 
directory.  For example, if your source code file is 
'examples/newProgram.cpp', then you can run 'make examples/newProgram' 
from the top-level Aria directory, or 'make newProgram' from within 
the 'examples' directory, to build it.   This makes it easy to copy
one of the example programs to a new name and modify it -- a good way
to learn how to use ARIA.



# Using ARIA from your own Makefile or other build system: #

If you want to keep your program in a different place than the Aria 
examples directory, and use your own Makefile or other build tool,
you need to use the following g++ options to compile the program:

 -fPIC -I/usr/local/Aria/include 

If you wish, you may also use -g and -Wall for debugging information
and useful warnings.   Aria does not use exceptions, so you may also
use -fno-exceptions if you wish; this will cause any use of exceptions
in your program to trigger a compile error.

For linking with libAria use these options:

  -L/usr/local/Aria/lib -lAria -lpthread -ldl

If you are also using ArNetworking, use the following compile option
in addition to the Aria options above:

  -I/usr/local/Aria/ArNetworking/include
  
And for linking to libArNetworking:

  -lArNetworking




# Learning more about using Linux: #

If you are new to using GNU/Linux, some guides on getting started can be 
found at the following sites:

 * If your robot is using RedHat (it displays "RedHat Linux" on the console), 
   start with
   <https://www.redhat.com/docs/manuals/linux/RHL-7.3-Manual/getting-started-guide/>.
   More at <https://www.redhat.com/support/resources/howto/rhl73.html>.

 * If your robot is using Debian, start with 
   <http://www.us.debian.org/doc/manuals/debian-tutorial>. More is at
   <http://www.debian.org/doc/>.

 * <http://www.tldp.org> is a repository of many different guides, FAQ and
   HOWTO documents about Linux in general, including various programming
   and system administration tasks.

For more depth, there are many books about using Linux, consult your
local computer bookseller. The ideal way to learn about Linux is to work 
with an experienced colleague who can demonstrate things and answer 
questions as they arise.



Learning C++
------------

If you are new to C++ programming, the definitive guide is Bjarne
Stroustroup's book "The C++ Programming Language".   The book
"C++ In a Nutshell", published by O'Reilly & Associates, is a
good quick guide and reference. There are also several websites
and many other books about C++.


Using ARIA from Java or Python
==============================

Even though ARIA was written in C++, it is possible to use ARIA from
the Java and Python programming languages as well. The directories
'python' and 'java' contain Python and Java packages automatically generated
by the 'swig' tool (http://www.swig.org) which mirror the ARIA API
closely, and are "wrappers" around the native ARIA library: When you call
an ARIA function in a Python or Java program, the wrapper re-marshalls the
function arguments and makes the equivalent call into the C++ ARIA library.
See the 'pythonExamples' and 'javaExamples' directories for more information
and example programs.  See the ARIA API Reference Manual in 'docs' for 
some more information.

More about the Python programming language is at <http://www.python.org>.
More about the Java programming language is at <http://java.sun.com>.


MobileSim Simulator
===================

SRIsim is no longer included with Aria.  There is now a seperately
downloadable MobileSim simulator at our support webpage 
<http://robots.mobilerobots.com>.  MobileSim is compatible with SRISim,
but adds many new features.




-------------------------
MobileRobots, Inc.
19 Columbia Drive
Amherst, NH, 03031. USA

support@mobilerobots.com
http://www.activrobots.com


