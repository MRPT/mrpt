This directory contains the scripts I've been using to maintain Debian, Ubuntu and Fedora
packages for MRPT. They should NOT appear in source tarballs (that's the rule according
to Debian policies) but I'll keep them in SVN HEAD.
-----------------------------------------------------

Note: the file /ubuntu/control.in only has only minor changes wrt Debian's, 
with the goal of keeping compatibility with u16.04 (Xenial). It could be 
deleted when Xenial reaches EOL.

To create a Debian package from the sources:

- Go to the MRPT source root directory.
- Run: bash scripts/prepare_debian.sh

It will create a directory "/home/USER/mrpt_debian", and run "debuild -S -sa"
to build the source package. It will first create a new changelog entry with
debchange using the MRPT source version number, plus SVN version if available.


                   **** IMPORTANT NOTE ON THE SOURCES ****

The .orig source tarball used to create Debian distributions is a stripped version
without some Windows files and some extra libraries which are forced as dependencies
in Debian. If you plan to compile for Windows or other Linux configurations,
grab the original sources from https://www.mrpt.org/
