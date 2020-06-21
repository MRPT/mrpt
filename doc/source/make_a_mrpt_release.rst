
(These notes are mostly for myself (J.L. Blanco), but hopefully they'll be 
 useful if someone else maintains MRPT in the future...)

  MRPT release check-list:
=============================

1) CREATE PACKAGES OF SOURCES
> Go to MRPT dir. 
> Edit docs/doxygen-pages/changelog.h and set release date.
> bash scripts/build_docs.sh -c (from Windows, or everything from linux, then CHM from Windows)
> Do the final "git commit" before the release.
> bash scripts/prepare_release.sh
> bash scripts/prepare_debian.sh

The packages are in $(HOME)/mrpt_release and $(HOME)/mrpt_debian

Now for windows binary packages:
(see also the script: MRPT/scripts/automated_build_msvc_binary_package.bat)
> Extract mrpt-x.y.z.zip
> Run the script: automated_create_all_windows_MSVC_MinGW_build_dirs.bat

2) DEBIAN PACKAGE
> Go to mrpt_debian/debian
> Edit changelog
> Go to mrpt_debian
> debuild -S -sa

Test with: "lintian *.changes"

3) Try compilation in Debian Unstable
> sudo ARCH=amd64 DIST=sid pbuilder --update
> sudo ARCH=amd64 DIST=sid pbuilder --build  MRPT*.dsc

Test all binary pkgs with: "lintian *.deb"


