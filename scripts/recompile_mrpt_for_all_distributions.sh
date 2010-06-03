#!/bin/bash
# This script first creates a Debian source package for the current SVN version of MRPT,
#  then compile it using pbuilder for different Debian & Ubuntu distributions and 
#  architectures, and then uploads them to the babel.isa.uma.es debian repository.
#  Jose Luis Blanco, Aug 2008
# Usage: From MRPT svn source root: 
#   scripts/recompile_mrpt_for_all_distributions.sh
#
# Requires .pbuilderrc as in: https://wiki.ubuntu.com/PbuilderHowto
#

echo "FIRST STEP: Building Debian Source package".
source scripts/prepare_debian.sh


echo ""
cd $HOME/mrpt_debian
if [ `pwd` != "$HOME/mrpt_debian" ];
then
	exit 1;
fi

# Prepare temp directory:
TMPDIR="/tmp/mrptdeb"
echo "Starting with a fresh $TMPDIR..."
rm -fR $TMPDIR
mkdir $TMPDIR
mkdir $TMPDIR/dists

DISTs=( hardy    hardy    ) #intrepid intrepid  sid   sid  )
COMPs=( universe universe ) #universe universe  main  main )
ARCHs=( amd64    i386     ) #amd64    i386      amd64 i386 )

# Go thru the array of builds:
for (( i = 0 ; i < ${#ARCHs[*]} ; i++ ))
do
	DIST=${DISTs[$i]}
	ARCH=${ARCHs[$i]}
	COMP=${COMPs[$i]}
	echo " ========== COMPILING FOR $DIST $ARCH ($COMP) ========== "
	mkdir $TMPDIR/dists/$DIST/
	mkdir $TMPDIR/dists/$DIST/$COMP/
	mkdir $TMPDIR/dists/$DIST/$COMP/source
	mkdir $TMPDIR/dists/$DIST/$COMP/binary-$ARCH

	# Clear old contents:
	rm /var/cache/pbuilder/$DIST-$ARCH/result/*
	pbuilder build $HOME/mrpt_debian/*.dsc

	# Now the *.deb files and source pckg are in /var/cache/pbuilder/$DIST-$ARCH/result/
	mv /var/cache/pbuilder/$DIST-$ARCH/result/*.deb $TMPDIR/dists/$DIST/$COMP/binary-$ARCH/
	mv /var/cache/pbuilder/$DIST-$ARCH/result/*.dsc $TMPDIR/dists/$DIST/$COMP/source/
	mv /var/cache/pbuilder/$DIST-$ARCH/result/*.gz $TMPDIR/dists/$DIST/$COMP/source/

	# Generate index files:
	dpkg-scanpackages $TMPDIR/dists/$DIST/$COMP/binary-$ARCH/ /dev/null | gzip -9c > $TMPDIR/dists/$DIST/$COMP/binary-$ARCH/Packages.gz
	dpkg-scansources $TMPDIR/dists/$DIST/$COMP/source/ /dev/null | gzip -9c > $TMPDIR/dists/$DIST/$COMP/source/Sources.gz

done

