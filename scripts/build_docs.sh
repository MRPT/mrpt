#! /bin/sh
# build_docs - Builds the doxygen documentation.
# Script: build_docs.sh
# Author: Jose Luis Blanco
# Bugs:  <jlblanco@ctima.uma.es>

# If genHTML=YES, HTML will be generated, but it will be deleted if outHTML!=1
genHTML="NO"
outHTML="NO"
outCHM="NO"
genLATEX="NO"
genRTF="NO"
includeCounter="NO"
skipSVN="NO"

emptyARGS=1
errARGS=0

while getopts 'chrlwso' OPTION
do 
	case "$OPTION" in
	h)	outHTML="YES"
		genHTML="YES"
		emptyARGS=0
			;;
	c) 	outCHM="YES"
		genHTML="YES"
		emptyARGS=0 
			;;
	l)	genLATEX="YES"
		emptyARGS=0 
			;;
	r)	genRTF="YES"
		emptyARGS=0 
			;;
	w)	includeCounter="YES"
			;;
	s)	skipSVN="YES"
			;;
	[?])	errARGS=1
			;;
	esac
done

if ( [ "$emptyARGS" -eq "1" ] || [ $errARGS -eq "1" ] )
then
	echo "Usage: $0 [-c] [-h] [-r] [-l] [-w] [-s] [-o]" >&2
	echo " Flags:" >&2
	echo " -h: Generate HTML documentation" >&2
	echo " -c: Generate CHM documentation" >&2
	echo " -r: Generate RTF documentation" >&2
	echo " -l: Generate LATEX/PDF documentation" >&2
	echo " -w: Include web visit counter & footer (select only for publishing the HTML files)" >&2
	echo " -s: Skip the SVN number (if your copy of MRPT has not been obtained from Subversion)" >&2
	exit 1
fi

# List of directories with header files (.h) to generate doc from:
#  Paths relative to "MRPT/doc"
# -------------------------------------------------------------------
CUR_DIR=`pwd`
MRPT_LIST_DIRECTORIES="$CUR_DIR/doc/doxygen-pages \
	$CUR_DIR/libs/base/include \
	$CUR_DIR/libs/gui/include \
	$CUR_DIR/libs/hmtslam/include \
	$CUR_DIR/libs/hwdrivers/include \
	$CUR_DIR/libs/maps/include \
	$CUR_DIR/libs/obs/include \
	$CUR_DIR/libs/opengl/include \
	$CUR_DIR/libs/reactivenav/include \
	$CUR_DIR/libs/slam/include \
	$CUR_DIR/libs/topography/include \
	$CUR_DIR/libs/detectors/include \
	$CUR_DIR/libs/vision/include"

# Checks
# --------------------------------
if [ -f version_prefix.txt ]
then
	MRPT_VERSION_STR=`cat version_prefix.txt`
else
	echo "ERROR: Cannot find the file version_prefix.txt!\nIt should be at the MRPT root directory."
	exit 1
fi

# The name of the system can be:
#  "CYGWIN_NT-XXX" or "MINGW-XXX"
#  "Linux"
#  other...
#
#  We will directly execute HHC.exe only in the case of CYGWIN, or 
#  through "wine" otherwise.
#-------------------------------------------------------------------
SYS_NAME=`uname -s`
MATCH_COUNT=`expr index "$SYS_NAME" 'GW'`

echo 
if [ $MATCH_COUNT -eq 0 ]
then
	SYSTEM_TYPE="Linux"
else
	SYSTEM_TYPE="Windows"
fi

echo "--------------------------------------------------------------------"
echo " The DOXYGEN documentation for the MRPT C++ library is to be built  "
echo " Options: "
printf "\tGenerate HTML docs:\t\t%s\n" $outHTML
printf "\tGenerate CHM docs:\t\t%s\n" $outCHM
printf "\tGenerate LATEX docs:\t\t%s\n" $genLATEX
printf "\tGenerate RTF docs:\t\t%s\n" $genRTF
printf "\tInclude web counter:\t\t%s\n" $includeCounter
printf "\tSkip SVN number:\t\t%s\n" $skipSVN
printf "\tSystem type:\t\t%s\n" $SYSTEM_TYPE
echo ""
echo "List of directories: $MRPT_LIST_DIRECTORIES"
echo ""
echo " Script: build_docs.sh, by JLB. Bugs: <jlblanco@ctima.uma.es>"
echo "    Part of the MRPT project - http://www.isa.uma.es/jlblanco"
echo "--------------------------------------------------------------------"

sleep 2

# Obtain the SVN number:
if [ "$skipSVN" = "NO" ]
then
	printf "Obtaining SVN number (This may take a *while* the first time, be patient)...\n"
	printf "Running svnversion..."
	MRPT_SVN_NUMBER="SVN:`svnversion .`"

	if [ $? -ne 0 ]
	then
		#echo "ERROR: svnversion returns an error code! Is SVN installed in the system???"
		#echo " You can use the argument -s to skip SVN number extraction. See README."
		#exit 1
		echo "WARNING: svnversion returns an error code! Is SVN installed in the system??? Ignoring SVN number"
		MRPT_SVN_NUMBER = ""
	fi
	printf "OK (%s)\n" $MRPT_SVN_NUMBER
else
	MRPT_SVN_NUMBER=""
fi

# Build the complete library name:
MRPT_COMPLETE_NAME="MRPT $MRPT_VERSION_STR $MRPT_SVN_NUMBER"
printf "The library complete name to be used is: %s\n" "$MRPT_COMPLETE_NAME"

cd doc/
echo "The cwd is: " `pwd`

# Load libs graph in DOT format:
MRPT_LIBS_DOT=`cat design_of_images/graph_mrpt_libs.dot`

# Create the new directory:
printf "Copying images..."
mkdir images 2> /dev/null
rm images/* 2> /dev/null
mkdir html 2> /dev/null
cp design_of_images/*.gif images/
cp design_of_images/*.png images/
cp design_of_images/*.map images/
cp screenshots/*.* images/
cp images/*.* html/
cp html_postbuild/*.* html/
printf "OK\n"

# If we are in windows, run the HHC directly:
if [ "$SYSTEM_TYPE" = "Windows" ]
then
	HHC_INVOKING_CODE="../../scripts/hhc.exe"
else
	HHC_INVOKING_CODE="../../scripts/run_hhc.sh"
fi

# Set the CHM file name:
CHM_FILENAME="libMRPT-$MRPT_VERSION_STR.chm"

# Create the DOXYGEN scripts by replacing constant values:
#  We parse the file doxygen_project.txt.in into the 
#  file doxygen_project.txt, by replacing:
#
#   $MRPT_COMPLETE_NAME
#   $MRPT_LIST_DIRECTORIES
#   ...
#
printf "Generating DOXYGEN project..."
eval "echo \"`cat doxygen_project.txt.in`\"" > doxygen_project.txt
printf "OK\n"

printf "Parsing header (.h.in) files for version variables..."
eval "echo \"`cat ../doc/doxygen-pages/mainPage_doc.h.in`\"" > ../doc/doxygen-pages/mainPage_doc.h
printf "OK\n"

printf "Generating the html footer..."

if [ "$includeCounter" = "YES" ]
then
	WEB_COUNTER_STR='<script type="text/javascript"> var sc_project=2835288; var sc_invisible=0; var sc_partition=28; var sc_security="862cd600";</script> <script type="text/javascript" src="http://www.statcounter.com/counter/counter_xhtml.js"></script><noscript> <div class="statcounter"><img class="statcounter" src="http://c29.statcounter.com/2835288/0/862cd600/0/" alt="free html hit counter" ></div></noscript>'

	HTML_VALID_LOGO='<a href="http://validator.w3.org/check?uri=referer" target="_blank"><img src="http://www.w3.org/Icons/valid-html401" border="0" alt="Valid HTML 4.01 Transitional" height="31" width="88"></a>'
	SOURCEFORGE_LOGO="<small>Hosted on:</small><br><a href=\"http://sourceforge.net\"><img src=\"http://sflogo.sourceforge.net/sflogo.php?group_id=205280&amp;type=1\" width=\"88\" height=\"31\" border=\"0\" alt=\"SourceForge.net Logo\" ></a>"
else
	WEB_COUNTER_STR=""
	HTML_VALID_LOGO=""
	SOURCEFORGE_LOGO=""
fi

echo "<br><hr><br> <table border=\"0\" width=\"100%\"> <tr> <td> Page generated by <a href=\"http://www.doxygen.org\" target=\"_blank\">Doxygen `doxygen --version`</a> for $MRPT_COMPLETE_NAME at `date`</td><td>$WEB_COUNTER_STR</td> <td width=\"100\">$HTML_VALID_LOGO </td> <td width=\"150\"> $SOURCEFORGE_LOGO </td></tr> </table> </body></html>" > doxygen_footer.html

printf "OK\n"

# Execute doxygen:
doxygen doxygen_project.txt

# Cleanup
# -------------------------------------
rm doxygen_footer.html
rm doxygen_project.txt

rm images/*

#if outHTML is NO, delete also intermediate HTML files:
if [ "$outHTML" = "NO" ]
then
	# Save these files only:
    FILES_TO_SAVE='changelog.html install.html'
	cd html
	mv -f $FILES_TO_SAVE ../
	cd ..
	
	# Remove all:
	rm html/*.html > /dev/null  2>&1
	rm html/*.png > /dev/null  2>&1

	# Save these files only:
	mv -f $FILES_TO_SAVE  html/
fi

cd ..

# Keep a valid command at the end to assure that ERRORLEVEL is "0" when executing the script in Windows
printf "Done!\n\n"



