#! /bin/bash
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
MRPT_USE_SEARCHENGINE="YES"
skipMAINMRPTDOCS=0

# Generate call graphs: obbey MRPT_genCALLERGRAPHS if defined. Yes, otherwise.
genCALLERGRAPHS=${MRPT_genCALLERGRAPHS:-YES}

emptyARGS=1
errARGS=0

while getopts 'chrlwsdo' OPTION
do
	case "$OPTION" in
	c) 	outCHM="YES"
		genHTML="YES"
		MRPT_USE_SEARCHENGINE="NO"
		emptyARGS=0
			;;
	h)	outHTML="YES"
		genHTML="YES"
		emptyARGS=0
			;;
	r)	genRTF="YES"
		emptyARGS=0
			;;
	l)	genLATEX="YES"
		emptyARGS=0
			;;
	w)	includeCounter="YES"
			;;
	s)	skipSVN="YES"
			;;
	d)	skipMAINMRPTDOCS=1
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
	echo " -d: Skip all .h files and just parse doc/* files" >&2
	exit 1
fi

# List of directories with header files (.h) and individual files to
# generate doc from.
#  Paths relative to "$MRPT_ROOT"
# -------------------------------------------------------------------
CUR_DIR=`pwd`
EXTRA_INDIV_FILES=`find libs -name '*SSE*.cpp' | xargs -I FIL printf "$CUR_DIR/FIL "`
VISION_CITATION_FILES="$CUR_DIR/doc/pnp_algos.bib"
if ( [ "$skipMAINMRPTDOCS" -eq "0" ] )
then
	MRPT_LIST_DIRECTORIES=$(echo $CUR_DIR/doc/doxygen-pages $CUR_DIR/libs/*/include/ $CUR_DIR/libs/*/src/ $CUR_DIR/samples/*/)
else
	MRPT_LIST_DIRECTORIES=$(echo $CUR_DIR/doc/doxygen-pages $CUR_DIR/samples/*/)
fi
MRPT_LIST_INPUT="$MRPT_LIST_DIRECTORIES $EXTRA_INDIV_FILES"

MRPT_EXAMPLE_PATH="$CUR_DIR/samples/ $CUR_DIR/share/mrpt/config_files/"

# Checks
# --------------------------------
if [ -f version_prefix.txt ]
then
	MRPT_VERSION_STR=`head -n 1 version_prefix.txt`
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
printf "\tPut a search box:\t\t%s\n" $MRPT_USE_SEARCHENGINE
printf "\tSystem type:\t\t%s\n" $SYSTEM_TYPE
echo ""
echo "List of directories: $MRPT_LIST_DIRECTORIES"
echo ""
echo " Script: build_docs.sh, by JLB. Bugs: <jlblanco@ctima.uma.es>"
echo "    Part of the MRPT project - http://www.isa.uma.es/jlblanco"
echo "--------------------------------------------------------------------"

sleep 2

# Obtain the SVN number:
MRPT_SVN_NUMBER=""

if [ -d .svn ]
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
fi

if [ -d .git ]
then
	printf "Obtaining Git commit id...\n"
	printf "Running..."
	MRPT_SVN_NUMBER="Git: `git log --pretty=format:'%h %cd' -n 1`"

	if [ $? -ne 0 ]
	then
		#echo "ERROR: svnversion returns an error code! Is SVN installed in the system???"
		#echo " You can use the argument -s to skip SVN number extraction. See README."
		#exit 1
		echo "WARNING: git returns an error code! Is git installed in the system???"
		MRPT_SVN_NUMBER = ""
	fi
	printf "OK (%s)\n" "$MRPT_SVN_NUMBER"
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
cp design_of_images/*.jpg images/
cp images/*.* html/
cp html_postbuild/*.* html/

# Build & copy PDF manuals:
for dir in ./{pbmap-guide,graphslam-engine-guide}; do
    cd $dir
    make 2> /dev/null
    mv *.pdf ..
    cd ..
done

rm html/*.pdf 2> /dev/null
cp *.pdf html/ 2> /dev/null

# Perf stats:
rm html/perf-html/* 2> /dev/null
mkdir html/perf-html 2> /dev/null
cp perf-data/perf-html/*.html html/perf-html/
printf "OK\n"

# If we are in windows, run the HHC directly:
if [ "$SYSTEM_TYPE" = "Windows" ]
then
	HHC_INVOKING_CODE="../../scripts/hhc.exe"
else
	HHC_INVOKING_CODE="../../scripts/run_hhc.sh"
fi
mkdir doc/chm

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
export MRPT_VERSION_STR
export MRPT_LIST_DIRECTORIES
export VISION_CITATION_FILES
export MRPT_LIST_INPUT
export MRPT_EXAMPLE_PATH
export genHTML
export outCHM
export genCALLERGRAPHS
export CHM_FILENAME
export HHC_INVOKING_CODE
export MRPT_USE_SEARCHENGINE
export genLATEX
export genRTF

printf "Generating DOXYGEN project..."
envsubst < doxygen_project.txt.in > doxygen_project.txt

printf "OK\n"

printf "Parsing header (.h.in) files for version variables..."
eval "echo \"`cat ../doc/doxygen-pages/mainPage_doc.h.in`\"" > ../doc/doxygen-pages/mainPage_doc.h
printf "OK\n"

printf "Generating the html footer..."

if [ "$includeCounter" = "YES" ]
then
	WEB_COUNTER_STR='<script type="text/javascript"> var sc_project=2835288; var sc_invisible=0; var sc_partition=28; var sc_security="862cd600";</script> <script type="text/javascript" src="http://www.statcounter.com/counter/counter_xhtml.js"></script><noscript> <div class="statcounter"><img class="statcounter" src="http://c29.statcounter.com/2835288/0/862cd600/0/" alt="free html hit counter" ></div></noscript>'

	# Note: The trailing "$" is to allow escaping the single quotes like in ANSI C:
	GOOGLE_ANALYTICS_STR=$'<script type="text/javascript"> var gaJsHost = (("https:" == document.location.protocol) ? "https://ssl." : "http://www."); document.write(unescape("%3Cscript src=\'" + gaJsHost + "google-analytics.com/ga.js\' type=\'text/javascript\'%3E%3C/script%3E")); </script> <script type="text/javascript"> try{ var pageTracker = _gat._getTracker("UA-21128561-2"); pageTracker._trackPageview(); } catch(err) {} </script>'
else
	WEB_COUNTER_STR=""
	GOOGLE_ANALYTICS_STR=""
fi

echo "<br><hr><br> <table border=\"0\" width=\"100%\"> <tr> <td> Page generated by <a href=\"http://www.doxygen.org\" target=\"_blank\">Doxygen `doxygen --version`</a> for $MRPT_COMPLETE_NAME at `date`</td><td>$WEB_COUNTER_STR</td> </tr> </table> $GOOGLE_ANALYTICS_STR </body></html>" > doxygen_footer.html

printf "OK\n"

# Execute doxygen:
doxygen doxygen_project.txt

# Cleanup
# -------------------------------------
rm doxygen_footer.html
#rm doxygen_project.txt

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
