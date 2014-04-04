#! /bin/sh
# run_hhc.sh - Runs the HHC.exe in a Windows/Linux independent way
# Author: Jose Luis Blanco
# Bugs:  <jlblanco@ctima.uma.es>

# The name of the system can be:
#  "CYGWIN_NT-XXX"
#  "Linux"
#  other...
#
#  We will directly execute HHC.exe only in the case of CYGWIN, or 
#  through "wine" otherwise.
# ----------------------------------------------------------------------
SYS_NAME=`uname -s`

MATCH_COUNT=`expr match "$SYSNAME" 'CYGWIN'`

if [ $MATCH_COUNT -eq 0 ]
then
	# Is Linux:
	echo "Running HHC in Linux. The command line is:"
	echo "../../scripts/hhc.exe $1 $2 $3"
	wine ../../scripts/hhc.exe $1 $2 $3	
	#echo "***************************************************************"
	#echo "Skipping CHM file creation: There is a unsolved bug (Aug 2007) "
	#echo " in wine so hhc.exe cannot be executed in Linux!! :-("
	#echo "***************************************************************"
else
	# Is Windows:
	echo "Running HHC in Windows. The command line is:"
	echo "../../scripts/hhc.exe $1 $2 $3"
	../../scripts/hhc.exe $1 $2 $3
fi

