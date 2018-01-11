#!/usr/bin/python
# Python script to bootstrap the creation of a new library within MRPT
#  Invoke without arguments from the MRPT root directory, that is:
#   $ scripts/bootstrap_new_lib.py
#
#  or from the MRPT/scripts directory:
#   $ ./bootstrap_new_lib.py
#
#  Script by Jose Luis Blanco, July 2010.

import os
import sys
import re
import string

#------   MAIN   -------
def main():
	if os.path.exists(os.path.normpath("../version_prefix.txt")):
		os.chdir('..')

	if not os.path.exists(os.path.normpath("./version_prefix.txt")):
		print("ERROR: Invoke this script from the <MRPT> ROOT directory")
		return 1  # end

	# Make sure our CWD is the root of MRPT source tree:
	assert os.path.exists("libs")

	print("Enter the name of the new library (example, for 'mrpt-foo', enter 'foo'):")
	NewLibName = input(">")

	# Replace string lists:
	sFrom = ['@NAME@', '@name@']
	sTo = [NewLibName.upper(), NewLibName.lower()]

	parseDir = os.path.normpath("parse-files/new_lib_bootstrap")
	baseDir  = os.path.normpath("libs/" + NewLibName )

	assert os.path.exists(parseDir)
	assert not os.path.exists(baseDir)

	# Create new lib:
	# --------------------------------
	print("Creating new lib in directory: " + baseDir)

	os.mkdir(baseDir)   # Create dirs
	os.mkdir(os.path.normpath(baseDir+"/include"))
	os.mkdir(os.path.normpath(baseDir+"/include/mrpt"))
	os.mkdir(os.path.normpath(baseDir+"/include/mrpt/"+ NewLibName.lower() ))
	os.mkdir(os.path.normpath(baseDir+"/src"))

	replaceInFile(
		os.path.normpath(parseDir + '/CMakeLists.txt.in'),
		os.path.normpath(baseDir + '/CMakeLists.txt'),
		sFrom, sTo)

	replaceInFile(
		os.path.normpath(parseDir+'/include/mrpt/name.h.in'),
		os.path.normpath(baseDir+'/include/mrpt/'+ NewLibName.lower() + '.h'),
		sFrom, sTo)

	replaceInFile(
		os.path.normpath(parseDir+'/src/name-precomp.cpp.in'),
		os.path.normpath(baseDir+'/src/'+ NewLibName.lower() +'-precomp.cpp'),
		sFrom, sTo)
	replaceInFile(
		os.path.normpath(parseDir+'/src/name-precomp.h.in'),
		os.path.normpath(baseDir+'/src/'+ NewLibName.lower() +'-precomp.h'),
		sFrom, sTo)

	replaceInFile(
		os.path.normpath(parseDir+'/src/registerAllClasses.cpp.in'),
		os.path.normpath(baseDir+'/src/registerAllClasses.cpp'),
		sFrom, sTo)


	return 0  # OK
#-----------


def replaceInFile(fileIn, fileOut, textToSearch, newText) :
	o = open(fileOut,"w")
	data = open(fileIn).read()
	for idx in range(len(textToSearch)):
		data = re.sub(textToSearch[idx], newText[idx], data)
	o.write(data)
	o.close()
#-----------
    

if __name__ == "__main__":
	sys.exit(main())
