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
    if os.path.exists("../version_prefix.txt"):
        os.chdir('..')

    if not os.path.exists("./version_prefix.txt"):
        print "ERROR: Invoke this script from the <MRPT> ROOT directory"
        return 1  # end

    # Make sure our CWD is the root of MRPT source tree:
    assert os.path.exists("libs")

    print "Enter the name of the new library (example, for 'mrpt-foo', enter 'foo'):"
    NewLibName = raw_input(">")

    # Replace string lists:
    sFrom = ['@NAME@', '@name@']
    sTo = [string.upper(NewLibName), string.lower(NewLibName)]

    parseDir = "parse-files/new_lib_bootstrap/"
    baseDir  = "libs/" + NewLibName +"/"

    assert os.path.exists(parseDir)
    assert not os.path.exists(baseDir)
    
    # Create new lib:
    # --------------------------------
    print "Creating new lib in directory: " + baseDir
    
    os.mkdir(baseDir)   # Create dirs
    os.mkdir(baseDir+"include")
    os.mkdir(baseDir+"include/mrpt")
    os.mkdir(baseDir+"include/mrpt/"+ string.lower(NewLibName) )
    os.mkdir(baseDir+"src")
    
    replaceInFile(
        parseDir + 'CMakeLists.txt.in',
        baseDir + 'CMakeLists.txt',
        sFrom, sTo)

    replaceInFile(
        parseDir + 'include/mrpt/name.h.in',
        baseDir  + 'include/mrpt/'+ string.lower(NewLibName) + '.h',
        sFrom, sTo)

    replaceInFile(
        parseDir + 'include/mrpt/name/link_pragmas.h.in',
        baseDir  + 'include/mrpt/'+ string.lower(NewLibName) + '/link_pragmas.h',
        sFrom, sTo)

    replaceInFile(
        parseDir + 'src/precomp_hdr.cpp.in',
        baseDir  + 'src/precomp_hdr.cpp',
        sFrom, sTo)

    replaceInFile(
        parseDir + 'src/registerAllClasses.cpp.in',
        baseDir  + 'src/registerAllClasses.cpp',
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
