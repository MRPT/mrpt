# Building MRPT documentation
The following targets can be generated:
  - HTML files: Will be generated at "doc/html"
  - (Obsolete) A CHM file: A compressed, searchable file. Generated at the root directory.

Or any combinations of them. Instructions for building the docs:

## In Windows
Run the "build_docs_XXX.bat" file, according to the files you want to generate.

### PREREQUISITES
- Doxygen: Available in http://www.doxygen.org
- Cygwin: Available in http://www.cygwin.com/
- Remember to put the programs in the PATH!

## In GNU/Linux
Go to the "MRPT/scripts" directory and run:
    $ ./build_docs.sh -h -c
or:
    $ sh build_docs.sh -h -c

Usage: build_docs.sh: [-h] [-c] [-w]
 Flags:
  -h    : Generate HTML documentation
  -c    : Generate the CHM documentation
  -w    : Include a web visit counter (select only when publishing the HTML files)

## PREREQUISITES
- Doxygen
