#!/bin/bash
# Generates the doxygen page of each C++ example (from its README.md,
# screenshot and source file) and the examples.rst index listing them all.
# Invoked automatically by doc/Makefile.

set -e

cd "$(dirname "$0")/.."

OUT_MD_DIR=doc/source/doxygen-docs
OUT_RST=doc/source/examples.rst

rm -f $OUT_MD_DIR/example-*.md

cat > $OUT_RST <<'EOT'
.. _examples:

===============
C++ examples
===============

The source code for all these C++ examples can be found
under `MRPT/mrpt_examples_cpp <https://github.com/MRPT/mrpt/tree/develop/mrpt_examples_cpp>`_.

Python examples are `here <python_examples.html>`_.


.. toctree::
  :maxdepth: 1

EOT

for d in mrpt_examples_cpp/*/; do
	NAME=$(basename $d)
	SRC_FILE=main.cpp
	if [ ! -f $d/$SRC_FILE ]; then
		SRC_FILE=$(cd $d && ls -1 *.cpp 2>/dev/null | head -n1)
	fi
	if [ -z "$SRC_FILE" ]; then
		continue
	fi

	F=$OUT_MD_DIR/example-$NAME.md
	echo "\page $NAME Example: $NAME" > $F

	if [ -f $d/README.md ]; then
		echo "" >> $F
		cat $d/README.md >> $F
		echo "" >> $F
	fi

	FILE_SCREENSHOT=$(ls -1 doc/source/images/${NAME}_screenshot.* 2>/dev/null | head -n1)
	if [ -n "$FILE_SCREENSHOT" ]; then
		echo "" >> $F
		echo "![$NAME screenshot]($(basename "$FILE_SCREENSHOT"))" >> $F
	fi

	echo "C++ example source code:" >> $F
	echo "\include $NAME/$SRC_FILE" >> $F

	echo "  page_$NAME.rst" >> $OUT_RST
done
