#!/usr/bin/env bash

# Current script converts all the "//!< ..." doxygen inline comments into
# /**...*/ comments placed in the previous line.
# This runs on the provided file

if ! [[ $# -eq 1 ]]; then
	printf "./Usage convert_inline_doxygen_comments.sh <file_to_work_on>\n"
	printf "Exiting...\n"
	exit 1
fi

f=$1

# Modify all //!< ... comments to /** ... */ and place them in the previous line
# - Move the comment to the prevous line
# - Keep indentation level
sed -i''  "s/^\(\s\+\)\(.*\)\s*\/\/\!<\(.*\)$/\1\/\*\*\3 \*\/\\n\1\2/g" ${f}

# remove trailing spaces - run it only for statements (ending in ";"), not for
# e.g., macros.
# clang-format is coming up to do the cleanup...
sed -i'' "s/;\s*$/;/g" ${f}

exit 0
