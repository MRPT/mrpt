#!/usr/bin/env bash

# Current file provides a wrapper around the
# clang_git_format/scripts/convert_inline_dox_comments_file.sh script, so that
# the latter is executed in multiple files:
# - If no extra arguments are given it runs on the whole MRPT codebase
# - If two git commits are given it runs on all the .h .cpp files included in
# these commits.
# - If multiple filepaths are given it runs on each one of the files
# individually

function commit_exists() {

ret=
if git cat-file -e $1^{commit} 2>/dev/null; then
    ret=0
else
    ret=1
fi
return $ret

}

if ! [ -f version_prefix.txt ]
then
	echo "ERROR: Cannot find the file version_prefix.txt!"
	echo "It should be at the MRPT root directory."
	echo "Change to the root directory and rerun script."
	exit 1
fi

convert_script="scripts/clang_git_format/scripts/convert_inline_dox_comments_file.sh"

# find which files to run on
valid_files=
if [ "$#" -eq 0 ]; then
    valid_files=$(find apps libs samples python -iname "*.cpp" -o -iname "*.h")
elif [ "$#" -eq 2 ] &&  $(commit_exists $1) && $(commit_exists $2) ; then
    valid_files=$(git diff --name-only $1..$2)
    valid_files=$(echo ${valid_files} | xargs -d' ' -n 1 | grep -ie ".*\.h$\|.*\.cpp$")
else
    valid_files="${@:2}"
fi

printf "Valid files: ${valid_files}\n"

for f in ${valid_files}; do
    printf "Working on ${f}\n"
    bash ${convert_script} ${f}
done

exit 0
