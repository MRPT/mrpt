#!/bin/bash

echo "List to copy to examples.rst:"

for d in samples/*/ ; do
    NAME=$(basename $d)
    F=doc/source/doxygen-docs/example-$NAME.md
    echo "\page $NAME Example: $NAME" > $F

    FILE_DESCRIPTION="$d/README.md"
    if [ -f $FILE_DESCRIPTION ]; then
        echo "" >> $F
        cat $FILE_DESCRIPTION >> $F
        echo "" >> $F
    fi

    FILE_SCREENSHOT="doc/source/images/${NAME}_screenshot.png"
    if [ -f $FILE_SCREENSHOT ]; then
        echo "" >> $F
        echo "![$NAME screenshot]($FILE_SCREENSHOT)" >> $F
    fi

    echo "C++ example source code:" >> $F
    echo "\include $NAME/test.cpp" >> $F

    echo "  page_$NAME.rst"
done
