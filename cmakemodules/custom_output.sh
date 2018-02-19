#!/bin/bash
SOURCE_FILE="${@: -1:1}"
SOURCE_FILE_NAME=`basename \"$SOURCE_FILE\" | tr -d '"'`

TIME_OUTPUT=`/usr/bin/time --format=" %E real, %U user, %S sys" $@ 2>&1 |tr '\n' ' '`
echo "-BUILD TIME: $TIME_OUTPUT $SOURCE_FILE_NAME"
