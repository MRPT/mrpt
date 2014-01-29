#!/bin/bash

#
# Careful: this script changes "it++,i++" to "++it++,i"
#

LST1=$(find . -name '*.cpp') 
LST2=$(find . -name '*.cxx') 
LST3=$(find . -name '*.h') 
LST4=$(find . -name '*.hpp')

for fl in $LST1 $LST2 $LST3 $LST4;do
	cat $fl | sed '/.*for.*iterator/ s/\([a-z,A-Z]*\)++/++\1/g' > A.tmp && mv A.tmp $fl
done

