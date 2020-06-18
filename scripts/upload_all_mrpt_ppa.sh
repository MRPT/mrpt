#!/bin/bash

find xenial -name '*.changes' | xargs -I FIL dput ppa:joseluisblancoc/mrpt-unstable-xenial FIL
rm -fr xenial || true
find . -name '*.changes'      | xargs -I FIL dput ppa:joseluisblancoc/mrpt FIL
