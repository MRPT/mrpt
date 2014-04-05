#!/bin/bash

find . -name '*.changes' | xargs -I FIL dput ppa:joseluisblancoc/mrpt-no-sse3 FIL

