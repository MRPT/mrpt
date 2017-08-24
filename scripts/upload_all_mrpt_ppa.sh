#!/bin/bash

find . -name '*.changes' | xargs -I FIL dput ppa:joseluisblancoc/mrpt-1.5 FIL
