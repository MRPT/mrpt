#!/bin/bash
# Creates or append lines to the pending UNRELEASED entry of changelog.
# JLBC, 2021

set -e   # end on error

source packaging/parse_mrpt_version.sh

cd packaging

set -x
DEBEMAIL="José Luis Blanco Claraco <joseluisblancoc@gmail.com>" \
debchange -b --newversion 1:${MRPT_VERSION_STR}-1 --distribution UNRELEASED $1
set +x

cd ..


exit 0
