#!/usr/bin/env python

import argparse
import os
import platform
import sys
import shutil
import subprocess

DEFAULT_MRPT_VERSION = '1:1.3.0-1'

# args
parser = argparse.ArgumentParser()
parser.add_argument('-b', '--build_dir', help='Path to the build directory.')
parser.add_argument('-m', '--mrpt_version', help='The MRPT lib version those bindings are generated from. DEFAULT: {}'.format(DEFAULT_MRPT_VERSION))
parser.add_argument('-a', '--architecture', required=True, help='The architecture the bindings are built for (i386, amd, armhf, etc.).')
args = parser.parse_args()

# check requirements
def check_required_program(program):
    try:
        subprocess.call(['which', program])
    except:
        print 'Required program "{}" not found.'.format(program)
        sys.exit(1)
print 'Looking for required programs:'
check_required_program('sudo')
check_required_program('dpkg')
check_required_program('chrpath')
check_required_program('lintian')

# get supplied build path, otherwise set default
if args.build_dir:
    build_dir = args.build_dir
else:
    build_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'build')

# save current dir
curr_dir = str(os.path.abspath(os.path.curdir))

# find pymrpt.so in build dir
built_lib = os.path.join(build_dir, 'lib', 'pymrpt.so')
if os.path.exists(os.path.join(built_lib)):
    print 'Found local build: "{}".'.format(built_lib)
else:
    print 'Could not find "pymrpt.so" in {}. Supply path to build directory as argument!'.format(build_dir)
    print 'Example: {} <path/to/build>'.format(sys.argv[0])
    print '\nExit.'
    sys.exit(1)

# TODO find version automatically
built_lib_version = args.mrpt_version if args.mrpt_version else DEFAULT_MRPT_VERSION
built_lib_size = os.path.getsize(built_lib)
built_lib_arch = args.architecture

# user home
home_dir = os.path.expanduser('~')

# packaging dir
pkg_dir = os.path.join(home_dir, 'mrpt-python-bindings')

# check if packaging dir exists
if os.path.exists(pkg_dir):
    cont = raw_input('Packaging dir already exists. Continue anyway? (y/N): ')
    if cont != 'y':
        print '\nExit.'
        sys.exit(0)
    # remove existing dir (utilizing sudo)
    try:
        command = 'rm -rf {}'.format(pkg_dir)
        subprocess.call(["/usr/bin/sudo", "sh", "-c", command])
    except:
        raise
    print 'Removed existing packaging dir.'

print 'Preparing:'
print '========='

# create packaging directory
os.mkdir(pkg_dir)
print 'Created packaging dir: "{}".'.format(pkg_dir)

# create DEBIAN directory
deb_dir = os.path.join(pkg_dir, 'DEBIAN')
os.mkdir(deb_dir)
print 'Created DEBIAN dir: "{}".'.format(deb_dir)

# create install directory (utilizing sudo)
inst_dir = os.path.join(pkg_dir, 'usr', 'lib', 'python2.7', 'dist-packages')
try:
    command = 'mkdir -p {}'.format(inst_dir)
    subprocess.call(["/usr/bin/sudo", "sh", "-c", command])
except:
    raise
print 'Created install dir: "{}".'.format(inst_dir)

# copy local built pymrpt.so to install dir (utilizing sudo)
command = 'cp {} {}'.format(built_lib, inst_dir)
subprocess.call(["/usr/bin/sudo", "sh", "-c", command])
print 'Copied library file: "{}" to "{}".'.format(built_lib, inst_dir)

# change file permissions
inst_so = os.path.join(inst_dir, 'pymrpt.so')
command = 'chmod 0644 {}'.format(inst_so)
subprocess.call(["/usr/bin/sudo", "sh", "-c", command])
print 'Changed "{}" permissions to: 0644.'.format(inst_so)

# remove rpath from shared lib
command = 'chrpath -d {}'.format(inst_so)
subprocess.call(["/usr/bin/sudo", "sh", "-c", command])
print 'Removed RPATH from "{}".'.format(inst_so)


# create control file
control_content = [
    'Package: mrpt-python-bindings',
    'Version: {}'.format(built_lib_version),
    'Architecture: {}'.format(built_lib_arch),
    'Maintainer: Peter Rudolph <semael23@gmail.com>',
    'Installed-Size: {}'.format(built_lib_size),
    'Depends: libmrpt-base1.3, libmrpt-slam1.3, libmrpt-obs1.3, libmrpt-maps1.3, libmrpt-nav1.3, libmrpt-opengl1.3, libmrpt-gui1.3, libboost-python1.54, libc6',
    'Section: devel',
    'Priority: optional',
    'Homepage: http://www.mrpt.org',
    'Description: MRPT python bindings.',
    ' This package contains the python bindings for',
    ' the Mobile Robot Programming Toolkit (MRPT).',
    '' # final new line
]
control_filename = os.path.join(pkg_dir, 'DEBIAN', 'control')
control_file = open(control_filename, 'w+')
control_file.write('\n'.join(control_content))
control_file.close()
print 'Created control file: "{}".'.format(control_filename)


# create copyright file
copyright_content = [
    'Format: http://www.debian.org/doc/packaging-manuals/copyright-format/1.0/',
    'Upstream-Name: mrpt-python-bindings',
    'Upstream-Contact: Peter Rudolph <semael23@gmail.com>',
    'Source: https://github.com/MRPT/mrpt',
    '',
    'Files: *',
    'Copyright: 2014-2015 Peter Rudolph',
    'License: Expat',
    '',
    'License: Expat',
    ' Permission is hereby granted, free of charge, to any person obtaining a copy',
    ' of this software and associated documentation files (the "Software"), to deal',
    ' in the Software without restriction, including without limitation the rights',
    ' to use, copy, modify, merge, publish, distribute, sublicense, and/or sell',
    ' copies of the Software, and to permit persons to whom the Software is',
    ' furnished to do so, subject to the following conditions:',
    ' ',
    ' The above copyright notice and this permission notice shall be included in',
    ' all copies or substantial portions of the Software.',
    ' ',
    ' THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR',
    ' IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,',
    ' FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE',
    ' AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER',
    ' LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,',
    ' OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN',
    ' THE SOFTWARE.',
    '',
]
copyright_filename = os.path.join(pkg_dir, 'DEBIAN', 'copyright')
copyright_file = open(copyright_filename, 'w+')
copyright_file.write('\n'.join(copyright_content))
copyright_file.close()
print 'Created copyright file: "{}".'.format(copyright_filename)

# TODO use auto-generated changelog
changelog_content = [
    'python-mrpt-bindings ({}) unstable; urgency=low'.format(built_lib_version),
    '',
    '  * Initial Release.',
    '',
    ' -- Peter Rudolph <semael23@gmail.com>  Tue, 20 Jan 2015 12:30:54 +0100',
    '',
]
changelog_filename = os.path.join(pkg_dir, 'DEBIAN', 'changelog')
changelog_file = open(changelog_filename, 'w+')
changelog_file.write('\n'.join(changelog_content))
changelog_file.close()
print 'Created changelog file: "{}".'.format(changelog_filename)


os.chdir(home_dir)

# build the package
print 'Build package:'
subprocess.call(['dpkg', '-b', 'mrpt-python-bindings'])

# check package with lintian
print 'Check package:'
subprocess.call(['lintian', os.path.join(home_dir, 'mrpt-python-bindings.deb')])

# go to initial dir
os.chdir(curr_dir)

print 'Done.'
