\defgroup mrpt_img_grp [mrpt-img]

Basic computer vision data structures and tools: bitmap images, canvas, color
maps, and pinhole camera models.

[TOC]

# Library mrpt-img

This C++ library is part of MRPT and can be installed in Debian-based systems
with:

		sudo apt install libmrpt-img-dev

Read also [how to import MRPT into your CMake scripts](mrpt_from_cmake.html).


Find below some examples of use.

## Image handling

The class mrpt::img::CImage is a custom, portable image container, including
basic functionality like image loading, saving, and manipulation.

In MRPT 3.0.0 it was ported away from OpenCV so we do not depend 
on opencv at all for building or running.

# Library contents
