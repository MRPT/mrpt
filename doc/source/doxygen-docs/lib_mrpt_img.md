\defgroup mrpt_img_grp [mrpt-img]

Basic computer vision data structures and tools: bitmap images, canvas, color
maps, and pinhole camera models.

[TOC]

# Library `mrpt-img`
<small> [New in MRPT 2.0.0] </small>

This C++ library is part of MRPT and can be installed in Debian-based systems
with:

		sudo apt install libmrpt-img-dev

See also: [Import MRPT into your CMake scripts](mrpt_from_cmake.html).


Find below some examples of use.

## Image handling

The class mrpt::img::CImage represents a wrapper around OpenCV images, plus
extra functionality such as on-the-fly loading of images stored in disk upon
first usage. The `cv::Mat` object is always available so
OpenCV's functions can be still used to operate on MRPT images.


