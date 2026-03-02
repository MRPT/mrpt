"""
mrpt.vision — Image processing and computer vision utilities.

Provides:
  - TKeyPoint      : Integer-coordinate image keypoint
  - TKeyPointf     : Float-coordinate image keypoint
  - CUndistortMap  : Pre-computed lens undistortion map
  - CImagePyramid  : Gaussian image pyramid

Example::

    import mrpt.vision as vision
    import mrpt.img as img

    image = img.CImage()
    image.loadFromFile("image.jpg")

    pyramid = vision.CImagePyramid()
    pyramid.buildFromImage(image, nOctaves=4)
    for i in range(pyramid.octaveCount()):
        oct = pyramid.getOctave(i)
        print(f"Octave {i}: {oct.getWidth()}x{oct.getHeight()}")
"""

from mrpt.vision._bindings import (
    CImagePyramid,
    CUndistortMap,
    TKeyPoint,
    TKeyPointf,
)

__all__ = [
    "TKeyPoint",
    "TKeyPointf",
    "CUndistortMap",
    "CImagePyramid",
]
