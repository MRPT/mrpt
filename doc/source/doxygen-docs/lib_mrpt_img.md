\defgroup mrpt_img_grp [mrpt-img]

Basic computer vision data structures and tools: bitmap images, canvas, color
maps, pinhole camera models, and image undistortion.

[TOC]

# Library mrpt-img

This C++ library is part of MRPT and can be installed in Debian-based systems
with:

		sudo apt install libmrpt-img-dev

Read also [how to import MRPT into your CMake scripts](mrpt_from_cmake.html).

## Image handling

The class mrpt::img::CImage is a portable image container supporting
grayscale, RGB, and RGBA images in 8-bit depth. It provides loading/saving
(via STB), pixel access, drawing primitives, resizing, and color conversion.

In MRPT 3.0, `CImage` no longer depends on OpenCV. All image I/O and
processing is handled natively (STB for codecs, custom kernels for
filtering and resampling).

## Camera models and distortion

mrpt::img::TCamera stores pinhole intrinsics and lens distortion coefficients
for three models (see mrpt::img::DistortionModel):
- `DistortionModel::none` — ideal pinhole
- `DistortionModel::plumb_bob` — radial + tangential (Brown-Conrady, 5 params)
- `DistortionModel::kannala_brandt` — fisheye / equidistant (4 params)

Distortion/undistortion math is in mrpt::img::camera_geometry (header:
`<mrpt/img/camera_geometry.h>`).

## Image undistortion

- mrpt::img::CUndistortMap — precomputes a remap table for a single camera,
  then applies it efficiently to any number of frames. Much faster than
  calling `CImage::undistort()` repeatedly with the same camera params.
- mrpt::img::CStereoRectifyMap — Bouguet-style stereo rectification for a
  calibrated stereo pair. Produces remap tables for both cameras so that
  epipolar lines become horizontal.
- `CImage::undistort()` — convenience one-shot undistortion (internally
  creates a `CUndistortMap`).

All three support every `DistortionModel` defined in `TCamera`.

## Image pyramids

mrpt::img::CImagePyramid builds a multi-scale (octave) image pyramid by
repeated half-size decimation, with optional Gaussian smoothing.
`buildPyramidFast()` moves the source image into the pyramid to avoid a copy.

## Eigen interop

`<mrpt/img/CImage_Eigen.h>` provides zero-copy `Eigen::Map` views over
grayscale `CImage` data:

```cpp
#include <mrpt/img/CImage_Eigen.h>
mrpt::img::CImage img(640, 480, mrpt::img::CH_GRAY);
auto map = mrpt::img::asEigenMap(img);  // Eigen::Map with stride
map.setZero();  // works directly on image memory
```

## SIMD (SSE/AVX) optimizations

MRPT supports optional SIMD-optimized code paths for image operations on
Intel/AMD CPUs. The system uses **compile-time feature detection** via
`#if MRPT_ARCH_INTEL_COMPATIBLE` and separate translation units per
instruction set:

- **`CImage.SSE2.cpp`**: SSE2 optimizations (scale-half for 1-channel 8-bit,
  smooth scale-half).
- **`CImage.SSSE3.cpp`**: SSSE3 optimizations (scale-half for 3-channel RGB,
  RGB/BGR to grayscale).
- **`CImage.SSEx.h`**: Shared declarations for all SIMD-optimized functions
  (private header in `src/`).

Current SIMD-optimized functions:
1. `image_SSE2_scale_half_1c8u()` — Grayscale 1:2 decimation
2. `image_SSE2_scale_half_smooth_1c8u()` — Grayscale smooth (2x2 average) decimation
3. `image_SSSE3_scale_half_3c8u()` — RGB 1:2 decimation
4. `image_SSSE3_rgb_to_gray_8u()` — RGB to grayscale conversion
5. `image_SSSE3_bgr_to_gray_8u()` — BGR to grayscale conversion

Pattern for adding new SIMD kernels:
- Create a new `.cpp` file named `CImage.<ISA>.cpp` (e.g., `CImage.AVX2.cpp`).
- Guard the entire file with `#if MRPT_ARCH_INTEL_COMPATIBLE` and the
  appropriate `#include <immintrin.h>`.
- Add the function declaration to `CImage.SSEx.h`.
- Call the SIMD function from the main `CImage.cpp` with a runtime fallback
  to scalar code.
- Functions return `bool` indicating whether the SIMD path was taken (used
  by `CImagePyramid::buildPyramid()`).

The bilinear remap in `remap_bilinear.h` (used by `CUndistortMap` and
`CStereoRectifyMap`) is a candidate for future SSE2/AVX2 optimization.

# Library contents
