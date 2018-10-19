/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "vision-precomp.h"  // Precompiled headers

#if MRPT_HAS_OPENCV

#include "faster_corner_prototypes.h"

#include <cvd/fast_corner.h>
#include <cvd/utility.h>

template <void (*F)(
	const CVD::BasicImage<CVD::byte>& I, std::vector<CVD::ImageRef>& corners,
	int barrier)>
void fast_corner_detect(
	const IplImage* I, TSimpleFeatureList& corners, int barrier, uint8_t octave,
	std::vector<size_t>* out_feats_index_by_row)
{
	auto ptr = reinterpret_cast<CVD::byte*>(I->imageData);
	CVD::BasicImage<CVD::byte> img(ptr, {I->width, I->height}, I->widthStep);

	std::vector<CVD::ImageRef> outputs;
	// reerve enough corners for every pixel
	outputs.reserve(I->width * I->height);
	F(img, outputs, barrier);
	for (auto& output : outputs)
	{
		corners.push_back_fast(output.x << octave, output.y << octave);
	}
	if (out_feats_index_by_row)
	{
		auto& counters = *out_feats_index_by_row;
		counters.assign(I->height, 0);
		for (auto& output : outputs)
		{
			counters[output.y]++;
		}
	}
}

void fast_corner_detect_9(
	const IplImage* I, TSimpleFeatureList& corners, int barrier, uint8_t octave,
	std::vector<size_t>* out_feats_index_by_row)
{
	fast_corner_detect<CVD::fast_corner_detect_9>(
		I, corners, barrier, octave, out_feats_index_by_row);
}

void fast_corner_detect_10(
	const IplImage* I, TSimpleFeatureList& corners, int barrier, uint8_t octave,
	std::vector<size_t>* out_feats_index_by_row)
{
	fast_corner_detect<CVD::fast_corner_detect_10>(
		I, corners, barrier, octave, out_feats_index_by_row);
}

void fast_corner_detect_12(
	const IplImage* I, TSimpleFeatureList& corners, int barrier, uint8_t octave,
	std::vector<size_t>* out_feats_index_by_row)
{
	fast_corner_detect<CVD::fast_corner_detect_12>(
		I, corners, barrier, octave, out_feats_index_by_row);
}
#endif
