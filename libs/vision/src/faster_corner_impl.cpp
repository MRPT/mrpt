/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
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
	const cv::Mat& I, TKeyPointList& corners, int barrier, uint8_t octave,
	std::vector<size_t>* out_feats_index_by_row)
{
	auto ptr = I.data;
	CVD::BasicImage<CVD::byte> img(ptr, {I.cols, I.rows}, I.step[0]);

	std::vector<CVD::ImageRef> outputs;
	// reerve enough corners for every pixel
	outputs.reserve(I.cols * I.rows);
	F(img, outputs, barrier);
	corners.reserve(corners.size() + outputs.size());
	for (auto& output : outputs)
		corners.emplace_back(output.x << octave, output.y << octave);
	if (out_feats_index_by_row)
	{
		auto& counters = *out_feats_index_by_row;
		counters.assign(I.rows, 0);
		for (auto& output : outputs) counters[output.y]++;
	}
}

void fast_corner_detect_9(
	const cv::Mat& I, TKeyPointList& corners, int barrier, uint8_t octave,
	std::vector<size_t>* out_feats_index_by_row)
{
	fast_corner_detect<CVD::fast_corner_detect_9>(
		I, corners, barrier, octave, out_feats_index_by_row);
}

void fast_corner_detect_10(
	const cv::Mat& I, TKeyPointList& corners, int barrier, uint8_t octave,
	std::vector<size_t>* out_feats_index_by_row)
{
	fast_corner_detect<CVD::fast_corner_detect_10>(
		I, corners, barrier, octave, out_feats_index_by_row);
}

void fast_corner_detect_12(
	const cv::Mat& I, TKeyPointList& corners, int barrier, uint8_t octave,
	std::vector<size_t>* out_feats_index_by_row)
{
	fast_corner_detect<CVD::fast_corner_detect_12>(
		I, corners, barrier, octave, out_feats_index_by_row);
}
#endif
