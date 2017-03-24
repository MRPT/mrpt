/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "maps-precomp.h" // Precomp header

#include <mrpt/maps/CPointCloudFilterBase.h>

using namespace mrpt::maps;

CPointCloudFilterBase::CPointCloudFilterBase()
{
}

CPointCloudFilterBase::~CPointCloudFilterBase()
{
}

mrpt::maps::CPointCloudFilterBase::TExtraFilterParams::TExtraFilterParams() :
	out_deletion_mask(nullptr),
	do_not_delete(false)
{
}
