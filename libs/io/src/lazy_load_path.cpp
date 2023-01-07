/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "io-precomp.h"	 // Precompiled headers
//
#include <mrpt/core/exceptions.h>
#include <mrpt/io/lazy_load_path.h>

static std::string LAZY_LOAD_PATH_BASE(".");

std::string mrpt::io::lazy_load_absolute_path(
	const std::string& relativeOrAbsolutePath)
{
	const auto& fil = relativeOrAbsolutePath;

	ASSERT_(fil.size() > 2);

	if (fil[0] == '/' || (fil[1] == ':' && (fil[2] == '\\' || fil[2] == '/')))
	{  // already absolute:
		return fil;
	}

	// It is relative:
	auto out_path = LAZY_LOAD_PATH_BASE;
	ASSERT_(!LAZY_LOAD_PATH_BASE.empty());

	const auto lastChar = *LAZY_LOAD_PATH_BASE.rbegin();

	if (lastChar != '/' && lastChar != '\\') out_path += "/";

	out_path += fil;
	return out_path;
}

const std::string& mrpt::io::getLazyLoadPathBase()
{
	return LAZY_LOAD_PATH_BASE;
}

void mrpt::io::setLazyLoadPathBase(const std::string& path)
{
	LAZY_LOAD_PATH_BASE = path;
}
