/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "serialization-precomp.h"	// Precompiled headers
//
#include <mrpt/core/exceptions.h>
#include <mrpt/serialization/CSchemeArchiveBase.h>
#include <mrpt/serialization/CSerializable.h>

using namespace mrpt::serialization;
void CSchemeArchiveBase_impl::ReadObject(
	CSchemeArchiveBase& out, const CSerializable& obj)
{
	CSchemeArchiveBase::ReadObject(out, obj);
}
void CSchemeArchiveBase_impl::WriteObject(
	CSchemeArchiveBase& in, CSerializable& obj)
{
	CSchemeArchiveBase::WriteObject(in, obj);
}
