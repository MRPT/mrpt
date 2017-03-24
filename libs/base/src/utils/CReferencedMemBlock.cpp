/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/utils/CReferencedMemBlock.h>

using namespace mrpt::utils;

CReferencedMemBlock::CReferencedMemBlock(size_t mem_block_size) :
	m_data( new std::vector<char>(mem_block_size) )
{
}

CReferencedMemBlock::~CReferencedMemBlock()
{
}

void CReferencedMemBlock::resize(size_t mem_block_size)
{
	m_data->resize(mem_block_size);
}

unsigned int CReferencedMemBlock::alias_count() const 
{
	return m_data.alias_count(); 
}

void CReferencedMemBlock::clear() 
{
	m_data.clear(); 
}
