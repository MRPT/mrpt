/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

/** \file While refactoring mrpt-base into smaller sublibraries, CMemoryChunk
 was found to be problematic: it inherited from very low-level classes in both
 mrpt-io and mrpt-serialization. To keep those two libraries independent of
 each other, I propose to IMPLEMENT its RTTI stuff here, in mrpt-opengl, as a
 dirty workaround, while it is DECLARED in mrpt-serialization using CRTP.
*/

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/io/CMemoryStream.h>
#include <mrpt/serialization/CMemoryChunk.h>

// IMPLEMENTS_SERIALIZABLE(CMemoryChunk, CSerializable, mrpt::serialization)
template <class Derived>
mrpt::rtti::CObject*
	mrpt::serialization::CMemoryChunkBase<Derived>::CreateObject()
{
	return static_cast<mrpt::rtti::CObject*>(
		new mrpt::serialization::CMemoryChunkBase<Derived>);
}
template <class Derived>
const mrpt::rtti::TRuntimeClassId*
	mrpt::serialization::CMemoryChunkBase<Derived>::_GetBaseClass()
{
	return CLASS_ID_NAMESPACE(CSerializable, mrpt::serialization);
}
template <class Derived>
const mrpt::rtti::TRuntimeClassId&
	mrpt::serialization::CMemoryChunkBase<Derived>::GetRuntimeClassIdStatic()
{
	return mrpt::serialization::CMemoryChunkBase<Derived>::runtimeClassId;
}
template <class Derived>
const mrpt::rtti::TRuntimeClassId
	mrpt::serialization::CMemoryChunkBase<Derived>::runtimeClassId = {
		"CMemoryChunk",
		&mrpt::serialization::CMemoryChunkBase<Derived>::CreateObject,
		&CMemoryChunk::_GetBaseClass};
template <class Derived>
const mrpt::rtti::TRuntimeClassId*
	mrpt::serialization::CMemoryChunkBase<Derived>::GetRuntimeClass() const
{
	return CLASS_ID_NAMESPACE(CMemoryChunk, mrpt::serialization);
}
template <class Derived>
mrpt::rtti::CLASSINIT
	mrpt::serialization::CMemoryChunkBase<Derived>::_init_CMemoryChunk(
		CLASS_ID_NAMESPACE(CMemoryChunk, mrpt::serialization));
template <class Derived>
mrpt::rtti::CObject* mrpt::serialization::CMemoryChunkBase<Derived>::clone()
	const
{
	return static_cast<mrpt::rtti::CObject*>(
		new mrpt::serialization::CMemoryChunkBase<Derived>(*this));
}

// Explicit instantiation
template class mrpt::serialization::CMemoryChunkBase<mrpt::io::CMemoryStream>;
