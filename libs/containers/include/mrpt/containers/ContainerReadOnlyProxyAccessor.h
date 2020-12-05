/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/exceptions.h>
#include <cstdlib>  // size_t

namespace mrpt::containers
{
/** \addtogroup mrpt_containers_stlext_grp STL containers extensions
 *  \ingroup mrpt_containers_grp
 * @{ */

/** A generic proxy accessor template that only allows read-only access to the
 * original binded STL container object. */
template <typename STLCONTAINER>
struct ContainerReadOnlyProxyAccessor
{
	/** ctor: binds to source object */
	ContainerReadOnlyProxyAccessor(STLCONTAINER& source) : m_source(source) {}
	/** (Deleted ctor) ignore copies (keep reference to original object) */
	ContainerReadOnlyProxyAccessor(
		const ContainerReadOnlyProxyAccessor<STLCONTAINER>&) = delete;
	/** Don't copy the reference to the source object, but copy the contained
	 * data. This is only allowed if the size of the buffers coincide. */
	ContainerReadOnlyProxyAccessor<STLCONTAINER>& operator=(
		const ContainerReadOnlyProxyAccessor<STLCONTAINER>& o)
	{
		ASSERT_EQUAL_(m_source.size(), o.m_source.size());
		m_source = o.m_source;
		return *this;
	}
	/** Transparent conversion to const ref to original source object. */
	operator const STLCONTAINER&() const { return m_source; }
	size_t size() const { return m_source.size(); }
	const typename STLCONTAINER::value_type& operator[](const int i) const
	{
		return m_source[i];
	}

	typename STLCONTAINER::const_iterator begin() const
	{
		return m_source.begin();
	}
	typename STLCONTAINER::const_iterator end() const { return m_source.end(); }
	typename STLCONTAINER::const_reverse_iterator rbegin() const
	{
		return m_source.rbegin();
	}
	typename STLCONTAINER::const_reverse_iterator rend() const
	{
		return m_source.rend();
	}

   private:
	STLCONTAINER& m_source;
};

/** @} */  // end of grouping
}  // namespace mrpt::containers
