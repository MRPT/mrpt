/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <cstdlib> // size_t
#include <mrpt/utils/mrpt_macros.h>

namespace mrpt
{
	namespace utils
	{
		/** \addtogroup stlext_grp 
		  * @{ */

		/** A generic proxy accessor template that only allows read-only access to the original binded STL container object. */
		template <typename STLCONTAINER>
		struct ContainerReadOnlyProxyAccessor
		{
			ContainerReadOnlyProxyAccessor(STLCONTAINER &source) : m_source(source) { } //!< ctor: binds to source object
			ContainerReadOnlyProxyAccessor(const ContainerReadOnlyProxyAccessor<STLCONTAINER> &) MRPT_DELETED_FUNC; //!< (Deleted ctor) ignore copies (keep reference to original object)
			/** Don't copy the reference to the source object, but copy the contained data. This is only allowed if the size of the buffers coincide. */
			ContainerReadOnlyProxyAccessor<STLCONTAINER> & operator =(const ContainerReadOnlyProxyAccessor<STLCONTAINER> & o)
			{
				ASSERT_EQUAL_(m_source.size(), o.m_source.size());
				m_source = o.m_source;
				return *this; 
			}
			/** Transparent conversion to const ref to original source object. */
			operator const STLCONTAINER & () const { return m_source; }

			size_t size() const { return m_source.size(); }
			const typename STLCONTAINER::value_type & operator [](const int i) const { return m_source[i]; }

			typename STLCONTAINER::const_iterator begin() const { return m_source.begin(); }
			typename STLCONTAINER::const_iterator end() const { return m_source.end(); }
			typename STLCONTAINER::const_reverse_iterator rbegin() const { return m_source.rbegin(); }
			typename STLCONTAINER::const_reverse_iterator rend() const { return m_source.rend(); }
	
		private:
			STLCONTAINER & m_source;
		};

		/** @} */  // end of grouping
	}
}

