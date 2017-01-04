/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/utils/CConfigFileBase.h>

namespace mrpt
{
namespace utils
{
	/** A wrapper for other CConfigFileBase-based objects that prefixes a given token to every key and/or section. 
	  *  If, for example, your code expect: 
	  *   \code
	  *     [params1]
	  *     foo = 34.0
	  *     bar = /dev/ttyUSB0
	  *   \endcode
	  *
	  *  Using this class with key entries prefix "s1_" will enable the same existing code to transparently parse this file content:
	  * 
	  *   \code
	  *     [params1]
	  *     s1_foo = 34.0
	  *     s1_bar = /dev/ttyUSB0
	  *   \endcode
	  *
	  * \sa CConfigFileBase
	  * \ingroup mrpt_base_grp
	  */
	class BASE_IMPEXP CConfigFilePrefixer : public CConfigFileBase
	{
	private:
		CConfigFileBase *m_bound_object; //!< The object we are wrapping
		std::string      m_prefix_sections, m_prefix_keys;

	protected:
		void  writeString(const std::string &section,const std::string &name, const std::string &str) MRPT_OVERRIDE;
		std::string  readString(const std::string &section,const std::string &name,const std::string &defaultStr,bool failIfNotFound = false) const MRPT_OVERRIDE;

	public:
		/** Unbound constructor: must bind this object to CConfigFileBase before usage with \a bind() and \a setPrefixes() */
		CConfigFilePrefixer();
		/** Construct and bind to (wrap) a given object with given prefix texts */
		CConfigFilePrefixer(const CConfigFileBase &o, const std::string &prefix_sections, const std::string &prefix_keys);

		/** Make this object to wrap the given existing CConfigFileBase object. Can be changed at any moment after construction */
		void bind(const CConfigFileBase &o);

		/** Change the prefix for sections and keys. Can be called at any moment. */
		void setPrefixes(const std::string &prefix_sections, const std::string &prefix_keys);

		std::string getSectionPrefix() const;
		std::string getKeyPrefix() const;
		CConfigFileBase *getBoundConfigFileBase() const;  //!< Returns the currently-bounded config source, or NULL if none.

		virtual ~CConfigFilePrefixer();

		void getAllSections( vector_string	&sections ) const MRPT_OVERRIDE; // See base class docs
		void getAllKeys( const std::string &section, vector_string	&keys ) const MRPT_OVERRIDE; // See base class docs

	}; // End of class def.
} // End of namespace
} // end of namespace
