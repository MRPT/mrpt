/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/config/CConfigFileBase.h>

namespace mrpt::config
{
/** A wrapper for other CConfigFileBase-based objects that prefixes a given
 * token to every key and/or section.
 *  If, for example, your code expect:
 *   \code
 *     [params1]
 *     foo = 34.0
 *     bar = /dev/ttyUSB0
 *   \endcode
 *
 *  Using this class with key entries prefix "s1_" will enable the same
 * existing code to transparently parse this file content:
 *
 *   \code
 *     [params1]
 *     s1_foo = 34.0
 *     s1_bar = /dev/ttyUSB0
 *   \endcode
 *
 * See: \ref config_file_format
 * \sa CConfigFileBase
 * \ingroup mrpt_config_grp
 */
class CConfigFilePrefixer : public CConfigFileBase
{
   private:
	/** The object we are wrapping */
	CConfigFileBase* m_bound_object{nullptr};
	std::string m_prefix_sections, m_prefix_keys;

   protected:
	void writeString(
		const std::string& section, const std::string& name,
		const std::string& str) override;
	std::string readString(
		const std::string& section, const std::string& name,
		const std::string& defaultStr,
		bool failIfNotFound = false) const override;

   public:
	/** Unbound constructor: must bind this object to CConfigFileBase before
	 * usage with \a bind() and \a setPrefixes() */
	CConfigFilePrefixer();
	/** Construct and bind to (wrap) a given object with given prefix texts */
	CConfigFilePrefixer(
		const CConfigFileBase& o, const std::string& prefix_sections,
		const std::string& prefix_keys);

	/** Make this object to wrap the given existing CConfigFileBase object. Can
	 * be changed at any moment after construction */
	void bind(const CConfigFileBase& o);

	/** Change the prefix for sections and keys. Can be called at any moment. */
	void setPrefixes(
		const std::string& prefix_sections, const std::string& prefix_keys);

	std::string getSectionPrefix() const;
	std::string getKeyPrefix() const;
	/** Returns the currently-bounded config source, or nullptr if none. */
	CConfigFileBase* getBoundConfigFileBase() const;

	~CConfigFilePrefixer() override;

	void getAllSections(std::vector<std::string>& sections)
		const override;  // See base class docs
	void getAllKeys(const std::string& section, std::vector<std::string>& keys)
		const override;  // See base class docs

};  // End of class def.
}  // namespace mrpt::config
