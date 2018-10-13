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
#include <mrpt/core/pimpl.h>
#include <string>
#include <vector>

namespace mrpt::config
{
/** This class implements a config file-like interface over a memory-stored
 * string list.
 *
 * Use base class `CConfigFileBase`'s methods
 * `read_{int,float,double,string,...}()` and `write()` to actually read and
 * write values.
 *
 * See: \ref config_file_format
 * \ingroup mrpt_base_grp
 */
class CConfigFileMemory : public CConfigFileBase
{
   public:
	/** Empty constructor. Upon construction, call any of the "setContent"
	 * method */
	CConfigFileMemory();
	/** Constructor and initialize from a list of strings */
	CConfigFileMemory(const std::vector<std::string>& stringList);
	/** Constructor and initialize from string with the whole "config file" */
	CConfigFileMemory(const std::string& str);
	/** dtor */
	~CConfigFileMemory() override;

	/** Changes the contents of the virtual "config file" */
	void setContent(const std::vector<std::string>& stringList);
	/** Changes the contents of the virtual "config file" */
	void setContent(const std::string& str);
	/** Return the current contents of the virtual "config file" */
	void getContent(std::string& str) const;
	/** \overload */
	inline std::string getContent() const
	{
		std::string s;
		getContent(s);
		return s;
	}

	/** Returns a list with all the section names */
	void getAllSections(std::vector<std::string>& sections) const override;
	/** Returs a list with all the keys into a section */
	void getAllKeys(const std::string& section, std::vector<std::string>& keys)
		const override;

   private:
	/** The IniFile object */
	struct Impl;
	mrpt::pimpl<Impl> m_impl;

   protected:
	/** A virtual method to write a generic string */
	void writeString(
		const std::string& section, const std::string& name,
		const std::string& str) override;
	/** A virtual method to read a generic string */
	std::string readString(
		const std::string& section, const std::string& name,
		const std::string& defaultStr,
		bool failIfNotFound = false) const override;

};  // End of class def.

}  // namespace mrpt::config
