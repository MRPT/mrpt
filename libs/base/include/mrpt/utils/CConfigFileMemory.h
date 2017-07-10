/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#ifndef CConfigFileMemory_H
#define CConfigFileMemory_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/utils/CStringList.h>
#include <mrpt/utils/safe_pointers.h>

namespace mrpt
{
namespace utils
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
class BASE_IMPEXP CConfigFileMemory : public CConfigFileBase
{
   public:
	/** Empty constructor. Upon construction, call any of the "setContent"
	 * method */
	CConfigFileMemory();
	/** Constructor and initialize from a list of strings */
	CConfigFileMemory(const utils::CStringList& stringList);
	/** Constructor and initialize from string with the whole "config file" */
	CConfigFileMemory(const std::string& str);
	/** dtor */
	virtual ~CConfigFileMemory();

	/** Copy constructor */
	CConfigFileMemory(const CConfigFileMemory& o);
	/** Copy operator */
	CConfigFileMemory& operator=(const CConfigFileMemory& o);

	/** Changes the contents of the virtual "config file" */
	void setContent(const utils::CStringList& stringList);
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
	void getAllSections(vector_string& sections) const override;
	/** Returs a list with all the keys into a section */
	void getAllKeys(
		const std::string& section, vector_string& keys) const override;

   private:
	/** The IniFile object */
	void_ptr_noncopy m_ini;

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

}  // End of namespace
}  // end of namespace
#endif
