/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/core/pimpl.h>

namespace mrpt::config
{
/** This class allows loading and storing values and vectors of different types
 * from ".ini" files easily.
 *  The contents of the file will be modified by "write" operations in memory,
 * and will be saved back
 *   to the file at the destructor, and only if at least one write operation
 * has been applied.
 *
 * Use base class `CConfigFileBase`'s methods
 * `read_{int,float,double,string,...}()` and `write()` to actually read and
 * write values.
 *
 * See: \ref config_file_format
 *
 * \ingroup mrpt_config_grp
 */
class CConfigFile : public CConfigFileBase
{
   private:
	/** The name of the file */
	std::string m_file;
	/** The interface to the file: */
	struct Impl;
	mrpt::pimpl<Impl> m_impl;
	/** If modified since load. */
	bool m_modified;

   protected:
	/** A virtual method to write a generic string  */
	void writeString(
		const std::string& section, const std::string& name,
		const std::string& str) override;

	/** A virtual method to read a generic string.
	 * \exception std::exception If the key name is not found and
	 * "failIfNotFound" is true. Otherwise the "defaultValue" is returned. */
	std::string readString(
		const std::string& section, const std::string& name,
		const std::string& defaultStr,
		bool failIfNotFound = false) const override;

   public:
	/** Constructor associating with a given configuration filename.
	 * If the file exists, it loads and parses its contents; otherwise, it
	 * silently just start with an empty configuration file in memory. The file
	 * will be written upon destruction, or at any time using writeNow()
	 */
	CConfigFile(const std::string& fileName);

	/** Constructor, does not open any file. You should call "setFileName"
	 * before reading or writting or otherwise nothing will be read and write
	 * operations will be eventually lost.
	 * However, it's perfectly right to use this object without an associated
	 * file, in which case it will behave as an "in-memory" file.
	 */
	CConfigFile();

	/** Associate this object with the given file, reading its contents right
	 * now. Upon destruction, the updated contents will be written to that file.
	 */
	void setFileName(const std::string& fil_path);

	/** Dumps the changes to the physical configuration file now, not waiting
	 * until destruction.
	 * \exception std::runtime_error Upon error writing.
	 */
	void writeNow();

	/** Discard saving (current) changes to physical file upon destruction */
	void discardSavingChanges();

	/** Returns the file currently open by this object. */
	std::string getAssociatedFile() const { return m_file; }
	/** Destructor */
	~CConfigFile() override;

	/** Returns a list with all the section names. */
	void getAllSections(std::vector<std::string>& sections) const override;

	/** Empties the "config file" */
	void clear() override;

	/** Returs a list with all the keys into a section. */
	void getAllKeys(const std::string& section, std::vector<std::string>& keys)
		const override;

};	// End of class def.

}  // namespace mrpt::config
