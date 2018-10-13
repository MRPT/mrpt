/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/serialization/CSerializable.h>

namespace mrpt::hmtslam
{
/** An arbitrary list of "annotations", or named attributes, each being an
 * instance of any CSerializable object.
 *  A multi-hypotheses version exists in CMHPropertiesValuesList.
 * \sa CSerializable, CMHPropertiesValuesList, mrpt::system::TParameters
 * \ingroup mrpt_base_grp
 */
class CPropertiesValuesList : public mrpt::serialization::CSerializable
{
	DEFINE_SERIALIZABLE(CPropertiesValuesList)
   protected:
	struct TPropertyValuePair
	{
		std::string name;
		CSerializable::Ptr value;
	};
	/** The properties list: a map between strings and objects
	 */
	std::vector<TPropertyValuePair> m_properties;

   public:
	/** Default constructor
	 */
	CPropertiesValuesList();

	/** Copy constructor
	 */
	CPropertiesValuesList(const CPropertiesValuesList& o);

	/** Copy operator
	 */
	CPropertiesValuesList& operator=(const CPropertiesValuesList& o);

	/** Destructor
	 */
	~CPropertiesValuesList() override;

	/** Clears the list.
	 */
	void clear();

	/** Returns the value of the property (case insensitive), or nullptr if it
	 * does not exist.
	 */
	CSerializable::Ptr get(const std::string& propertyName) const;

	/** Sets/change the value of the property (case insensitive), making a copy
	 * of the object (or setting it to nullptr if it is the passed value)
	 */
	void set(const std::string& propertyName, const CSerializable::Ptr& obj);

	/** Returns the number of properties in the list
	 */
	size_t size() const;

	/** Returns the name of all properties in the list
	 */
	std::vector<std::string> getPropertyNames() const;

};  // End of class def.
}  // namespace mrpt::hmtslam
