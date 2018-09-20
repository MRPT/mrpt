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
#include <mrpt/system/string_utils.h>
#include <cstdio>
#include <sstream>

namespace mrpt::hmtslam
{
/** Internal triplet for each property in utils::CMHPropertiesValuesList */
struct TPropertyValueIDTriplet
{
	TPropertyValueIDTriplet() = default;
	std::string name, basic_value;
	mrpt::serialization::CSerializable::Ptr value;
	int64_t ID{0};
};

/** An arbitrary list of "annotations", or named attributes, each being an
 * instance of any CSerializable object (Multi-hypotheses version).
 *   For each named annotation (or attribute), several values may exist, each
 * associated to a given hypothesis ID.
 * A non multi-hypotheses version exists in CPropertiesValuesList.
 * \sa CSerializable, CPropertiesValuesList
 * \ingroup mrpt_base_grp
 */
class CMHPropertiesValuesList : public mrpt::serialization::CSerializable
{
	DEFINE_SERIALIZABLE(CMHPropertiesValuesList)

   private:
	std::vector<TPropertyValueIDTriplet> m_properties;

   public:
	/** Default constructor
	 */
	CMHPropertiesValuesList();

	/** Copy constructor
	 */
	CMHPropertiesValuesList(const CMHPropertiesValuesList& o);

	/** Copy operator
	 */
	CMHPropertiesValuesList& operator=(const CMHPropertiesValuesList& o);

	/** Destructor
	 */
	~CMHPropertiesValuesList() override;

	/** Clears the list and frees all object's memory.
	 */
	void clear();

	/** Returns the value of the property (case insensitive) for some given
	 * hypothesis ID, or a nullptr smart pointer if it does not exist.
	 */
	CSerializable::Ptr get(
		const char* propertyName, const int64_t& hypothesis_ID) const;

	/** Returns the value of the property (case insensitive) for some given
	 * hypothesis ID checking its class in runtime, or a nullptr smart pointer
	 * if it does not exist.
	 */
	template <typename T>
	typename T::Ptr getAs(
		const char* propertyName, const int64_t& hypothesis_ID,
		bool allowNullPointer = true) const
	{
		MRPT_START
		CSerializable::Ptr obj = get(propertyName, hypothesis_ID);
		if (!obj)
		{
			if (allowNullPointer)
				return typename T::Ptr();
			else
				THROW_EXCEPTION("Null pointer");
		}
		const mrpt::rtti::TRuntimeClassId* class_ID =
			&T::GetRuntimeClassIdStatic();
		ASSERT_(class_ID == obj->GetRuntimeClass());
		return std::dynamic_pointer_cast<T>(obj);
		MRPT_END
	}

	/** Returns the value of the property (case insensitive) for the first
	 * hypothesis ID found, or nullptr if it does not exist.
	 */
	CSerializable::Ptr getAnyHypothesis(const char* propertyName) const;

	/** Sets/change the value of the property (case insensitive) for the given
	 * hypothesis ID, making a copy of the object (or setting it to nullptr if
	 * it is the passed value)
	 * \sa setMemoryReference
	 */
	void set(
		const char* propertyName, const CSerializable::Ptr& obj,
		const int64_t& hypothesis_ID);

	/** Sets/change the value of the property (case insensitive) for the given
	 * hypothesis ID, directly replacing the pointer instead of making a copy of
	 * the object.
	 * \sa set
	 */
	void setMemoryReference(
		const char* propertyName, const CSerializable::Ptr& obj,
		const int64_t& hypothesis_ID);

	/** Remove a given property, if it exists.
	 */
	void remove(const char* propertyName, const int64_t& hypothesis_ID);

	/** Remove all the properties for the given hypothesis.
	 */
	void removeAll(const int64_t& hypothesis_ID);

	/** Sets/change the value of a property (case insensitive) for the given
	 * hypothesis ID, from an elemental data type.
	 */
	template <class T>
	void setElemental(
		const char* propertyName, const T& data, const int64_t& hypothesis_ID)
	{
		MRPT_START

		std::string basic_value;
		basic_value.resize(sizeof(T));
		::memcpy(&basic_value[0], &data, sizeof(T));

		for (auto& m_propertie : m_properties)
		{
			if (m_propertie.ID == hypothesis_ID &&
				mrpt::system::strCmpI(propertyName, m_propertie.name))
			{
				// Delete current contents &
				// Copy new value:
				m_propertie.basic_value = basic_value;
				return;
			}
		}

		// Insert as new:
		TPropertyValueIDTriplet newPair;
		newPair.name = std::string(propertyName);
		newPair.basic_value = basic_value;
		newPair.ID = hypothesis_ID;
		m_properties.push_back(newPair);

		MRPT_END_WITH_CLEAN_UP(
			printf("Exception while setting annotation '%s'", propertyName););
	}

	/** Gets the value of a property (case insensitive) for the given hypothesis
	 * ID, retrieves it as an elemental data type (types must coincide, basic
	 * size check is performed).
	 * \return false if the property does not exist for the given hypothesis.
	 */
	template <class T>
	bool getElemental(
		const char* propertyName, T& out_data, const int64_t& hypothesis_ID,
		bool raiseExceptionIfNotFound = false) const
	{
		MRPT_START
		for (const auto& m_propertie : m_properties)
		{
			if (mrpt::system::strCmpI(propertyName, m_propertie.name) &&
				m_propertie.ID == hypothesis_ID)
			{
				if (m_propertie.basic_value.size() != sizeof(out_data))
					THROW_EXCEPTION("Data sizes do not match.");
				out_data =
					*reinterpret_cast<const T*>(&m_propertie.basic_value[0]);
				return true;
			}
		}
		// Not found:
		if (raiseExceptionIfNotFound)
			THROW_EXCEPTION_FMT("Property '%s' not found", propertyName);
		return false;
		MRPT_END
	}

	/** Returns the name of all properties in the list
	 */
	std::vector<std::string> getPropertyNames() const;

	using iterator = std::vector<TPropertyValueIDTriplet>::iterator;
	using const_iterator = std::vector<TPropertyValueIDTriplet>::const_iterator;

	iterator begin() { return m_properties.begin(); }
	const_iterator begin() const { return m_properties.begin(); }
	iterator end() { return m_properties.end(); }
	const_iterator end() const { return m_properties.end(); }
	size_t size() const { return m_properties.size(); }
};  // End of class def.
}  // namespace mrpt::hmtslam
