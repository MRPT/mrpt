/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  CMHPropertiesValuesList_H
#define  CMHPropertiesValuesList_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/CMemoryChunk.h>
#include <mrpt/system/string_utils.h>
#include <cstdio>

namespace mrpt
{
    namespace utils
    {
        // This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE( CMHPropertiesValuesList, mrpt::utils::CSerializable )

        /** Internal triplet for each property in utils::CMHPropertiesValuesList */
        struct BASE_IMPEXP  TPropertyValueIDTriplet
        {
            TPropertyValueIDTriplet() : name(), value(NULL),ID(0)
            {}

            std::string			name;
            CSerializablePtr	value;
            int64_t				ID;
        };

        /** An arbitrary list of "annotations", or named attributes, each being an instance of any CSerializable object (Multi-hypotheses version).
         *   For each named annotation (or attribute), several values may exist, each associated to a given hypothesis ID.
         * A non multi-hypotheses version exists in CPropertiesValuesList.
         * \sa CSerializable, CPropertiesValuesList
	 * \ingroup mrpt_base_grp
         */
        class BASE_IMPEXP CMHPropertiesValuesList : public mrpt::utils::CSerializable
        {
            // This must be added to any CSerializable derived class:
            DEFINE_SERIALIZABLE( CMHPropertiesValuesList )

		private:
			std::vector<TPropertyValueIDTriplet>	m_properties;

        public:
            /** Default constructor
              */
            CMHPropertiesValuesList();

            /** Copy constructor
              */
            CMHPropertiesValuesList( const CMHPropertiesValuesList& o );

            /** Copy operator
              */
            CMHPropertiesValuesList & operator =( const CMHPropertiesValuesList& o );

            /** Destructor
              */
            virtual ~CMHPropertiesValuesList();

            /** Clears the list and frees all object's memory.
              */
            void  clear();

            /** Returns the value of the property (case insensitive) for some given hypothesis ID, or a NULL smart pointer if it does not exist.
              */
            CSerializablePtr  get(const char *propertyName, const int64_t & hypothesis_ID ) const;

            /** Returns the value of the property (case insensitive) for some given hypothesis ID checking its class in runtime, or a NULL smart pointer if it does not exist.
              */
			template <typename T>
            typename T::SmartPtr getAs(const char *propertyName, const int64_t & hypothesis_ID, bool allowNullPointer = true) const
			{
				MRPT_START
				CSerializablePtr obj = get(propertyName,hypothesis_ID);
				if (!obj)
				{
					if (allowNullPointer)
							return typename T::SmartPtr();
					else	THROW_EXCEPTION("Null pointer")
				}
				const mrpt::utils::TRuntimeClassId*	class_ID = T::classinfo;
				ASSERT_( class_ID == obj->GetRuntimeClass() );
				return typename T::SmartPtr( obj );
				MRPT_END
			}


			/** Returns the value of the property (case insensitive) for the first hypothesis ID found, or NULL if it does not exist.
              */
            CSerializablePtr  getAnyHypothesis(const char *propertyName) const;

            /** Sets/change the value of the property (case insensitive) for the given hypothesis ID, making a copy of the object (or setting it to NULL if it is the passed value)
              * \sa setMemoryReference
              */
            void  set(const char *propertyName, const CSerializablePtr &obj, const int64_t & hypothesis_ID);

            /** Sets/change the value of the property (case insensitive) for the given hypothesis ID, directly replacing the pointer instead of making a copy of the object.
              * \sa set
              */
            void  setMemoryReference(const char *propertyName, const CSerializablePtr& obj, const int64_t & hypothesis_ID);

            /** Remove a given property, if it exists.
              */
            void  remove(const char *propertyName, const int64_t & hypothesis_ID);

            /** Remove all the properties for the given hypothesis.
              */
            void  removeAll(const int64_t & hypothesis_ID);

            /** Sets/change the value of a property (case insensitive) for the given hypothesis ID, from an elemental data type.
              */
            template <class T>
            void  setElemental(const char *propertyName, const T &data, const int64_t & hypothesis_ID)
            {
                MRPT_START

                CMemoryChunkPtr memChunk = CMemoryChunkPtr( new CMemoryChunk() );
				memChunk->setAllocBlockSize(10);
                (*memChunk) << data;

                for (std::vector<TPropertyValueIDTriplet>::iterator it=m_properties.begin();it!=m_properties.end();++it)
                {
                    if ( it->ID == hypothesis_ID && mrpt::system::strCmpI(propertyName,it->name) )
                    {
                        // Delete current contents &
                        // Copy new value:
                        it->value = memChunk;
                        return;
                    }
                }

                // Insert as new:
                TPropertyValueIDTriplet	newPair;
                newPair.name = std::string(propertyName);
                newPair.value = memChunk;
                newPair.ID    = hypothesis_ID;
                m_properties.push_back(newPair);

                MRPT_END_WITH_CLEAN_UP( \
                    printf("Exception while setting annotation '%s'",propertyName); \
                    );
            }

            /** Gets the value of a property (case insensitive) for the given hypothesis ID, retrieves it as an elemental data type (types must coincide, basic size check is performed).
              * \return false if the property does not exist for the given hypothesis.
              */
            template <class T>
            bool getElemental(const char *propertyName, T &out_data, const int64_t & hypothesis_ID, bool raiseExceptionIfNotFound = false) const
            {
                MRPT_START
                for (std::vector<TPropertyValueIDTriplet>::const_iterator it=m_properties.begin();it!=m_properties.end();++it)
                {
                    if (mrpt::system::strCmpI(propertyName,it->name) && it->ID == hypothesis_ID )
                    {
                        CMemoryChunkPtr memChunk = CMemoryChunkPtr(it->value);
                        ASSERT_(memChunk)
                        if (memChunk->getTotalBytesCount()!=sizeof(out_data)) THROW_EXCEPTION("Data sizes do not match.");
                        out_data = *static_cast<T*>( memChunk->getRawBufferData() );
                        return true;
                    }
                }
                // Not found:
                if (raiseExceptionIfNotFound)
                    THROW_EXCEPTION_CUSTOM_MSG1("Property '%s' not found", propertyName );
                return false;
                MRPT_END
            }

            /** Returns the name of all properties in the list
              */
            std::vector<std::string>  getPropertyNames() const;


			typedef std::vector<TPropertyValueIDTriplet>::iterator iterator;
			typedef std::vector<TPropertyValueIDTriplet>::const_iterator const_iterator;

			iterator begin() { return m_properties.begin(); }
			const_iterator begin() const { return m_properties.begin(); }
			iterator end() { return m_properties.end(); }
			const_iterator end() const { return m_properties.end(); }

			size_t size() const { return m_properties.size(); }

        }; // End of class def.
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE( CMHPropertiesValuesList, mrpt::utils::CSerializable )


	} // End of namespace
} // end of namespace
#endif
