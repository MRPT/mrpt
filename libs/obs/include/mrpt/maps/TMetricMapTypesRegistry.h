/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/utils/core_defs.h>
#include <mrpt/obs/link_pragmas.h>
#include <mrpt/obs/obs_frwds.h>
#include <map>
#include <string>

namespace mrpt
{
	namespace maps
	{
		struct TMetricMapInitializer;
		namespace internal
		{
			typedef mrpt::maps::TMetricMapInitializer* (*MapDefCtorFunctor)(void);
			typedef mrpt::maps::CMetricMap* (*MapCtorFromDefFunctor)(const mrpt::maps::TMetricMapInitializer&);

			/** Class factory & registry for map classes. Used from mrpt::maps::TMetricMapInitializer */
			struct OBS_IMPEXP TMetricMapTypesRegistry
			{
			public:
				static TMetricMapTypesRegistry & Instance();
				size_t doRegister(const std::string &name,MapDefCtorFunctor func1,MapCtorFromDefFunctor func2); //!< Return the index of the class in the list (not important, just used as a trick to initialize static members)
				mrpt::maps::TMetricMapInitializer* factoryMapDefinition(const std::string &className) const; //!< Return NULL if not found
				mrpt::maps::CMetricMap* factoryMapObjectFromDefinition(const mrpt::maps::TMetricMapInitializer&mi) const; //!< Return NULL if not found
				typedef std::map<std::string,std::pair<MapDefCtorFunctor,MapCtorFromDefFunctor> > TListRegisteredMaps;
				const TListRegisteredMaps & getAllRegistered() const { return m_registry;}
			private:
				TMetricMapTypesRegistry() {}  // Access thru singleton in Instance()
				TListRegisteredMaps  m_registry;
			};

			/** Add a MAP_DEFINITION_START() ... MAP_DEFINITION_END() block inside the declaration of each metric map */
			#define MAP_DEFINITION_START(_CLASS_NAME_,_LINKAGE_) \
				public: \
					/** @name Map Definition Interface stuff (see mrpt::maps::TMetricMapInitializer) @{ */ \
					struct _LINKAGE_ TMapDefinitionBase : public mrpt::maps::TMetricMapInitializer { \
						TMapDefinitionBase() : TMetricMapInitializer(CLASS_ID(_CLASS_NAME_)) { } \
					};\
					struct _LINKAGE_ TMapDefinition : public TMapDefinitionBase { \

			#define MAP_DEFINITION_END(_CLASS_NAME_,_LINKAGE_) \
						TMapDefinition();\
					protected: \
						void loadFromConfigFile_map_specific(const mrpt::utils::CConfigFileBase  &source, const std::string &sectionNamePrefix) MRPT_OVERRIDE; \
						void dumpToTextStream_map_specific(mrpt::utils::CStream	&out) const MRPT_OVERRIDE; \
					}; \
					/** Returns default map definition initializer. See mrpt::maps::TMetricMapInitializer */ \
					static mrpt::maps::TMetricMapInitializer* MapDefinition(); \
					/** Constructor from a map definition structure: initializes the map and its parameters accordingly */ \
					static _CLASS_NAME_* CreateFromMapDefinition(const mrpt::maps::TMetricMapInitializer &def); \
					static mrpt::maps::CMetricMap* internal_CreateFromMapDefinition(const mrpt::maps::TMetricMapInitializer &def); \
					/** ID used to initialize class registration (just ignore it) */ \
					static const size_t m_private_map_register_id; \
					/** @} */ 

			/** Registers one map class into TMetricMapInitializer factory. 
				* One or several alternative class names can be provided, separated with whitespaces or commas */
			#define MAP_DEFINITION_REGISTER(_CLASSNAME_STRINGS, _CLASSNAME_WITH_NS) \
				const size_t _CLASSNAME_WITH_NS::m_private_map_register_id = mrpt::maps::internal::TMetricMapTypesRegistry::Instance().doRegister(_CLASSNAME_STRINGS,&_CLASSNAME_WITH_NS::MapDefinition,&_CLASSNAME_WITH_NS::internal_CreateFromMapDefinition); \
				mrpt::maps::TMetricMapInitializer* _CLASSNAME_WITH_NS::MapDefinition() { return new _CLASSNAME_WITH_NS::TMapDefinition; } \
				_CLASSNAME_WITH_NS* _CLASSNAME_WITH_NS::CreateFromMapDefinition(const mrpt::maps::TMetricMapInitializer &def) \
					{ return dynamic_cast<_CLASSNAME_WITH_NS*>(_CLASSNAME_WITH_NS::internal_CreateFromMapDefinition(def)); }

		} // end NS internal
	} // End of namespace
} // End of namespace

