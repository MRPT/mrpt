/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/obs/obs_frwds.h>

#include <functional>
#include <map>
#include <memory>
#include <string>

namespace mrpt
{
namespace maps
{
struct TMetricMapInitializer;
namespace internal
{
using MapDefCtorFunctor = std::function<std::shared_ptr<mrpt::maps::TMetricMapInitializer>(void)>;
using MapCtorFromDefFunctor = std::function<std::shared_ptr<mrpt::maps::CMetricMap>(
    const mrpt::maps::TMetricMapInitializer&)>;

/** Class factory & registry for map classes. Used from
 * mrpt::maps::TMetricMapInitializer */
struct TMetricMapTypesRegistry
{
 public:
  static TMetricMapTypesRegistry& Instance();
  /** Return the index of the class in the list (not important, just used as a
   * trick to initialize static members) */
  size_t doRegister(const std::string& name, MapDefCtorFunctor func1, MapCtorFromDefFunctor func2);

  /** Return nullptr if not found */
  std::shared_ptr<mrpt::maps::TMetricMapInitializer> factoryMapDefinition(
      const std::string& className) const;

  /** Return nullptr if not found */
  std::shared_ptr<mrpt::maps::CMetricMap> factoryMapObjectFromDefinition(
      const mrpt::maps::TMetricMapInitializer& mi) const;

  struct InfoPerMapClass
  {
    InfoPerMapClass() = default;
    InfoPerMapClass(const MapDefCtorFunctor& DefCtor, const MapCtorFromDefFunctor& MapCtor) :
        defCtor(DefCtor), mapCtor(MapCtor)
    {
    }

    MapDefCtorFunctor defCtor;
    MapCtorFromDefFunctor mapCtor;
  };

  using TListRegisteredMaps = std::map<std::string, InfoPerMapClass>;
  const TListRegisteredMaps& getAllRegistered() const { return m_registry; }

 private:
  TMetricMapTypesRegistry() = default;  // Access thru singleton in Instance()
  TListRegisteredMaps m_registry;
};

/** Add a MAP_DEFINITION_START() ... MAP_DEFINITION_END() block inside the
 * declaration of each metric map */
#define MAP_DEFINITION_START(_CLASS_NAME_)                                  \
 public:                                                                    \
  /** @name Map Definition Interface stuff (see                             \
   * mrpt::maps::TMetricMapInitializer) @{ */                               \
  struct TMapDefinitionBase : public mrpt::maps::TMetricMapInitializer      \
  {                                                                         \
    TMapDefinitionBase() : TMetricMapInitializer(CLASS_ID(_CLASS_NAME_)) {} \
  };                                                                        \
  struct TMapDefinition : public TMapDefinitionBase                         \
  {
#define MAP_DEFINITION_END(_CLASS_NAME_)                                                           \
  TMapDefinition();                                                                                \
                                                                                                   \
 protected:                                                                                        \
  void loadFromConfigFile_map_specific(                                                            \
      const mrpt::config::CConfigFileBase& source, const std::string& sectionNamePrefix) override; \
  void dumpToTextStream_map_specific(std::ostream& out) const override;                            \
  }                                                                                                \
  ;                                                                                                \
  /** Returns default map definition initializer. See                                              \
   * mrpt::maps::TMetricMapInitializer */                                                          \
  static std::shared_ptr<mrpt::maps::TMetricMapInitializer> MapDefinition();                       \
  /** Constructor from a map definition structure: initializes the map and                         \
   * its parameters accordingly */                                                                 \
  static std::shared_ptr<_CLASS_NAME_> CreateFromMapDefinition(                                    \
      const mrpt::maps::TMetricMapInitializer& def);                                               \
  static std::shared_ptr<mrpt::maps::CMetricMap> internal_CreateFromMapDefinition(                 \
      const mrpt::maps::TMetricMapInitializer& def);                                               \
  /** ID used to initialize class registration (just ignore it) */                                 \
  static const size_t m_private_map_register_id;                                                   \
/** @} */

/** Registers one map class into TMetricMapInitializer factory.
 * One or several alternative class names can be provided, separated with
 * whitespaces or commas */
#define MAP_DEFINITION_REGISTER(_CLASSNAME_STRINGS, _CLASSNAME_WITH_NS)                  \
  const size_t _CLASSNAME_WITH_NS::m_private_map_register_id =                           \
      mrpt::maps::internal::TMetricMapTypesRegistry::Instance().doRegister(              \
          _CLASSNAME_STRINGS, &_CLASSNAME_WITH_NS::MapDefinition,                        \
          &_CLASSNAME_WITH_NS::internal_CreateFromMapDefinition);                        \
  std::shared_ptr<mrpt::maps::TMetricMapInitializer> _CLASSNAME_WITH_NS::MapDefinition() \
  {                                                                                      \
    return std::make_shared<_CLASSNAME_WITH_NS::TMapDefinition>();                       \
  }                                                                                      \
  std::shared_ptr<_CLASSNAME_WITH_NS> _CLASSNAME_WITH_NS::CreateFromMapDefinition(       \
      const mrpt::maps::TMetricMapInitializer& def)                                      \
  {                                                                                      \
    return std::dynamic_pointer_cast<_CLASSNAME_WITH_NS>(                                \
        _CLASSNAME_WITH_NS::internal_CreateFromMapDefinition(def));                      \
  }
}  // namespace internal
}  // namespace maps
}  // namespace mrpt
