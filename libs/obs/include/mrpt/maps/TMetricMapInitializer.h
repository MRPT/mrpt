/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/rtti/CObject.h>  // For TRuntimeClassId
#include <mrpt/maps/TMetricMapTypesRegistry.h>
#include <mrpt/maps/metric_map_types.h>
#include <deque>

namespace mrpt
{
namespace maps
{
class TSetOfMetricMapInitializers;

/** Virtual base for specifying the kind and parameters of one map (normally, to
 * be inserted into mrpt::maps::CMultiMetricMap)
  *  See `mrpt::maps::TSetOfMetricMapInitializers::loadFromConfigFile()` as an
 * easy way of initialize this object, or
  *  construct with the factory methods `<metric_map_class>::MapDefinition()`
 * and `TMetricMapInitializer::factory()`
  *
  * \sa TSetOfMetricMapInitializers, mrpt::maps::CMultiMetricMap
  * \ingroup mrpt_obs_grp
  */
struct TMetricMapInitializer : public mrpt::config::CLoadableOptions
{
	friend class TSetOfMetricMapInitializers;
	/** Smart pointer to TMetricMapInitializer */
	using Ptr = std::shared_ptr<TMetricMapInitializer>;

	/** Common params for all maps: These are automatically set in
	 * TMetricMapTypesRegistry::factoryMapObjectFromDefinition()  */
	mrpt::maps::TMapGenericParams genericMapParams;

	/** Load all params from a config file/source. For examples and format, read
	 * the docs of mrpt::maps::CMultiMetricMap
	  * Typical section names:
	  *  - `<sectionNamePrefix>_creationOpts`
	  *  - `<sectionNamePrefix>_insertOpts`
	  *  - `<sectionNamePrefix>_likelihoodOpts`
	  */
	void loadFromConfigFile(
		const mrpt::config::CConfigFileBase& source,
		const std::string& sectionNamePrefix) override;  // See base docs
	void dumpToTextStream(std::ostream& out) const override;
};

}  // End of namespace
}  // End of namespace
