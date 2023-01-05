/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

namespace mrpt::containers
{
/** See mrpt::containers::yaml::PrintAsYaml
 *
 * \ingroup mrpt_containers_yaml
 * \note [New in MRPT 2.1.0]
 */
struct YamlEmitOptions
{
	/** Emit the `%YAML 1.2\n---\n` at the beginning. */
	bool emitHeader = true;

	bool emitComments = true;
	bool endWithNewLine = true;

	bool indentSequences = true;
};

}  // namespace mrpt::containers
