/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
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
