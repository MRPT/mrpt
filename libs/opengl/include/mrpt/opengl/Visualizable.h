/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/opengl/opengl_frwds.h>

#include <memory>  // shared_ptr

namespace mrpt::opengl
{
/** Interface for classes visualizable as an mrpt::opengl::CSetOfObjects.
 *
 * \ingroup mrpt_opengl_grp
 */
class Visualizable
{
   public:
	Visualizable() = default;
	~Visualizable() = default;

	/** Inserts 3D primitives representing this object into the provided
	 * container.
	 * Note that the former contents of `o` are not cleared.
	 *
	 * \sa getVisualization()
	 */
	virtual void getVisualizationInto(mrpt::opengl::CSetOfObjects& o) const = 0;

	/** Creates 3D primitives representing this objects.
	 * This is equivalent to getVisualizationInto() but creating, and returning
	 * by value, a new rpt::opengl::CSetOfObjects::Ptr shared pointer.
	 * \sa getVisualizationInto()
	 */
	std::shared_ptr<mrpt::opengl::CSetOfObjects> getVisualization() const;
};

}  // namespace mrpt::opengl
