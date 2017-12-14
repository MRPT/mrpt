/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#ifndef CAction_H
#define CAction_H

#include <mrpt/serialization/CSerializable.h>
#include <mrpt/system/datetime.h>

namespace mrpt
{
/** \ingroup mrpt_obs_grp */
namespace obs
{
/** Declares a class for storing a robot action. It is used in
 * mrpt::obs::CRawlog,
 *    for logs storage and particle filter based simulations.
 *  See derived classes for implementations.
 *
 * \sa CActionCollection, CRawlog
 * \ingroup mrpt_obs_grp
 */
class CAction : public mrpt::serialization::CSerializable
{
	DEFINE_VIRTUAL_SERIALIZABLE(CAction)

	/** Default constructor
	  */
	CAction();

	/** Constructor
	  */
	virtual ~CAction();

	/** The associated time-stamp.
	  *  This was added at 2-Dec-2007, new serialization versions have been
	 * added to derived classes to manage this time-stamp.
	  *  Prior versions will be read as having a INVALID_TIMESTAMP value.
	 */
	mrpt::system::TTimeStamp timestamp;

};  // End of class def.

}  // End of namespace
}  // End of namespace

#endif
