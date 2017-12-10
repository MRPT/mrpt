/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

namespace mrpt
{
namespace serialization
{
/** Virtual base class for "archives": classes abstracting I/O streams.
* This class separates the implementatins details of serialization (in
* mrpt::serialization::CSerializable) and data storage (derived classes).
*
* Two main sets of implementations are provided:
* - CArchiveStream: for MRPT mrpt::io::CStream objects, and
* - CArchiveStdIStream and CArchiveStdOStream: for std::istream and std::ostream, respectively.
*
* \sa mrpt::io::CStream, mrpt::serialization::CSerializable
* \ingroup mrpt_serialization_grp
*/
class CArchive
{
	CArchive() {}
	virtual ~CArchive() {}
   public:
};

}  // End of namespace
}  // End of namespace
