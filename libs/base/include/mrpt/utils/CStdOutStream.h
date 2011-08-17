/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */
#ifndef  CSTDOUTSTREAM_H
#define  CSTDOUTSTREAM_H

#include <mrpt/utils/CStream.h>

/*---------------------------------------------------------------
	Class
  ---------------------------------------------------------------*/
namespace mrpt
{
namespace utils
{
	/** This CStdOutStream derived class allow printing to standard out, normally
	 *    the console text output. Please notice CStdOutStream's are binary streams,
	 *    so "char *" data types only should be used if textual outputs are
	 *    desired.
	 *
	 * \sa CStream
	 * \ingroup mrpt_base_grp
	 */
	class BASE_IMPEXP CStdOutStream : public CStream
	{
	protected:
		 /** Method responsible for reading from the stream:
		  *   In this class it has no effect.
		  */
		size_t  Read(void *Buffer, size_t Count) { THROW_EXCEPTION("Read-only stream"); }

		/** Method responsible for writing to the stream.
		 *  Write attempts to write up to Count bytes to Buffer, and returns the number of bytes actually written.
		 */
		size_t Write(const void *Buffer,size_t Count);

	public:
		 /** Constructor
		  */
		CStdOutStream() { }

		 /** Destructor
		 */
		virtual ~CStdOutStream() { }

		/** It has no efect in this class.
		 */
		uint64_t Seek(long Offset, CStdOutStream::TSeekOrigin Origin = sFromBeginning)
			{ THROW_EXCEPTION("Invalid operation for this kind of stream"); }

		/** It has no efect in this class.
		 */
		uint64_t getTotalBytesCount()
			{ THROW_EXCEPTION("Invalid operation for this kind of stream"); }

		/** It has no efect in this class.
		 */
		uint64_t getPosition()
			{ THROW_EXCEPTION("Invalid operation for this kind of stream"); }

	}; // End of class def.

} // End of namespace
} // End of namespace

#endif
