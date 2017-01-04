/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/utils/utils_defs.h>

namespace mrpt
{
	namespace system
	{
		/** @defgroup vector_loadsave Load and save vectors to files (in #include <mrpt/system/vector_loadsave.h>)
		  * \ingroup mrpt_base_grp
		  * @{ */

		/** A useful function for debugging, which saves a std::vector into a text file (compat. with MATLAB)
		* \return Returns false on any error, true on everything OK.
		*/
		bool  BASE_IMPEXP vectorToTextFile( const std::vector<float> &vec, const std::string &fileName, bool append = false, bool byRows=false );
		//! \overload
		bool  BASE_IMPEXP vectorToTextFile( const std::vector<double> &vec, const std::string &fileName, bool append = false, bool byRows=false );
		//! \overload
		bool  BASE_IMPEXP vectorToTextFile( const std::vector<int> &vec, const std::string &fileName, bool append = false, bool byRows=false );
		//! \overload
		bool  BASE_IMPEXP vectorToTextFile( const std::vector<size_t> &vec, const std::string &fileName, bool append = false, bool byRows=false );
		//! \overload
		template <class EIGEN_MATRIX>
		bool vectorToTextFile( const EIGEN_MATRIX &vec, const std::string &fileName ) {
			try {
				vec.saveToTextFile(fileName);
				return true;
			} catch(...) {return false;}
		}

		/** Load a std::vector from a text file (compat. with MATLAB)
		* \return Returns false on any error, true on everything OK.
		* \sa loadBinaryFile
		*/
		bool  BASE_IMPEXP vectorFromTextFile( std::vector<double> &vec, const std::string &fileName, const bool byRows=false );

		/** Saves a vector directly as a binary dump to a file:
		* \return Returns false on any error, true on everything OK.
		* \sa loadBinaryFile
		*/
		bool BASE_IMPEXP vectorToBinaryFile( const vector_byte &vec, const std::string &fileName );

		/** Loads a entire file as a vector of bytes.
		* \return Returns false on any error, true on everything OK.
		* \sa vectorToBinaryFile
		*/
		bool BASE_IMPEXP loadBinaryFile( vector_byte &out_data, const std::string &fileName );

		/** @} */

	} // End of namespace
} // End of namespace
