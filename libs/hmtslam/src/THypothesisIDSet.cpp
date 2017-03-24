/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hmtslam-precomp.h" // Precomp header

using namespace mrpt::hmtslam;
using namespace mrpt::utils;


IMPLEMENTS_SERIALIZABLE(THypothesisIDSet, CSerializable, mrpt::hmtslam)


/*---------------------------------------------------------------
					writeToStream
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  THypothesisIDSet::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	if (version)
		*version = 0;
	else
	{
		uint32_t	N = (uint32_t) size();
		out << N;
		for (const_iterator it=begin();it!=end();++it)
			out << *it;
	}
}

/*---------------------------------------------------------------
					readFromStream
   Implements the reading from a CStream capability of
      CSerializable objects
  ---------------------------------------------------------------*/
void  THypothesisIDSet::readFromStream(mrpt::utils::CStream &in, int version)
{
	switch(version)
	{
	case 0:
		{
			uint32_t	i,N;
			in >> N;

			clear();
			for (i=0;i<N;i++)
			{
				THypothesisID tmp;
				in >> tmp;
				insert(tmp);
			}

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}
