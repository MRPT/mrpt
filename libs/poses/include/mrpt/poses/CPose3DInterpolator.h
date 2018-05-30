/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/poses/CPoseInterpolatorBase.h>
#include <mrpt/serialization/CSerializable.h>

namespace mrpt
{
namespace poses
{
/** This class stores a time-stamped trajectory in SE(3) (CPose3D poses).
  *  It can also interpolate SE(3) poses over time using linear, splines or
 * SLERP interpolation, as set in CPose3DInterpolator::setInterpolationMethod()
  *  Usage:
  *   - Insert new poses into the sequence with CPose3DInterpolator::insert()
  *   - Query an exact/interpolated pose with
 * CPose3DInterpolator::interpolate().
  * Example:
  * \code
  * CPose3DInterpolator		path;
  *
  * path.setInterpolationMethod( mrpt::poses::imSplineSlerp );
  *
  * path.insert( t0, mrpt::poses::CPose3D(...) );
  * path.insert( t1, mrpt::math::TPose3D(...) ); // prefered (faster)
  *
  * mrpt::math::TPose3D p;
  * bool valid;
  *
  * cout << "Pose at t: " << path.interpolate(t,p,valid).asString() << endl;
  * \endcode
  *
  *  Time is represented with mrpt::system::TTimeStamp. See mrpt::system for
 * methods and utilities to manage these time references.
  *
  *  See TInterpolatorMethod for the list of interpolation methods. The default
 * method at constructor is "imLinearSlerp".
  *
  * \sa CPoseOrPoint
 * \ingroup interpolation_grp poses_grp
 */
class CPose3DInterpolator : public mrpt::serialization::CSerializable,
							public mrpt::poses::CPoseInterpolatorBase<3>
{
	DEFINE_SERIALIZABLE(CPose3DInterpolator)
  
public:
  template <typename SCHEMA_CAPABLE>
	SCHEMA_CAPABLE serializeTo() const
	{
		SCHEMA_CAPABLE out;
		out["datatype"] = this->GetRuntimeClass()->className;
		out["version"] = 1;
        int k = 0;
        for(const auto& e : m_path)
        {
            out["path"][k][0] = e.first;
            out["path"][k][1] = e.second.serializeTo<SCHEMA_CAPABLE>();
            ++k;
        }
        out["N"] = k;
        return out;	
	}

	/** Templatized serializeFrom function 
	 * Serializes only if the datatype matched to className 
	*/
	template <typename SCHEMA_CAPABLE>
	void serializeFrom(SCHEMA_CAPABLE& in)
	{
		uint8_t version = in.get("version",0);
		if(in["datatype"] == this->GetRuntimeClass()->className)
		{
			switch(version)
			{
				case 1:
				{
					uint32_t N = in["N"];
          for(int i = 0;i < N ;i++ )
          {
              mrpt::math::TPose3D ps;
              ps.serializeFrom<SCHEMA_CAPABLE>(in["path"][i][1]);
              (*this).insert(in["path"][i][0], ps);
          }
                        
				}
				break;
				default:
					MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
			}
		}
	}
};  // End of class def.
}  // End of namespace
}  // End of namespace
