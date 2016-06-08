/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header

#include <mrpt/nav/reactive/CLogFileRecord.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/utils/CStream.h>

using namespace mrpt::nav;

IMPLEMENTS_SERIALIZABLE( CLogFileRecord, CSerializable,mrpt::nav )


/*---------------------------------------------------------------
					Constructor
  ---------------------------------------------------------------*/
CLogFileRecord::CLogFileRecord() :
    timestamp ( INVALID_TIMESTAMP ),
    nPTGs     ( 0 )
{
	infoPerPTG.clear();
	WS_Obstacles.clear();
}

CLogFileRecord::~CLogFileRecord()
{
}

/*---------------------------------------------------------------
						writeToStream
 ---------------------------------------------------------------*/
void  CLogFileRecord::writeToStream(mrpt::utils::CStream &out,int *version) const
{
	if (version)
		*version = 10;
	else
	{
		uint32_t	i,n;

		// Version 0 ---------
		n = infoPerPTG.size();
		out << n;
		for (i=0;i<n;i++)
		{
			out << infoPerPTG[i].PTG_desc.c_str();

			uint32_t m = infoPerPTG[i].TP_Obstacles.size();
			out << m;
			if (m) out.WriteBuffer((const void*)&(*infoPerPTG[i].TP_Obstacles.begin()), m * sizeof(infoPerPTG[i].TP_Obstacles[0]));

			out << infoPerPTG[i].TP_Target;  // v8: CPoint2D -> TPoint2D
			out << infoPerPTG[i].timeForTPObsTransformation << infoPerPTG[i].timeForHolonomicMethod;
			out << infoPerPTG[i].desiredDirection << infoPerPTG[i].desiredSpeed << infoPerPTG[i].evaluation;
			out << *infoPerPTG[i].HLFR;

			// Version 9: Removed security distances. Added optional field with PTG info.
			MRPT_TODO("Replace all this by making PTGs CSerializable and not storing trajectories but only the params");
#if 0
			const bool there_is_ptg_data = infoPerPTG[i].ptg_trajectory.present();
			out << there_is_ptg_data;
			if (there_is_ptg_data)
			{
				infoPerPTG[i].ptg_trajectory->saveTrajectories( out );
			}
#endif
		}
		out << nSelectedPTG << WS_Obstacles << robotOdometryPose << WS_target_relative /*v8*/ << cmd_vel /*v10*/ << executionTime;

		// Previous values: REMOVED IN VERSION #6

		n = robotShape_x.size();
		out << n;
		if (n) {
			out.WriteBuffer((const void*)&(*robotShape_x.begin()), n*sizeof(robotShape_x[0]));
			out.WriteBuffer((const void*)&(*robotShape_y.begin()), n*sizeof(robotShape_y[0]));
		}

		// Version 1 ---------
		out << cur_vel /*v10*/;

		// Version 2 ----------
		out << estimatedExecutionPeriod;

		// Version 3 ----------
		for (i=0;i<infoPerPTG.size();i++)
		{
			n = infoPerPTG[i].evalFactors.size();

			out << n;
			for (unsigned int j=0;j<n;j++)
				out << infoPerPTG[i].evalFactors[j];
		}

		// Version 4 ----------
		out << nPTGs;

		// version 7:
		out << timestamp;
	}
}

/*---------------------------------------------------------------
						readFromStream
 ---------------------------------------------------------------*/
void  CLogFileRecord::readFromStream(mrpt::utils::CStream &in,int version)
{
	switch(version)
	{
	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
	case 5:
	case 6:
	case 7:
	case 8:
	case 9:
	case 10:
		{
			// Version 0 --------------
			uint32_t  i,n;

			infoPerPTG.clear();

			in >> n;
			infoPerPTG.resize(n);
			for (i=0;i<n;i++)
			{
				char str[256];
				in >> str;
				infoPerPTG[i].PTG_desc = std::string(str);

				int32_t m;
				in >> m;
				infoPerPTG[i].TP_Obstacles.resize(m);
				if (m) in.ReadBufferFixEndianness( &(*infoPerPTG[i].TP_Obstacles.begin()), m );

				if (version>=8)
					in >> infoPerPTG[i].TP_Target; 
				else
				{
					mrpt::poses::CPoint2D pos;
					in >> pos;
					infoPerPTG[i].TP_Target = mrpt::math::TPoint2D(pos);
				}

				in >> infoPerPTG[i].timeForTPObsTransformation >> infoPerPTG[i].timeForHolonomicMethod;
				in >> infoPerPTG[i].desiredDirection >> infoPerPTG[i].desiredSpeed >> infoPerPTG[i].evaluation;
				in >> infoPerPTG[i].HLFR;

				if (version>=9) // Extra PTG info
				{
					bool there_is_ptg_data;
					in >> there_is_ptg_data;
					if (there_is_ptg_data) 
					{
					MRPT_TODO("Replace all this by making PTGs CSerializable and not storing trajectories but only the params");
#if 0
						infoPerPTG[i].ptg_trajectory = CParameterizedTrajectoryGeneratorPtr( new CPTG_Dummy );
						infoPerPTG[i].ptg_trajectory->loadTrajectories(in);
#endif
					}
				}
			}

			in >> nSelectedPTG >> WS_Obstacles >> robotOdometryPose; 
				
			if (version>=8)
				in >> WS_target_relative; 
			else
			{
				mrpt::poses::CPoint2D pos;
				in >> pos;
				WS_target_relative = mrpt::math::TPoint2D(pos);
			}
			
			if (version >= 10) {
				in >> cmd_vel;
			}
			else {
				float v, w;
				in >> v >> w;
				cmd_vel.resize(2);
				cmd_vel[0] = v;
				cmd_vel[1] = w;
			}
			in >> executionTime;

			if (version<6)
			{
				mrpt::math::CVectorFloat prevV,prevW,prevSelPTG;

				// Previous values: (Removed in version 6)
				in >> n;
				prevV.resize(n);
				if (n) in.ReadBufferFixEndianness( &(*prevV.begin()),n);

				in >> n;
				prevW.resize(n);
				if (n) in.ReadBufferFixEndianness( &(*prevW.begin()),n);

				in >> n;
				prevSelPTG.resize(n);
				if (n) in.ReadBufferFixEndianness( &(*prevSelPTG.begin()),n);
			}

			in >> n;
			robotShape_x.resize(n);
			robotShape_y.resize(n);
			if (n) {
				in.ReadBufferFixEndianness( &(*robotShape_x.begin()), n);
				in.ReadBufferFixEndianness( &(*robotShape_y.begin()), n);
			}

			if (version > 0)
			{	// Version 1 --------------
				if (version >= 10) {
					in >> cur_vel;
				}
				else {
					float actual_v, actual_w;
					in >> actual_v >> actual_w;
					cur_vel.resize(2);
					cur_vel[0] = actual_v;
					cur_vel[1] = actual_w;
				}
			}
			else
			{	// Default values for old versions:
				cur_vel.resize(2, 0.0);
			}

			if (version > 1)
			{	// Version 2 --------------
				in >> estimatedExecutionPeriod;
			}
			else
			{	// Default values for old versions:
				estimatedExecutionPeriod = 0.06f;
			}

			if (version > 2)
			{
				// Version 3 ----------
				for (i=0;i<infoPerPTG.size();i++)
				{

					in >> n;
					infoPerPTG[i].evalFactors.resize(n);
					for (unsigned int j=0;j<n;j++)
						in >> infoPerPTG[i].evalFactors[j];
				}

			}
			else
			{
				for (i=0;i<infoPerPTG.size();i++)
					infoPerPTG[i].evalFactors.resize(0);
			}

			if (version > 3)
			{
				// Version 4 ----------
				in >> nPTGs;
				if (version <9)  // Old "securityDistances", now unused.
				{
					in >> n;
					float dummy;
					for (i=0;i<n;i++)
						in >> dummy;
				}
			}
			else
			{
				nPTGs = infoPerPTG.size();
			}

			if (version > 4)
			{
				if (version < 10)
				{
					int32_t navigatorBehavior; // removed in v10
					in >> navigatorBehavior;
				}

				if (version<6)  // Removed in version 6:
				{
					mrpt::poses::CPoint2D doorCrossing_P1,doorCrossing_P2;
					in >> doorCrossing_P1 >> doorCrossing_P2;
				}
			}

			if (version>6) {
				in >> timestamp;
			}
			else {
				timestamp = INVALID_TIMESTAMP;
			}

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}

