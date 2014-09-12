/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "reactivenav-precomp.h" // Precomp header

#include <mrpt/reactivenav/CLogFileRecord.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/utils/CStream.h>

using namespace mrpt::reactivenav;

IMPLEMENTS_SERIALIZABLE( CLogFileRecord, CSerializable,mrpt::reactivenav )


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

/*---------------------------------------------------------------
					Copy constructor
  ---------------------------------------------------------------*/
void CLogFileRecord::operator =( CLogFileRecord &o)
{
	// Free CHolonomicLogFileRecord objects:
	// ----------------------------------------
	freeInfoPerPTGs();

	// Copy
	// --------------------------
	infoPerPTG.resize( o.infoPerPTG.size() );
	for (unsigned int i=0;i<o.infoPerPTG.size();i++)
	{
		infoPerPTG[i] = o.infoPerPTG[i];
		// Copy the pointer only:
		infoPerPTG[i].HLFR = o.infoPerPTG[i].HLFR;
		o.infoPerPTG[i].HLFR.clear_unique();
	}

	WS_Obstacles = o.WS_Obstacles;
	WS_target_relative = o.WS_target_relative;
	robotOdometryPose = o.robotOdometryPose;
	v = o.v;
	w = o.w;
	nSelectedPTG = o.nSelectedPTG;
	executionTime = o.executionTime;
	estimatedExecutionPeriod = o.estimatedExecutionPeriod;

	robotShape_x=o.robotShape_x;
	robotShape_y=o.robotShape_y;

	actual_v = o.actual_v;
	actual_w = o.actual_w;

    timestamp = o.timestamp;
	nPTGs = o.nPTGs;
	securityDistances = o.securityDistances;

	navigatorBehavior = o.navigatorBehavior;
}


/*---------------------------------------------------------------
					freeInfoPerPTGs
  ---------------------------------------------------------------*/
void  CLogFileRecord::freeInfoPerPTGs()
{
	int	i,n;
	try
	{
		n = infoPerPTG.size();
		for (i=0;i<n;i++)
		{
			infoPerPTG[i].HLFR.clear_unique();
		}
	} catch(...) { };

	infoPerPTG.clear();
}

/*---------------------------------------------------------------
					Destructor
  ---------------------------------------------------------------*/
CLogFileRecord::~CLogFileRecord()
{
	freeInfoPerPTGs();
}


/*---------------------------------------------------------------
						writeToStream
 ---------------------------------------------------------------*/
void  CLogFileRecord::writeToStream(CStream &out,int *version) const
{
	if (version)
		*version = 8;
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
		}

		out << nSelectedPTG << WS_Obstacles << robotOdometryPose << WS_target_relative /*v8*/ << v << w << executionTime;

		// Previous values: REMOVED IN VERSION #6

		n = robotShape_x.size();
		out << n;
		if (n) {
			out.WriteBuffer((const void*)&(*robotShape_x.begin()), n*sizeof(robotShape_x[0]));
			out.WriteBuffer((const void*)&(*robotShape_y.begin()), n*sizeof(robotShape_y[0]));
		}

		// Version 1 ---------
		out << actual_v << actual_w;

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
		n = securityDistances.size();
		out << n;
		for (i=0;i<n;i++) out << securityDistances[i];

		// Version 5 -----------
		out << navigatorBehavior; // Removed in version 6: << doorCrossing_P1 << doorCrossing_P2;

		// version 7:
		out << timestamp;
	}
}

/*---------------------------------------------------------------
						readFromStream
 ---------------------------------------------------------------*/
void  CLogFileRecord::readFromStream(CStream &in,int version)
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
		{
			// Version 0 --------------
			uint32_t  i,n;

			// Free previous if required:
			freeInfoPerPTGs();


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
			
			in >> v >> w >> executionTime;


			if (version<6)
			{
				CVectorFloat prevV,prevW,prevSelPTG;

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
				in >> actual_v >> actual_w;
			}
			else
			{	// Default values for old versions:
				actual_v = actual_w = 0;
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
				in >> n;
				securityDistances.resize(n);
				for (i=0;i<n;i++)
					in >> securityDistances[i];
			}
			else
			{
				nPTGs = infoPerPTG.size();
				securityDistances.resize(1,0.0f);
			}

			if (version > 4)
			{
				// Version 5 ----------
				in >> navigatorBehavior;

				if (version<6)  // Removed in version 6:
				{
					mrpt::poses::CPoint2D doorCrossing_P1,doorCrossing_P2;
					in >> doorCrossing_P1 >> doorCrossing_P2;
				}
			}
			else
			{
				navigatorBehavior = 0;
			}

			if (version>6)
			{
			    in >> timestamp;
			}
			else
			{
			    timestamp = INVALID_TIMESTAMP;
			}

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}

