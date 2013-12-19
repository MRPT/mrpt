/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

#include <mrpt/reactivenav.h>  // Precomp header

//#if defined(_MSC_VER)
//	#pragma warning(disable:4267)
//#endif

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
		*version = 7;
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
			out.WriteBuffer((const void*)&(*infoPerPTG[i].TP_Obstacles.begin()), m * sizeof(infoPerPTG[i].TP_Obstacles[0]));

			out << infoPerPTG[i].TP_Target << infoPerPTG[i].timeForTPObsTransformation << infoPerPTG[i].timeForHolonomicMethod;
			out << infoPerPTG[i].desiredDirection << infoPerPTG[i].desiredSpeed << infoPerPTG[i].evaluation;
			out << *infoPerPTG[i].HLFR;
		}

		out << nSelectedPTG << WS_Obstacles << robotOdometryPose << WS_target_relative << v << w << executionTime;

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
				in.ReadBuffer((void*)&(*infoPerPTG[i].TP_Obstacles.begin()), m * sizeof(infoPerPTG[i].TP_Obstacles[0]));

				in >> infoPerPTG[i].TP_Target >> infoPerPTG[i].timeForTPObsTransformation >> infoPerPTG[i].timeForHolonomicMethod;
				in >> infoPerPTG[i].desiredDirection >> infoPerPTG[i].desiredSpeed >> infoPerPTG[i].evaluation;
				in >> infoPerPTG[i].HLFR;
			}

			in >> nSelectedPTG >> WS_Obstacles >> robotOdometryPose >> WS_target_relative >> v >> w >> executionTime;


			if (version<6)
			{
				vector_float prevV,prevW,prevSelPTG;

				// Previous values: (Removed in version 6)
				in >> n;
				prevV.resize(n);
				in.ReadBuffer((void*)&(*prevV.begin()),n*sizeof(prevV[0]));

				in >> n;
				prevW.resize(n);
				in.ReadBuffer((void*)&(*prevW.begin()),n*sizeof(prevW[0]));

				in >> n;
				prevSelPTG.resize(n);
				in.ReadBuffer((void*)&(*prevSelPTG.begin()),n*sizeof(prevSelPTG[0]));
			}

			in >> n;
			robotShape_x.resize(n);
			robotShape_y.resize(n);
			in.ReadBuffer((void*)&(*robotShape_x.begin()), n*sizeof(robotShape_x[0]));
			in.ReadBuffer((void*)&(*robotShape_y.begin()), n*sizeof(robotShape_y[0]));

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

