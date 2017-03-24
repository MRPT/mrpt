/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "maps-precomp.h" // Precomp header

#include <mrpt/maps/CBeacon.h>
#include <mrpt/maps/CBeaconMap.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/utils/CStream.h>

#include <mrpt/system/os.h>
#include <mrpt/math/geometry.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CEllipsoid.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CText.h>

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CBeacon, CSerializable,mrpt::maps)


/*---------------------------------------------------------------
						Default constructor
  ---------------------------------------------------------------*/
CBeacon::CBeacon( ) :
	m_typePDF(pdfGauss),
	m_locationMC(1),
	m_locationGauss(),
	m_locationSOG(1),
	m_ID(INVALID_BEACON_ID)
{
}

/*---------------------------------------------------------------
						Destructor
  ---------------------------------------------------------------*/
CBeacon::~CBeacon()
{
}

/*---------------------------------------------------------------
					writeToStream
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CBeacon::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	if (version)
		*version = 0;
	else
	{
		uint32_t	i = m_ID;
		uint32_t	j = m_typePDF;
		out << i << j << m_locationMC << m_locationGauss << m_locationSOG;
	}
}

/*---------------------------------------------------------------
					readFromStream
   Implements the reading from a CStream capability of
      CSerializable objects
  ---------------------------------------------------------------*/
void  CBeacon::readFromStream(mrpt::utils::CStream &in, int version)
{
	switch(version)
	{
	case 0:
		{
			uint32_t	i,j;
			in >> i >> j >> m_locationMC >> m_locationGauss >> m_locationSOG;
			m_ID = i;
			m_typePDF = static_cast<TTypePDF>(j);
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}

/*---------------------------------------------------------------
					getMean
  ---------------------------------------------------------------*/
void CBeacon::getMean(CPoint3D &p) const
{
	MRPT_START
	switch (m_typePDF)
	{
	case pdfMonteCarlo:	m_locationMC.getMean(p);		break;
	case pdfGauss:		m_locationGauss.getMean(p);		break;
	case pdfSOG:		m_locationSOG.getMean(p);		break;
	default:			THROW_EXCEPTION("ERROR: Invalid 'm_typePDF' value");
	};
	MRPT_END
}

/*---------------------------------------------------------------
					getCovarianceAndMean
  ---------------------------------------------------------------*/
void CBeacon::getCovarianceAndMean(CMatrixDouble33 &COV,CPoint3D &p) const
{
	MRPT_START
	switch (m_typePDF)
	{
	case pdfMonteCarlo:	m_locationMC.getCovarianceAndMean(COV,p);		break;
	case pdfGauss:		m_locationGauss.getCovarianceAndMean(COV,p);	break;
	case pdfSOG:		m_locationSOG.getCovarianceAndMean(COV,p);		break;
	default:			THROW_EXCEPTION("ERROR: Invalid 'm_typePDF' value");
	};
	MRPT_END
}

/*---------------------------------------------------------------
					bayesianFusion
  ---------------------------------------------------------------*/
void CBeacon::bayesianFusion(const  CPointPDF &p1,const  CPointPDF &p2, const double &minMahalanobisDistToDrop)
{
	MRPT_START
	switch (m_typePDF)
	{
	case pdfMonteCarlo:	m_locationMC.bayesianFusion(p1,p2,minMahalanobisDistToDrop);	break;
	case pdfGauss:		m_locationGauss.bayesianFusion(p1,p2,minMahalanobisDistToDrop);	break;
	case pdfSOG:		m_locationSOG.bayesianFusion(p1,p2,minMahalanobisDistToDrop);		break;
	default:			THROW_EXCEPTION("ERROR: Invalid 'm_typePDF' value");
	};
	MRPT_END
}


/*---------------------------------------------------------------
					drawSingleSample
  ---------------------------------------------------------------*/
void CBeacon::drawSingleSample(CPoint3D &outSample) const
{
	MRPT_START
	switch (m_typePDF)
	{
	case pdfMonteCarlo:	m_locationMC.drawSingleSample(outSample);		break;
	case pdfGauss:		m_locationGauss.drawSingleSample(outSample);	break;
	case pdfSOG:		m_locationSOG.drawSingleSample(outSample);		break;
	default:			THROW_EXCEPTION("ERROR: Invalid 'm_typePDF' value");
	};
	MRPT_END
}


/*---------------------------------------------------------------
					copyFrom
  ---------------------------------------------------------------*/
void  CBeacon::copyFrom(const CPointPDF &o)
{
	MRPT_START
	switch (m_typePDF)
	{
	case pdfMonteCarlo:	m_locationMC.copyFrom(o);		break;
	case pdfGauss:		m_locationGauss.copyFrom(o);	break;
	case pdfSOG:		m_locationSOG.copyFrom(o);		break;
	default:			THROW_EXCEPTION("ERROR: Invalid 'm_typePDF' value");
	};
	MRPT_END
}

/*---------------------------------------------------------------
					saveToTextFile
  ---------------------------------------------------------------*/
void  CBeacon::saveToTextFile(const std::string &file) const
{
	MRPT_START
	switch (m_typePDF)
	{
	case pdfMonteCarlo:	m_locationMC.saveToTextFile(file);		break;
	case pdfGauss:		m_locationGauss.saveToTextFile(file);	break;
	case pdfSOG:		m_locationSOG.saveToTextFile(file);		break;
	default:			THROW_EXCEPTION("ERROR: Invalid 'm_typePDF' value");
	};
	MRPT_END
}

/*---------------------------------------------------------------
					changeCoordinatesReference
  ---------------------------------------------------------------*/
void  CBeacon::changeCoordinatesReference( const CPose3D &newReferenceBase )
{
	MRPT_START
	switch (m_typePDF)
	{
	case pdfMonteCarlo:	m_locationMC.changeCoordinatesReference(newReferenceBase);		break;
	case pdfGauss:		m_locationGauss.changeCoordinatesReference(newReferenceBase);	break;
	case pdfSOG:		m_locationSOG.changeCoordinatesReference(newReferenceBase);		break;
	default:			THROW_EXCEPTION("ERROR: Invalid 'm_typePDF' value");
	};
	MRPT_END
}

/*---------------------------------------------------------------
					getAs3DObject
  ---------------------------------------------------------------*/
void  CBeacon::getAs3DObject( mrpt::opengl::CSetOfObjectsPtr	&outObj ) const
{
	MRPT_START

	switch (m_typePDF)
	{
	case pdfMonteCarlo:
		{
			opengl::CPointCloudPtr obj = opengl::CPointCloud::Create();
			obj->setColor(1,0,0);

			obj->setPointSize(2.5);

			const size_t N = m_locationMC.m_particles.size();
			obj->resize(N);

			for (size_t i=0;i<N;i++)
				obj->setPoint(i,
					m_locationMC.m_particles[i].d->x,
					m_locationMC.m_particles[i].d->y,
					m_locationMC.m_particles[i].d->z );

			outObj->insert( obj );
		}
		break;
	case pdfGauss:
		{
			opengl::CEllipsoidPtr obj = opengl::CEllipsoid::Create();

			obj->setPose(m_locationGauss.mean);
			obj->setLineWidth(3);

			CMatrixDouble C = CMatrixDouble(m_locationGauss.cov);
			if (C(2,2)==0) C.setSize(2,2);
			obj->setCovMatrix(C);

			obj->setQuantiles(3);
			obj->enableDrawSolid3D(false);

			obj->setColor(1,0,0, 0.85);
			outObj->insert( obj );
		}
		break;
	case pdfSOG:
		{
			m_locationSOG.getAs3DObject( outObj );
		}
		break;
	default:			THROW_EXCEPTION("ERROR: Invalid 'm_typePDF' value");
	};

	opengl::CTextPtr obj2 = opengl::CText::Create();
	obj2->setString( format("#%d",static_cast<int>(m_ID)) );

	CPoint3D	meanP;
	this->getMean(meanP);
	obj2->setLocation( meanP.x()+0.10, meanP.y()+0.10, meanP.z() );
	outObj->insert( obj2 );

	MRPT_END
}


/*---------------------------------------------------------------
					getAsMatlabDrawCommands
  ---------------------------------------------------------------*/
void CBeacon::getAsMatlabDrawCommands( utils::CStringList &out_Str  ) const
{
	MRPT_START

	out_Str.clear();
	char		auxStr[1000];

	switch (m_typePDF)
	{
	case pdfMonteCarlo:
		{
			// xs=[...];
			// ys=[...];
			// plot(xs,ys,'.','MarkerSize',4);
			size_t			i,N = m_locationMC.m_particles.size();
			std::string		sx,sy;

			sx= "xs=[";
			sy= "ys=[";
			for (i=0;i<N;i++)
			{
				os::sprintf(auxStr,sizeof(auxStr),"%.3f%c",m_locationMC.m_particles[i].d->x, (i==N-1) ? ' ':',' );
				sx=sx+std::string(auxStr);
				os::sprintf(auxStr,sizeof(auxStr),"%.3f%c",m_locationMC.m_particles[i].d->y, (i==N-1) ? ' ':',' );
				sy=sy+std::string(auxStr);
			}
			sx=sx+"];";
			sy=sy+"];";
			out_Str.add(sx);
			out_Str.add(sy);
			out_Str.add(std::string("plot(xs,ys,'k.','MarkerSize',4);"));
		}
		break;
	case pdfGauss:
		{
			// m=[x y];
			// C=[2x2]
			// error_ellipse(C,m,'conf',0.997,'style','k');

			os::sprintf(auxStr,sizeof(auxStr),"m=[%.3f %.3f];",m_locationGauss.mean.x(),m_locationGauss.mean.y());
			out_Str.add(std::string(auxStr));
			os::sprintf(auxStr,sizeof(auxStr),"C=[%e %e;%e %e];",
				m_locationGauss.cov(0,0),m_locationGauss.cov(0,1),
				m_locationGauss.cov(1,0),m_locationGauss.cov(1,1) );
			out_Str.add(std::string(auxStr));

			out_Str.add(std::string("error_ellipse(C,m,'conf',0.997,'style','k');"));
		}
		break;
	case pdfSOG:
		{
			for (CPointPDFSOG::const_iterator it = m_locationSOG.begin(); it!= m_locationSOG.end();++it)
			{
				os::sprintf(auxStr,sizeof(auxStr),"m=[%.3f %.3f];",(it)->val.mean.x(),(it)->val.mean.y());
				out_Str.add(std::string(auxStr));
				os::sprintf(auxStr,sizeof(auxStr),"C=[%e %e;%e %e];",
					(it)->val.cov(0,0),(it)->val.cov(0,1),
					(it)->val.cov(1,0),(it)->val.cov(1,1) );
				out_Str.add(std::string(auxStr));
				out_Str.add(std::string("error_ellipse(C,m,'conf',0.997,'style','k');"));
			}
		}
		break;
	default:			THROW_EXCEPTION("ERROR: Invalid 'm_typePDF' value");
	};

	// The text:
	CPoint3D	meanP;
	getMean(meanP);

	os::sprintf(auxStr,sizeof(auxStr),"text(%f,%f,'#%i');",meanP.x(),meanP.y(), static_cast<int>(m_ID) );
	out_Str.add(std::string(auxStr));

	MRPT_END
}


/*---------------------------------------------------------------
					generateObservationModelDistribution
 Compute the observation model p(z_t|x_t) for a given observation (range value), and return it as an approximate SOG.
*  Note that if the beacon is a SOG itself, the number of gaussian modes will be square.
*  As a speed-up, if a "center point"+"maxDistanceFromCenter" is supplied (maxDistanceFromCenter!=0), those modes farther than this sphere will be discarded.
*  Parameters such as the stdSigma of the sensor are gathered from "myBeaconMap"
*  The result is one "ring" for each Gaussian mode that represent the beacon position in this object.
*  The position of the sensor on the robot is used to shift the resulting densities such as they represent the position of the robot, not the sensor.
*  \sa CBeaconMap::insertionOptions, generateRingSOG

  ---------------------------------------------------------------*/
void CBeacon::generateObservationModelDistribution(
	const float &sensedRange,
	CPointPDFSOG	&outPDF,
	const CBeaconMap *myBeaconMap,
	const CPoint3D	&sensorPntOnRobot,
	const CPoint3D &centerPoint,
	const float &maxDistanceFromCenter) const
{
	MRPT_START


	const CPointPDFSOG *beaconPos=NULL;

	if ( m_typePDF==pdfGauss )
	{
		// Copy the gaussian to the SOG:
		CPointPDFSOG *new_beaconPos= new CPointPDFSOG(1);
		new_beaconPos->push_back( CPointPDFSOG::TGaussianMode() );
		new_beaconPos->get(0).log_w=0;
		new_beaconPos->get(0).val = m_locationGauss;
		beaconPos = new_beaconPos;
	}
	else
	{
		ASSERT_( m_typePDF==pdfSOG )
		beaconPos = static_cast<const CPointPDFSOG*> (&m_locationSOG );
	}

	outPDF.clear();

	for ( CPointPDFSOG::const_iterator it = beaconPos->begin(); it!= beaconPos->end();++it)
	{
		// The center of the ring to be generated
		CPoint3D ringCenter(
			(it)->val.mean.x() - sensorPntOnRobot.x(),
			(it)->val.mean.y() - sensorPntOnRobot.y(),
			(it)->val.mean.z() - sensorPntOnRobot.z()  ) ;

		size_t startIdx = outPDF.size();

		CBeacon::generateRingSOG(
			sensedRange,                    // Sensed range
			outPDF,                         // The ouput (Append all !!)
			myBeaconMap,                    // For params
			ringCenter,                     // The center of the ring to be generated
			&(it)->val.cov,					// The covariance to ADD to each mode, due to the composition of uncertainty
			false,                          // clearPreviousContentsOutPDF
			centerPoint,
			maxDistanceFromCenter           // Directly, do not create too far modes
			);

		// Adjust the weights to the one of "this" mode:
		for (size_t k=startIdx;k<outPDF.size();k++)
			outPDF.get(k).log_w = (it)->log_w;
	}

	if ( m_typePDF==pdfGauss )
		delete beaconPos;

	MRPT_END
}


/*---------------------------------------------------------------
					generateRingSOG
  ---------------------------------------------------------------*/
void CBeacon::generateRingSOG(
	const float 	&R,
	CPointPDFSOG	&outPDF,
	const CBeaconMap *myBeaconMap,
	const CPoint3D	&sensorPnt,
	const CMatrixDouble33   *covarianceCompositionToAdd,
	bool  clearPreviousContentsOutPDF,
	const CPoint3D &centerPoint,
	const float &maxDistanceFromCenter
	)
{
	MRPT_START

	ASSERT_(myBeaconMap)

	// Compute the number of Gaussians:
	const float	minEl = DEG2RAD(myBeaconMap->insertionOptions.minElevation_deg);
	const float	maxEl = DEG2RAD(myBeaconMap->insertionOptions.maxElevation_deg);
	ASSERT_(myBeaconMap->insertionOptions.minElevation_deg<=myBeaconMap->insertionOptions.maxElevation_deg)

	double  el,th,A_ang;
	const float	maxDistBetweenGaussians = myBeaconMap->insertionOptions.SOG_maxDistBetweenGaussians ;  // Meters

	// B: Number of gaussians in each cut to the sphere (XY,XZ,...)
	size_t	B = (size_t)(M_2PIf * R / maxDistBetweenGaussians ) + 1;

	// Assure minimum B (maximum angular increment):
	B = max(B, (size_t)30);

	// B must be even:
	if (B%2) B++;

	A_ang = M_2PI/B;  // Angular increments between modes:

	// The diagonal basic covariance matrix.
	//  (0,0) is the variance in the direction "in->out" of the sphere
	//  (1,1),(2,2) is the var. in the two directions tangent to the sphere
	CMatrixDouble33	S;
	S(0,0) = square( myBeaconMap->likelihoodOptions.rangeStd );
	S(1,1) = S(2,2) = square( A_ang*R / myBeaconMap->insertionOptions.SOG_separationConstant  );  //4.0f * sensedRange * S(0,0);

	CPoint3D	dir;

	// Create the SOG:
	size_t modeIdx;
	if (clearPreviousContentsOutPDF)
	{
		// Overwrite modes:
		modeIdx = 0;
		outPDF.resize(B*B);
	}
	else
	{
		// Append modes:
		modeIdx = outPDF.size(); // Start here
		outPDF.resize(outPDF.size()+B*B);
	}

	size_t idxEl, idxTh;  // idxEl:[0,B/2+1]

	for (idxEl=0;idxEl<=(1+B/2);idxEl++)
	{
		el = minEl + idxEl*A_ang;
		if (el>(maxEl+0.5*A_ang)) continue;

		size_t nThSteps = B;
		// Except if we are in the top/bottom of the sphere:
		if (fabs(cos(el))<1e-4)
		{
			nThSteps=1;
		}

		for (idxTh=0;idxTh<nThSteps;idxTh++)
		{
			th = idxTh*A_ang;

			// Compute the mean of the new Gaussian:
			dir.x( (sensorPnt.x() + R*cos(th)*cos(el)) );
			dir.y( (sensorPnt.y() + R*sin(th)*cos(el)) );
			dir.z( (sensorPnt.z() + R*sin(el)) );

			// If we are provided a radius for not creating modes out of it, check it:
			bool reallyCreateIt = true;
			if (maxDistanceFromCenter>0)
				reallyCreateIt = dir.distanceTo( centerPoint ) < maxDistanceFromCenter;

			if (reallyCreateIt)
			{
				// All have equal log-weights:
				outPDF.get(modeIdx).log_w = 0;

				// The mean:
				outPDF.get(modeIdx).val.mean = dir;

				// Compute the covariance:
				dir = dir - sensorPnt;
				CMatrixDouble33	H = CMatrixDouble33( math::generateAxisBaseFromDirection( dir.x(),dir.y(),dir.z() ));	// 3 perpendicular & normalized vectors.

				H.multiply_HCHt(
					S,
					outPDF.get(modeIdx).val.cov ); // out = H * S * ~H;
				if (minEl==maxEl)
				{   // We are in 2D:
					// 3rd column/row = 0
					CMatrixDouble33 &C33 = outPDF.get(modeIdx).val.cov;
					C33.get_unsafe(0,2)=C33.get_unsafe(2,0)=
					C33.get_unsafe(1,2)=C33.get_unsafe(2,1)=
					C33.get_unsafe(2,2)=0;
				}

				// Add covariance for uncertainty composition?
				if (covarianceCompositionToAdd)
					outPDF.get(modeIdx).val.cov += *covarianceCompositionToAdd;

				// One more mode is used:
				modeIdx++;

			} // end if reallyCreateIt

		} // end for idxTh
	} // end for idxEl

	// resize to the number of really used modes:
	outPDF.resize(modeIdx);

	MRPT_END
}
