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
#ifndef CBeacon_H
#define CBeacon_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/system/os.h>
#include <mrpt/utils/CStringList.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPointPDFParticles.h>
#include <mrpt/poses/CPointPDFGaussian.h>
#include <mrpt/poses/CPointPDFSOG.h>

#include <mrpt/opengl/CSetOfObjects.h>

#include <mrpt/maps/link_pragmas.h>


namespace mrpt
{
namespace slam
{
	using namespace mrpt::poses;
	using namespace mrpt::utils;

	class CBeaconMap;
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CBeacon, mrpt::utils::CSerializable, MAPS_IMPEXP )

	/** The class for storing individual "beacon landmarks" under a variety of 3D position PDF distributions.
	  *  This class is used for storage within the class CBeaconMap.
	  *  The class implements the same methods than the interface "CPointPDF", and invoking them actually becomes
	  *   a mapping into the methods of the current PDF representation of the beacon, selectable by means of "m_typePDF"
	  * \sa CBeaconMap, CPointPDFSOG
	  * \ingroup mrpt_maps_grp
	  */
	class MAPS_IMPEXP CBeacon : public CPointPDF
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CBeacon )

	public:
		/** The type for the IDs of landmarks.
		  */
		typedef	int64_t TBeaconID;

		/** See m_typePDF
		  */
		enum TTypePDF { pdfMonteCarlo = 0, pdfGauss, pdfSOG };

		/** Which one of the different 3D point PDF is currently used in this object: montecarlo, gaussian, or a sum of gaussians.
		  * \sa m_location
		  */
		TTypePDF	m_typePDF;

		/** The individual PDF, if m_typePDF=pdfMonteCarlo (publicly accesible for ease of use, but the CPointPDF interface is also implemented in CBeacon).
		  */
		CPointPDFParticles	m_locationMC;

		/** The individual PDF, if m_typePDF=pdfGauss (publicly accesible for ease of use, but the CPointPDF interface is also implemented in CBeacon).
		  */
		CPointPDFGaussian	m_locationGauss;

		/** The individual PDF, if m_typePDF=pdfSOG (publicly accesible for ease of use, but the CPointPDF interface is also implemented in CBeacon).
		  */
		CPointPDFSOG		m_locationSOG;

		/** An ID for the landmark (see details next...)
		  *  This ID was introduced in the version 3 of this class (21/NOV/2006), and its aim is
		  *  to provide a way for easily establishing correspondences between landmarks detected
		  *  in sequential image frames. Thus, the management of this field should be:
		  *		- In 'servers' (classes/modules/... that detect landmarks from images): A different ID must be assigned to every landmark (e.g. a sequential counter), BUT only in the case of being sure of the correspondence of one landmark with another one in the past (e.g. tracking).
		  *		- In 'clients': This field can be ignored, but if it is used, the advantage is solving the correspondence between landmarks detected in consequentive instants of time: Two landmarks with the same ID <b>correspond</b> to the same physical feature, BUT it should not be expected the inverse to be always true.
		  *
		  * Note that this field is never fill out automatically, it must be set by the programmer if used.
		  */
		TBeaconID			m_ID;

		/** Default constructor
		  */
		CBeacon();

		/** Virtual destructor
		  */
		virtual ~CBeacon();

		 /** Returns an estimate of the point, (the mean, or mathematical expectation of the PDF).
		   * \sa getCovariance
		   */
		void getMean(CPoint3D &mean_point) const;

		/** Returns an estimate of the point covariance matrix (3x3 cov matrix) and the mean, both at once.
		  * \sa getMean
		  */
		void getCovarianceAndMean(CMatrixDouble33 &cov,CPoint3D &mean_point) const;

		/** Copy operator, translating if necesary (for example, between particles and gaussian representations)
		  */
		void  copyFrom(const CPointPDF &o);

		/** Save PDF's particles to a text file. See derived classes for more information about the format of generated files.
		 */
		void  saveToTextFile(const std::string &file) const;

		/** This can be used to convert a PDF from local coordinates to global, providing the point (newReferenceBase) from which
		  *   "to project" the current pdf. Result PDF substituted the currently stored one in the object.
		  */
		void  changeCoordinatesReference( const CPose3D &newReferenceBase );

		/** Saves a 3D representation of the beacon into a given OpenGL scene
		  */
		void  getAs3DObject( mrpt::opengl::CSetOfObjectsPtr	&outObj ) const;

		/** Gets a set of MATLAB commands which draw the current state of the beacon:
		  */
		void getAsMatlabDrawCommands( utils::CStringList &out_Str ) const;

		/** Draw a sample from the pdf.
		  */
		void drawSingleSample(CPoint3D &outSample) const;

		/** Bayesian fusion of two point distributions (product of two distributions->new distribution), then save the result in this object (WARNING: See implementing classes to see classes that can and cannot be mixtured!)
		  * \param p1 The first distribution to fuse
		  * \param p2 The second distribution to fuse
		  * \param minMahalanobisDistToDrop If set to different of 0, the result of very separate Gaussian modes (that will result in negligible components) in SOGs will be dropped to reduce the number of modes in the output.
		  */
		void  bayesianFusion(const  CPointPDF &p1,const  CPointPDF &p2, const double &minMahalanobisDistToDrop = 0);


		/** Compute the observation model p(z_t|x_t) for a given observation (range value), and return it as an approximate SOG.
		  *  Note that if the beacon is a SOG itself, the number of gaussian modes will be square.
		  *  As a speed-up, if a "center point"+"maxDistanceFromCenter" is supplied (maxDistanceFromCenter!=0), those modes farther than this sphere will be discarded.
		  *  Parameters such as the stdSigma of the sensor are gathered from "myBeaconMap"
		  *  The result is one "ring" for each Gaussian mode that represent the beacon position in this object.
		  *  The position of the sensor on the robot is used to shift the resulting densities such as they represent the position of the robot, not the sensor.
		  *  \sa CBeaconMap::insertionOptions, generateRingSOG
		  */
		void generateObservationModelDistribution(
			const float &sensedRange,
			CPointPDFSOG	&outPDF,
			const CBeaconMap *myBeaconMap,
			const CPoint3D	&sensorPntOnRobot,
			const CPoint3D &centerPoint = CPoint3D(0,0,0),
			const float &maxDistanceFromCenter = 0
			) const;

		/** This static method returns a SOG with ring-shape (or as a 3D sphere) that can be used to initialize a beacon if observed the first time.
		  *  sensorPnt is the center of the ring/sphere, i.e. the absolute position of the range sensor.
		  *  If clearPreviousContentsOutPDF=false, the SOG modes will be added to the current contents of outPDF
		  *  If the 3x3 matrix covarianceCompositionToAdd is provided, it will be add to every Gaussian (to model the composition of uncertainty).
		  * \sa generateObservationModelDistribution
		  */
		static void generateRingSOG(
			const float &sensedRange,
			CPointPDFSOG	&outPDF,
			const CBeaconMap *myBeaconMap,
			const CPoint3D	&sensorPnt,
			const CMatrixDouble33   *covarianceCompositionToAdd = NULL,
			bool  clearPreviousContentsOutPDF = true,
			const CPoint3D &centerPoint = CPoint3D(0,0,0),
			const float &maxDistanceFromCenter = 0
			);


	}; // End of class definition


	} // End of namespace
} // End of namespace

#endif
