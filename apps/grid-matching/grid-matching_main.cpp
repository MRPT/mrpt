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

#include <mrpt/slam.h>
#include <mrpt/gui.h>
#include <mrpt/base.h>
#include <mrpt/otherlibs/tclap/CmdLine.h>


using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace mrpt::gui;
using namespace mrpt::random;
using namespace std;



//#define SAVE_SOG_GRID

bool 	SAVE_SOG_3DSCENE;
bool	SAVE_SOG_ALL_MAPS_OVERLAP_HYPOTHESES;
bool	SAVE_CORR_AND_NONCORR_DISTS;
bool	IS_VERBOSE;
bool	NOSAVE=false;
bool	SKIP_ICP_STAGE=false;

string 	SAVE_ICP_GOODNESS_FIL;

// Mode of operation
bool is_match, is_detect_test;

string      	RESULTS_DIR("GRID-MATCHING_RESULTS");
string 			fil_grid1, fil_grid2;
string 			OUTPUT_FIL;
string 			CONFIG_FIL;


double STD_NOISE_XY=0, STD_NOISE_PHI=0;
double STD_NOISE_LASER=0;
double GT_Ax,GT_Ay, GT_Aphi;

bool	NOISE_IN_LASER		= false;
bool	NOISE_IN_POSE		= false;

unsigned int N_ITERS		= 1;

#define START_NOISE_STD			0.00f
#define END_NOISE_STD			0.00f
#define NOISE_STEPS				0.01f


/* ------------------------------------------------------------------------
					do_grid_align
   ------------------------------------------------------------------------ */
void do_grid_align()
{
	CImage				img1,img2;

	randomGenerator.randomize();

	// Grid or simplemaps?
	if (is_match || (!is_detect_test && SAVE_CORR_AND_NONCORR_DISTS))
	{
		string ext1 = mrpt::system::extractFileExtension(fil_grid1,true);
		string ext2 = mrpt::system::extractFileExtension(fil_grid2,true);
		if (ext1!=ext2 || ext1!="simplemap")
		{
			cerr << "Both file extensions must be 'simplemap'" << endl;
			return;
		}

		cout << "Map1: " << fil_grid1 << endl;
		cout << "Map2: " << fil_grid2 << endl;
	}
	if (is_detect_test)
	{
		string ext1 = mrpt::system::extractFileExtension(fil_grid1,true);
		if (ext1!="simplemap")
		{
			cerr << "The file extension must be 'simplemap'" << endl;
			return;
		}
		cout << "Map: " << fil_grid1 << endl;
	}

	CMultiMetricMap			the_map1,the_map2;

	TSetOfMetricMapInitializers	map_inits;
	{
		TMetricMapInitializer	map_init;
		map_init.metricMapClassType = CLASS_ID( COccupancyGridMap2D );
		map_init.occupancyGridMap2D_options.resolution = 0.05;
		//map_init.m_disableSaveAs3DObject = true;

		map_init.occupancyGridMap2D_options.insertionOpts.maxOccupancyUpdateCertainty = 0.8;
		map_init.occupancyGridMap2D_options.insertionOpts.maxDistanceInsertion = 30;

		map_inits.push_back(map_init);
	}

	if (!SKIP_ICP_STAGE)
	{
		TMetricMapInitializer	map_init;
		map_init.metricMapClassType = CLASS_ID( CSimplePointsMap );
		map_init.pointsMapOptions_options.insertionOpts.minDistBetweenLaserPoints = 0.10;
		map_inits.push_back(map_init);
	}


	mrpt::system::deleteFiles(format("%s/*.*",RESULTS_DIR.c_str()));
	mrpt::system::createDirectory(RESULTS_DIR);

	CFileOutputStream f_log( RESULTS_DIR + string("/log.txt") );
	f_log.printf("%% std_noise_xy std_noise_phi std_noise_laser  covDet:mean&std stdPhi:mean&std error2D:mean&std  errorPhi:mean&phi bruteErr2D:mean&std brutErrPhi:mean&std GT_likelihood #GTcorrFound:mean&std \n");
	f_log.printf("%% -----------------------------------------\n");

	CFileStream			f_out_log;
	if (!OUTPUT_FIL.empty())
		f_out_log.open(OUTPUT_FIL, fomWrite); //, fomAppend );

	CGridMapAligner					gridAlign;
	CGridMapAligner::TReturnInfo	info;
	float							tim;

//	gridAlign.options.methodSelection = CGridMapAligner::amModifiedRANSAC;
//	gridAlign.options.methodSelection = CGridMapAligner::amRobustMatch;
//	gridAlign.options.methodSelection = CGridMapAligner::amCorrelation;

	if (!CONFIG_FIL.empty())
	{
		// Load options for the grid-matching algorithm:
		CConfigFile cfg(CONFIG_FIL);
		gridAlign.options.loadFromConfigFile( cfg, "grid-match" );

		// load alternative config. for the metric maps:
		if (cfg.sectionExists("metric_maps"))
		{
			map_inits.loadFromConfigFile(cfg,"metric_maps");
		}
	}

	// Dump params:
	if (IS_VERBOSE)
	{
		gridAlign.options.dumpToConsole();
		map_inits.dumpToConsole();
	}

	// Create the map with a points & grid-map within:
	the_map1.setListOfMaps( &map_inits );
	the_map2.setListOfMaps( &map_inits );

	ASSERT_( the_map1.m_gridMaps.size()>=1 );
	ASSERT_( the_map2.m_gridMaps.size()>=1 );

	COccupancyGridMap2DPtr	grid1 = the_map1.m_gridMaps[0];
	COccupancyGridMap2DPtr	grid2 = the_map2.m_gridMaps[0];

	// ---------------------------------------------
	//				Options: RANSAC
	// ---------------------------------------------
	gridAlign.options.ransac_SOG_sigma_m		= grid1->getResolution() * 2;

	CSimpleMap 	map1,map2, map2noisy;

	// Load maps:
	CFileGZInputStream(fil_grid1) >> map1;

	// If it's detect_test only load one map:
	if (is_match || mrpt::system::fileExists(fil_grid2) )
	{
		CFileGZInputStream(fil_grid2) >> map2;
	}

	// Generate the_map1 now:
	the_map1.loadFromProbabilisticPosesAndObservations( map1 );

	size_t N1 = max(40,(int)round( grid1->getArea() * gridAlign.options.featsPerSquareMeter));

	COccupancyGridMapFeatureExtractor	gridfeatextract;


	CLandmarksMap							lm1;

	gridfeatextract.extractFeatures(*grid1, lm1, N1, gridAlign.options.feature_descriptor, gridAlign.options.feature_detector_options );

	if (!NOSAVE && (is_match || is_detect_test))		// If it's only a SAVE_CORR_AND_NONCORR_DISTS test, save time
	{
		grid1->saveAsBitmapFile(format("%s/map1.png",RESULTS_DIR.c_str()));
		grid1->saveAsBitmapFileWithLandmarks( format("%s/map1_LM.png",RESULTS_DIR.c_str()) ,&lm1, true);
		CImage	img;
		grid1->getAsImageFiltered(img);
		img.saveToFile(format("%s/map1_filt.png",RESULTS_DIR.c_str()));
	}

	// ADDITIVE WHITE GAUSSIAN NOISE:
	// ----------------------------------------
	//for (noiseStd=START_NOISE_STD;noiseStd<=END_NOISE_STD;noiseStd+=NOISE_STEPS)
	{
		vector_float	stats_covDet, stats_stdPhi;
		vector_float	stats_errorXY, stats_errorPhi;
		vector_float	stats_bruteErrorXY, stats_bruteErrorPhi;
		vector_float	stats_GT_likelihood;
		vector_uint		overallGTcorrsFound;

		for (unsigned int iter=0;iter<N_ITERS;iter++)
		{
			// Insert noise into the laser scans of map2:
			// ------------------------------------------------
			map2noisy = is_detect_test ? map1 : map2;

			for (unsigned int q=0;q<map2noisy.size();q++)
			{
				CPose3DPDFPtr 		PDF;
				CSensoryFramePtr 	SF;

				map2noisy.get(q,PDF,SF);

				// If it's detect_test, translate the map2 by a fixed, known quantity:
				if (is_detect_test)
				{
					CPose2D  gt(GT_Ax,GT_Ay, GT_Aphi);
					gt = -gt;
					PDF->changeCoordinatesReference(  CPose3D( gt ) );
				}

				if (NOISE_IN_LASER)
				{
					CObservation2DRangeScanPtr obs = SF->getObservationByClass<CObservation2DRangeScan>();
					if (obs)
					{
						for (unsigned int k=0;k<obs->scan.size();k++)
						{
							obs->scan[k] += randomGenerator.drawGaussian1D(0,STD_NOISE_LASER);
							if (obs->scan[k]<0) obs->scan[k] = 0;
						}
					}
				} // end of NOISE_IN_LASER

				if (NOISE_IN_POSE)
				{
					CPosePDFGaussianPtr newPDF = CPosePDFGaussian::Create();
					newPDF->copyFrom(*PDF);

					// Change the pose:
					newPDF->mean.x_incr( randomGenerator.drawGaussian1D(0,STD_NOISE_XY) );
					newPDF->mean.y_incr( randomGenerator.drawGaussian1D(0,STD_NOISE_XY) );
					newPDF->mean.phi_incr( randomGenerator.drawGaussian1D(0,DEG2RAD(STD_NOISE_PHI)) );
					newPDF->mean.normalizePhi();

					// Change into the map:
					map2noisy.set( q, newPDF, CSensoryFramePtr() );

				} // end of NOISE_IN_POSE
			}

			the_map2.loadFromProbabilisticPosesAndObservations( map2noisy );

			//grid2->resetFeaturesCache();

			size_t N2 = max(40,(int)round( grid2->getArea() * gridAlign.options.featsPerSquareMeter));

			CLandmarksMap							lm2;
			gridfeatextract.extractFeatures(*grid2,lm2, N2, gridAlign.options.feature_descriptor, gridAlign.options.feature_detector_options );

			if (!NOSAVE && (is_match || is_detect_test))		// If it's only a SAVE_CORR_AND_NONCORR_DISTS test, save time
			{
				// Save maps:
				grid2->saveAsBitmapFile( format("%s/map2_noise_%f.png",RESULTS_DIR.c_str(),STD_NOISE_XY) );
				grid2->saveAsBitmapFileWithLandmarks( format("%s/map2_LM_noise_%f.png",RESULTS_DIR.c_str(),STD_NOISE_XY) ,&lm2, true);
				CImage	img;
				grid2->getAsImageFiltered(img);
				img.saveToFile(format("%s/map2_filt_noise_%f.png",RESULTS_DIR.c_str(),STD_NOISE_XY));
			}

			// Only if the case of "save-corr-dists" we can do NOT align the maps, since we're not interested in that
			if (is_match || is_detect_test)
			{
				// --------------------------
				//        DO ALIGN
				// --------------------------
				CPosePDFPtr	parts = gridAlign.Align(
					&the_map1, &the_map2,
					CPose2D(0,0,0),
					&tim,
					&info );

				// STATS:
				CPose2D			estimateMean;
				CMatrixDouble33	estimateCOV;

				// Get the mean, or the best Gassian mean in the case of a SOG:
				if (IS_CLASS(parts,CPosePDFSOG))
				{
					CPosePDFSOGPtr pdf_SOG= CPosePDFSOGPtr( parts );
					pdf_SOG->getMostLikelyCovarianceAndMean(estimateCOV,estimateMean);
				}
				else
				{
					parts->getCovarianceAndMean(estimateCOV,estimateMean);
				}

				float		stdPhi = sqrt(estimateCOV(2,2));

				CMatrixDouble22	estimateCOV22 = estimateCOV.block(0,0,2,2);
				float		stdXY  = sqrt(estimateCOV22.det());

				float		Axy			= estimateMean.distance2DTo(GT_Ax,GT_Ay);
				float		Aphi		= fabs( math::wrapToPi(estimateMean.phi() - (float)DEG2RAD( GT_Aphi )));
				float		AxyBrute	= info.noRobustEstimation.distance2DTo(GT_Ax,GT_Ay);
				float		AphiBrute	= fabs( math::wrapToPi(info.noRobustEstimation.phi() - (float)DEG2RAD( GT_Aphi )));

				printf("Done in %.03fms\n", 1000.0f*tim);

				std::cout << "Mean pose:\n\t " << estimateMean << "\n";
				std::cout << "Estimate covariance::\n" << estimateCOV << "\n";

				// Save particles:
				if (IS_CLASS(parts, CPosePDFParticles))
				{
					CPosePDFParticlesPtr partsPdf = CPosePDFParticlesPtr( parts );

					partsPdf->saveToTextFile( format("%s/particles_noise_%.03f.txt", RESULTS_DIR.c_str(), STD_NOISE_XY) );

					printf("Goodness: %.03f%%\n", 100 * info.goodness);
					std::cout << partsPdf->particlesCount() << " particles\n";
					std::cout << "Covariance:\n\t " << estimateCOV << "\n";
				}
				else
				if (IS_CLASS(parts,CPosePDFSOG))
				{
					CPosePDFSOGPtr pdf_SOG= CPosePDFSOGPtr( parts );
					printf("SoG has %u modes\n",(unsigned int)pdf_SOG->size());

					pdf_SOG->normalizeWeights();
					//pdf_SOG->saveToTextFile("_debug_SoG.txt");

					stats_GT_likelihood.push_back( (float)pdf_SOG->evaluatePDF(CPose2D(GT_Ax,GT_Ay,DEG2RAD(GT_Aphi)),true) );


					if (f_out_log.fileOpenCorrectly())
					{
						f_out_log.printf( "%% SOG_log_w   x   y   phi  \n");
						for (size_t m=0;m<pdf_SOG->size();m++)
							f_out_log.printf( "    %e     %f %f %f\n",
								pdf_SOG->get(m).log_w,
								pdf_SOG->get(m).mean.x(),
								pdf_SOG->get(m).mean.y(),
								pdf_SOG->get(m).mean.phi() );
					}


					if (SAVE_SOG_3DSCENE)
					{
						COpenGLScene				scene3D;
						opengl::CSetOfObjectsPtr	thePDF3D = opengl::CSetOfObjects::Create();
						pdf_SOG->getAs3DObject( thePDF3D );
						opengl::CGridPlaneXYPtr	gridXY	= opengl::CGridPlaneXY::Create(-10,10,-10,10,0,1);
						scene3D.insert( gridXY	);
						scene3D.insert( thePDF3D );
						CFileGZOutputStream("_out_SoG.3Dscene") << scene3D;
					}

					if (!SAVE_ICP_GOODNESS_FIL.empty())
					{
						vectorToTextFile( info.icp_goodness_all_sog_modes, SAVE_ICP_GOODNESS_FIL, true ); // append & as column
					}

					// Save all the maps overlap hypotheses:
					if (SAVE_SOG_ALL_MAPS_OVERLAP_HYPOTHESES)
					{
						CPosePDFSOG::iterator	it;
						size_t	nNode;
						CImage	imgGrid1, imgCanvas;
						grid1->resizeGrid( min(grid1->getXMin(),-60.0f), max(grid1->getXMax(),60.0f), min(-40.0f,grid1->getYMin()), max(30.0f,grid1->getYMax()) );
						grid1->getAsImage( imgGrid1, true );
						int			imgGrid1LY = imgGrid1.getHeight();
						const CPose2D	nullPose(0,0,0);

						const TPoint2D	q1( grid1->getXMin(), grid1->getYMin() );
						const TPoint2D	q2( grid1->getXMin(), grid1->getYMax() );
						const TPoint2D	q3( grid1->getXMax(), grid1->getYMax() );
						const TPoint2D	q4( grid1->getXMax(), grid1->getYMin() );

						const TPoint2D	p1( grid2->getXMin(), grid2->getYMin() );
						const TPoint2D	p2( grid2->getXMin(), grid2->getYMax() );
						const TPoint2D	p3( grid2->getXMax(), grid2->getYMax() );
						const TPoint2D	p4( grid2->getXMax(), grid2->getYMin() );
						for (nNode=0,it = pdf_SOG->begin();it!=pdf_SOG->end();it++,nNode++)
						{
							CPose2D		x = it->mean;
							const TPoint2D	pp1( x + p1 );
							const TPoint2D	pp2( x + p2 );
							const TPoint2D	pp3( x + p3 );
							const TPoint2D	pp4( x + p4 );

							// Draw the background = the_map1:
							imgCanvas = imgGrid1;

							// Draw the overlaped the_map2:
							imgCanvas.line( grid1->x2idx(pp1.x),imgGrid1LY-1-grid1->y2idx(pp1.y),grid1->x2idx(pp2.x),imgGrid1LY-1-grid1->y2idx(pp2.y),TColor::black);
							imgCanvas.line( grid1->x2idx(pp2.x),imgGrid1LY-1-grid1->y2idx(pp2.y),grid1->x2idx(pp3.x),imgGrid1LY-1-grid1->y2idx(pp3.y),TColor::black);
							imgCanvas.line( grid1->x2idx(pp3.x),imgGrid1LY-1-grid1->y2idx(pp3.y),grid1->x2idx(pp4.x),imgGrid1LY-1-grid1->y2idx(pp4.y),TColor::black);
							imgCanvas.line( grid1->x2idx(pp4.x),imgGrid1LY-1-grid1->y2idx(pp4.y),grid1->x2idx(pp1.x),imgGrid1LY-1-grid1->y2idx(pp1.y),TColor::black);

							imgCanvas.saveToFile( format("%s/_OVERLAP_MAPS_SOG_MODE_%04u.png",RESULTS_DIR.c_str(), (unsigned int)nNode  ) );

							// Save as 3D scene:
							COpenGLScene	scene;
							CPointsMap::COLOR_3DSCENE_R = 0;
							CPointsMap::COLOR_3DSCENE_G = 0;
							CPointsMap::COLOR_3DSCENE_B = 1;
							CSetOfObjectsPtr obj1 = CSetOfObjects::Create();
							the_map1.getAs3DObject( obj1 );

							CPointsMap::COLOR_3DSCENE_R = 1;
							CPointsMap::COLOR_3DSCENE_G = 0;
							CPointsMap::COLOR_3DSCENE_B = 0;
							CSetOfObjectsPtr obj2 = CSetOfObjects::Create();
							the_map2.getAs3DObject( obj2 );

							obj2->setPose(x);

							scene.insert(obj1);
							scene.insert(obj2);

							// Add also the borders of the maps:
							CSetOfLinesPtr	lines = CSetOfLines::Create();
							lines->setLineWidth(3);
							lines->setColor(0,0,1);

							lines->appendLine(q1.x,q1.y,0, q2.x,q2.y,0 );
							lines->appendLine(q2.x,q2.y,0, q3.x,q3.y,0 );
							lines->appendLine(q3.x,q3.y,0, q4.x,q4.y,0 );
							lines->appendLine(q4.x,q4.y,0, q1.x,q1.y,0 );

							lines->appendLine(pp1.x,pp1.y,0, pp2.x,pp2.y,0 );
							lines->appendLine(pp2.x,pp2.y,0, pp3.x,pp3.y,0 );
							lines->appendLine(pp3.x,pp3.y,0, pp4.x,pp4.y,0 );
							lines->appendLine(pp4.x,pp4.y,0, pp1.x,pp1.y,0 );

							scene.insert(lines);

							CFileGZOutputStream( format("%s/_OVERLAP_MAPS_SOG_MODE_%04u.3Dscene",RESULTS_DIR.c_str(), (unsigned int)nNode)) << scene;

							// Save also as EMF:
							/*{
								TMatchingPairList	corrs;
								// How to get corrs!?

								CEnhancedMetaFile::LINUX_IMG_WIDTH = grid1->getSizeX() + grid2->getSizeX() + 50;
								CEnhancedMetaFile::LINUX_IMG_HEIGHT = max(grid1->getSizeY(),grid2->getSizeY()) + 50;
								COccupancyGridMap2D::saveAsEMFTwoMapsWithCorrespondences(
									format("%s/_OVERLAP_MAPS_SOG_MODE_%04u_corrs.emf",RESULTS_DIR.c_str(), (unsigned int)nNode),
									grid1,
									grid2,
									corrs );
							}*/

						}


					} // end SAVE_SOG_ALL

	#ifdef SAVE_SOG_GRID
					// Save grid evaluation of the SOG:
					CMatrix		gridLimits(1,4);
					gridLimits(0,0) = estimateMean.x - 0.10f;
					gridLimits(0,1) = estimateMean.x + 0.10f,
					gridLimits(0,2) = estimateMean.y - 0.10f;
					gridLimits(0,3) = estimateMean.y + 0.10f;
					gridLimits.saveToTextFile( format("%s/SOG_grid_limits_noise_%f.txt",RESULTS_DIR.c_str() ,STD_NOISE_XY) );

					CMatrixD	evalGrid;
					pdf_SOG->evaluatePDFInArea(
						gridLimits(0,0), gridLimits(0,1),
						gridLimits(0,2), gridLimits(0,3),
						0.002f,
						0,
						evalGrid,
						true			// Sum over all phis
						);

					evalGrid.saveToTextFile( format("%s/SOG_grid_noise_%f.txt",RESULTS_DIR.c_str() ,STD_NOISE_XY) );
	#endif
				} // end if is SOG

				// STATS:
				stats_covDet.push_back( stdXY);
				stats_stdPhi.push_back(stdPhi);
				stats_errorXY.push_back(Axy);
				stats_errorPhi.push_back(Aphi);
				stats_bruteErrorXY.push_back(AxyBrute);
				stats_bruteErrorPhi.push_back(AphiBrute);

			} // end if we really do align


			// Save the descriptor distances for corresponding and non-corresponding features
			//  (only known if we are in the "is_detect_test" mode!)
			if (SAVE_CORR_AND_NONCORR_DISTS)
			{
				cout << "Generating coor & no.corr distances..." << endl;

				const bool SAVE_ALSO_COORS_DEBUG_MAPS = false;

				CLandmarksMapPtr lm1 = info.landmarks_map1;
				CLandmarksMapPtr lm2 = info.landmarks_map2;

				// only for the case of non "--match":
				if (!lm1 && !lm2)
				{
					lm1 = CLandmarksMap::Create();
					lm2 = CLandmarksMap::Create();

					gridfeatextract.extractFeatures(*grid1,*lm1, N1, gridAlign.options.feature_descriptor, gridAlign.options.feature_detector_options );
					gridfeatextract.extractFeatures(*grid2,*lm2, N2, gridAlign.options.feature_descriptor, gridAlign.options.feature_detector_options );
				}

				ASSERT_(lm1 && lm2);

				// GT transformation:
				const CPose2D  GT_Ap( GT_Ax, GT_Ay, GT_Aphi);
				TMatchingPairList	gt_corrs;

				CFileOutputStream	fout_CORR("GT_EXP_CORR.txt", true);
				CFileOutputStream	fout_NCORR("GT_EXP_NCORR.txt", true);

				// Compute the distances:
				for (size_t i1=0;i1<lm1->landmarks.size();i1++)
				{
					vector_double   D(lm2->landmarks.size());   // Distances in descriptor space
					vector_double   dErrs(lm2->landmarks.size()); // Distances in (x,y)
					size_t i2;
					//size_t gt_corr_of_i1=0;

					const CLandmark *l1 = lm1->landmarks.get( i1 );

					for (i2=0;i2<lm2->landmarks.size();i2++)
					{
						CLandmark *l2 = lm2->landmarks.get( i2 );

						CPoint2D  P1 = CPoint2D( l1->pose_mean );
						CPoint2D  P2 = GT_Ap + CPoint2D( l2->pose_mean );

						const double dErr = P1.distanceTo(P2);
						dErrs[i2]=dErr;

						ASSERT_(!l1->features.empty() && l1->features[0].present())
						ASSERT_(!l2->features.empty() && l2->features[0].present())

						D[i2] = l1->features[0]->descriptorDistanceTo( *l2->features[0] );
					}

					size_t best_match=0;
					dErrs.minimum(&best_match);
					double MIN_DESCR_DIST = mrpt::math::minimum(D);
					if (dErrs[best_match]<0.20)
					{
						CLandmark *l2 = lm2->landmarks.get( best_match );
						gt_corrs.push_back( TMatchingPair(
							i1,best_match,
							l1->pose_mean.x,l1->pose_mean.y,l1->pose_mean.z,
							l2->pose_mean.x,l2->pose_mean.y,l2->pose_mean.z ) );
					}
					else best_match=(unsigned int)(-1);


					//double m,s; // Mean & Std:
					//mrpt::math::meanAndStd(D,m,s);
					//D = Abs( (D-m)/s );
					// The output files:
					for (i2=0;i2<lm2->landmarks.size();i2++)
					{
						if (i2==best_match)
								fout_CORR.printf("%f %f\n",D[i2], D[i2]-MIN_DESCR_DIST);
						else	fout_NCORR.printf("%f %f\n",D[i2], D[i2]-MIN_DESCR_DIST);
					}

					//mrpt::system::pause();
				} // end for i1

				cout << "GT corrs found: " << gt_corrs.size() << endl;
				overallGTcorrsFound.push_back( gt_corrs.size() );
				if (SAVE_ALSO_COORS_DEBUG_MAPS)
				{
					COccupancyGridMap2D::saveAsBitmapTwoMapsWithCorrespondences(
						"GT_corrs.png",
						grid1.pointer(),
						grid2.pointer(),
						gt_corrs );
					mrpt::system::pause();
				}

			}


/*			printf("\n------------------------------------------------------------------------\n");
			printf("Noise:%f -> ratio:%.02f%%  ERRs:(%.03f,%.03f) STDs:(%.03f %.03f)\n",
				STD_NOISE_XY,ratio*100,Axy,Aphi,stdXY,stdPhi );
			printf("------------------------------------------------------------------------\n\n");*/


		} // end iter

		f_log.printf("%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %e %f %f\n",
			STD_NOISE_XY, STD_NOISE_PHI,
			STD_NOISE_LASER,
			math::mean(stats_covDet), math::stddev(stats_covDet),
			math::mean(stats_stdPhi), math::stddev(stats_stdPhi),
			math::mean(stats_errorXY), math::stddev(stats_errorXY),
			math::mean(stats_errorPhi), math::stddev(stats_errorPhi),
			math::mean(stats_bruteErrorXY), math::stddev(stats_bruteErrorXY),
			math::mean(stats_bruteErrorPhi), math::stddev(stats_bruteErrorPhi),
			math::mean(stats_GT_likelihood),
			math::mean(overallGTcorrsFound), math::stddev(overallGTcorrsFound)
			);


	} // For each noise

}


// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char **argv)
{
    try
    {
		// Declare the supported options.
		TCLAP::CmdLine cmd("grid-matching", ' ', MRPT_getVersion().c_str());

		TCLAP::SwitchArg arg_match("m","match","Operation: match two maps",cmd, false);
		TCLAP::SwitchArg arg_detect("d","detect-test","Operation: Quality of match with one map",cmd, false);

		TCLAP::ValueArg<std::string> arg_filgrid1("1","map1","Map #1 to align (*.simplemap)",true,"","map1.simplemap",cmd);
		TCLAP::ValueArg<std::string> arg_filgrid2("2","map2","Map #2 to align (*.simplemap)",false,"","map2.simplemap",cmd);

		TCLAP::ValueArg<std::string> arg_out("o","out","Output file for the results",false,"gridmatching_out.txt","result_outfile",cmd);
		TCLAP::ValueArg<std::string> arg_config("c","config","Optional config. file with more params",false,"","config.ini",cmd);

		TCLAP::SwitchArg arg_savesog3d("3","save-sog-3d","Save a 3D view of all the SOG modes",cmd, false);
		TCLAP::SwitchArg arg_savesogall("a","save-sog-all","Save all the map overlaps",cmd, false);
		TCLAP::SwitchArg arg_savecorrdists("t","save-corr-dists","Save corr & non-corr distances",cmd, false);
		TCLAP::ValueArg<std::string> arg_icpgoodness("i","save-icp-goodness","Append all ICP goodness values here",false,"","icp_goodness.txt",cmd);

		TCLAP::ValueArg<double> arg_noise_std_xy("x","noise-std-xy","In detect-test mode,std. noise in XY",false,0,"sigma",cmd);
		TCLAP::ValueArg<double> arg_noise_std_phi("p","noise-std-phi","In detect-test mode,std. noise in PHI (deg)",false,0,"sigma",cmd);
		TCLAP::ValueArg<double> arg_noise_std_laser("l","noise-std-laser","In detect-test mode,std. noise range (m)",false,0,"sigma",cmd);
		TCLAP::ValueArg<unsigned int> arg_niters("N","iters","In detect-test mode,number of trials",false,1,"rep.count",cmd);

		TCLAP::ValueArg<double> arg_Ax("X","Ax","In detect-test mode, displacement in X (m)",false,4,"X",cmd);
		TCLAP::ValueArg<double> arg_Ay("Y","Ay","In detect-test mode, displacement in Y (m)",false,2,"Y",cmd);
		TCLAP::ValueArg<double> arg_Aphi("P","Aphi","In detect-test mode, displacement in PHI (deg)",false,30,"PHI",cmd);

		TCLAP::SwitchArg arg_noise_pose("O","noise-pose","detect-test mode: enable noise in pose",cmd, false);
		TCLAP::SwitchArg arg_noise_laser("L","noise-laser","detect-test mode: enable noise in laser",cmd, false);


		TCLAP::SwitchArg arg_verbose("v","verbose","Verbose output",cmd, false);
		TCLAP::SwitchArg arg_nologo("g","nologo","skip the logo at startup",cmd, false);
		TCLAP::SwitchArg arg_nosave("n","nosave","skip saving map images",cmd, false);
		TCLAP::SwitchArg arg_skip_icp("s","noicp","skip ICP optimization stage",cmd, false);

		// Parse arguments:
		if (!cmd.parse( argc, argv ))
			return 0; // should exit.

		fil_grid1 = arg_filgrid1.getValue();
		fil_grid2 = arg_filgrid2.getValue();
		OUTPUT_FIL = arg_out.getValue();
		CONFIG_FIL = arg_config.getValue();
		SAVE_ICP_GOODNESS_FIL = arg_icpgoodness.getValue();

		STD_NOISE_XY = arg_noise_std_xy.getValue();
		STD_NOISE_PHI = DEG2RAD( arg_noise_std_phi.getValue() );
		STD_NOISE_LASER =  arg_noise_std_laser.getValue();
		N_ITERS = arg_niters.getValue();

		GT_Ax = arg_Ax.getValue();
		GT_Ay = arg_Ay.getValue();
		GT_Aphi = DEG2RAD( arg_Aphi.getValue() );

		SAVE_SOG_3DSCENE = arg_savesog3d.getValue();
		SAVE_SOG_ALL_MAPS_OVERLAP_HYPOTHESES= arg_savesogall.getValue();
		IS_VERBOSE = arg_verbose.getValue();
		NOSAVE = arg_nosave.getValue();
		SAVE_CORR_AND_NONCORR_DISTS = arg_savecorrdists.getValue();

		if (!arg_nologo.getValue())
		{
			printf(" grid-matching - Part of the MRPT\n");
			printf(" MRPT C++ Library: %s - BUILD DATE %s\n", MRPT_getVersion().c_str(), MRPT_getCompilationDate().c_str());
		}

		SKIP_ICP_STAGE = arg_skip_icp.getValue();
		NOISE_IN_POSE  = arg_noise_pose.getValue();
		NOISE_IN_LASER = arg_noise_laser.getValue();

		is_match = arg_match.getValue();
		is_detect_test = arg_detect.getValue();

		if ( ((!is_match && !is_detect_test) || (is_match && is_detect_test)) && !SAVE_CORR_AND_NONCORR_DISTS )
		{
			cout << endl << "Error: One operation mode 'match' or 'detect-test' or 'save-corr-dists' must be selected." << endl;
			TCLAP::StdOutput so;
			so.usage(cmd);
			return 1;
		}

		if ( is_match )
		{
			// maps:
			if ( !arg_filgrid1.isSet() || !arg_filgrid2.isSet() )
			{
				cout << endl << "Error: Two maps must be passed: --map1=xxx and --map2=xxx" << endl;
				TCLAP::StdOutput so;
				so.usage(cmd);
				return 1;
			}
		}

		// pass deg 2 rads:
		GT_Aphi= DEG2RAD(GT_Aphi);

		// Invoke method:
		do_grid_align();

		return 0;
	}
	catch(std::exception &e)
	{
		cerr << e.what() << endl;
		return -1;
	}
	catch(...)
	{
		cerr << "Untyped exception." << endl;
		return -1;
	}

}
