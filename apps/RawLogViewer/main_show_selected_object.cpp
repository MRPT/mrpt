/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "xRawLogViewerMain.h"

#include <mrpt/system/datetime.h>
#include <mrpt/math/ops_matrices.h> // << ops
#include <mrpt/math/ops_vectors.h> // << ops
#include <mrpt/math/wrap2pi.h>
#include <mrpt/opengl/CAxis.h>
#include <mrpt/opengl/CPointCloudColoured.h>

#include "../wx-common/CMyRedirector.h"

#define MRPT_NO_WARN_BIG_HDR // It's ok to include ALL hdrs here.
#include <mrpt/obs.h>

#include <mrpt/slam/CSimplePointsMap.h>
#include <mrpt/slam/CColouredPointsMap.h>
#include <mrpt/poses/CPosePDFParticles.h>

#include <iomanip>

#include <mrpt/gui/WxUtils.h>

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::slam;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace std;


// Update selected item display:
void xRawLogViewerFrame::SelectObjectInTreeView( const CSerializablePtr & sel_obj )
{
	WX_START_TRY

	if (sel_obj.present())
	{
		// -----------------------------------------------------------
		// 	Write to the memo:
		// 	Redirect all the console output (cout) to the "memo":
		// ------------------------------------------------------------
		memo->Clear();

		{ // scope for redirector
		CMyRedirector myRedirector( memo );

		// For contextualize the popup menu, etc...
		curSelectedObject = sel_obj;

		// Select the panel by class (cannot make "wxWindow::FindWindowByName" to run right!!! :-(
		//  And update the required data:
		const TRuntimeClassId *classID = sel_obj->GetRuntimeClass();

		// Common data:
		if ( classID->derivedFrom( CLASS_ID( CObservation ) ) )
		{
			CObservationPtr obs( sel_obj );
			cout << "Timestamp (UTC): " << dateTimeToString(obs->timestamp) << endl;
			cout << "  (as time_t): " <<  std::fixed << std::setprecision(5) << mrpt::system::timestampTotime_t(obs->timestamp) << endl;
			cout << "  (as TTimestamp): " << obs->timestamp <<  endl;
			cout << "Sensor label: '" << obs->sensorLabel << "'" << endl;
		}
		else
		if ( classID->derivedFrom( CLASS_ID( CAction ) ) )
		{
			CActionPtr obs( sel_obj );
			cout << "Timestamp (UTC): " << dateTimeToString(obs->timestamp) << endl;
		}

		// Specific data:
		if ( classID  == CLASS_ID(CObservation2DRangeScan) )
		{
			// ----------------------------------------------------------------------
			//              CObservation2DRangeScan
			// ----------------------------------------------------------------------
			Notebook1->ChangeSelection( 2 );
			CObservation2DRangeScanPtr obs = CObservation2DRangeScanPtr( sel_obj );
			curSelectedObservation = CObservationPtr( sel_obj );

			cout << "Homogeneous matrix for the sensor's 3D pose, relative to robot base:\n";
			cout << obs->sensorPose.getHomogeneousMatrixVal()
				 << obs->sensorPose << endl;

			cout << format("Samples direction: %s\n", (obs->rightToLeft) ? "Right->Left" : "Left->Right");
			cout << format("Points in the scan: %u\n", (unsigned)obs->scan.size());
			cout << format("Estimated sensor 'sigma': %f\n", obs->stdError );
			cout << format("Increment in pitch during the scan: %f deg\n", RAD2DEG( obs->deltaPitch ));

			size_t i,inval = 0;
			for (i=0;i<obs->scan.size();i++) if (!obs->validRange[i]) inval++;
			cout << format("Invalid points in the scan: %u\n", (unsigned)inval);

			cout << format("Sensor maximum range: %.02f m\n", obs->maxRange );
			cout << format("Sensor field-of-view (\"aperture\"): %.01f deg\n", RAD2DEG(obs->aperture) );

			cout << "Raw scan values: [";
			for (i=0;i<obs->scan.size();i++) cout << format("%.03f ", obs->scan[i] );
			cout << "]\n";

			cout << "Raw valid-scan values: [";
			for (i=0;i<obs->scan.size();i++) cout << format("%u ", obs->validRange[i] ? 1:0 );
			cout << "]\n\n";

			// The plot:
			mrpt::slam::CSimplePointsMap  dummMap;
			dummMap.insertionOptions.minDistBetweenLaserPoints = 0;
			dummMap.insertObservation( obs.pointer() );

			vector<float>    Xs,Ys;
			dummMap.getAllPoints(Xs,Ys);

			lyScan2D->SetData(Xs,Ys);
			plotScan2D->Fit();      // Update the window to show the new data fitted.
		}
		else
			if ( classID  == CLASS_ID(CObservationImage) )
			{
				// ----------------------------------------------------------------------
				//              CObservationImage
				// ----------------------------------------------------------------------
				Notebook1->ChangeSelection( 3 );
				CObservationImagePtr obs = CObservationImagePtr( sel_obj );
				curSelectedObservation = CObservationPtr( sel_obj );

				cout << "Homogeneous matrix for the sensor's 3D pose, relative to robot base:\n";
				cout << obs->cameraPose.getHomogeneousMatrixVal()
				<< obs->cameraPose << endl;

				cout << format("Focal length: %.03f mm\n",obs->cameraParams.focalLengthMeters*1000);

				cout << "Intrinsic parameters matrix for the camera:"<< endl
				<< obs->cameraParams.intrinsicParams.inMatlabFormat() << endl << obs->cameraParams.intrinsicParams << endl;

				cout << "Distorsion parameters for the camera: "
				<< obs->cameraParams.getDistortionParamsAsVector() << endl;

				if (obs->image.isExternallyStored())
					cout << " Image is stored externally in file: " << obs->image.getExternalStorageFile() << endl;

				cout << format(" Image size: %ux%u pixels\n", (unsigned int)obs->image.getWidth(), (unsigned int)obs->image.getHeight() );

				cout << " Channels order: " << obs->image.getChannelsOrder() << endl;

				cout << format(" Rows are stored in top-bottom order: %s\n",
							   obs->image.isOriginTopLeft() ? "YES" : "NO");

				// Get bitmap:
				// ----------------------
				wxImage *img = mrpt::gui::MRPTImage2wxImage( obs->image );
				bmpObsImage->SetBitmap( wxBitmap(*img) );
				bmpObsImage->Refresh();
				delete img;
				obs->image.unload(); // For externally-stored datasets
			}
			else
				if ( classID  == CLASS_ID(CObservationStereoImages) )
				{
					// ----------------------------------------------------------------------
					//              CObservationStereoImages
					// ----------------------------------------------------------------------
					Notebook1->ChangeSelection( 4);
					CObservationStereoImagesPtr obs = CObservationStereoImagesPtr(sel_obj);
					curSelectedObservation = CObservationPtr( sel_obj );


					cout << "Homogeneous matrix for the sensor's 3D pose, relative to robot base:\n";
					cout << obs->cameraPose.getHomogeneousMatrixVal() << endl
					<< "Camera pose: " << obs->cameraPose << endl
					<< "Camera pose (YPR): " << CPose3D(obs->cameraPose) << endl
					<< endl;

					mrpt::utils::TStereoCamera stParams;
					obs->getStereoCameraParams(stParams);
					cout << stParams.dumpAsText() << endl;

					cout << "Right camera pose wrt left camera (YPR):" << endl << CPose3D(stParams.rightCameraPose) << endl;

					if (obs->imageLeft.isExternallyStored())
						cout << " Left image is stored externally in file: " << obs->imageLeft.getExternalStorageFile() << endl;

					cout << " Right image";
					if (obs->hasImageRight )
					{
						if (obs->imageRight.isExternallyStored())
							cout << " is stored externally in file: " << obs->imageRight.getExternalStorageFile() << endl;
					}
					else cout << " : No.\n";

					cout << " Disparity image";
					if (obs->hasImageDisparity )
					{
						if (obs->imageDisparity.isExternallyStored())
							cout << " is stored externally in file: " << obs->imageDisparity.getExternalStorageFile() << endl;
					}
					else cout << " : No.\n";

					cout << format(" Image size: %ux%u pixels\n", (unsigned int)obs->imageLeft.getWidth(), (unsigned int)obs->imageLeft.getHeight() );

					cout << " Channels order: " << obs->imageLeft.getChannelsOrder() << endl;

					cout << format(" Rows are stored in top-bottom order: %s\n",
								   obs->imageLeft.isOriginTopLeft() ? "YES" : "NO");

					// Images:
					// ----------------------
					wxImage *imgLeft = mrpt::gui::MRPTImage2wxImage( obs->imageLeft );
					bmpObsStereoLeft->SetBitmap( wxBitmap(*imgLeft) );
					bmpObsStereoLeft->Refresh();
					delete imgLeft;

					wxImage *imgRight = mrpt::gui::MRPTImage2wxImage( obs->imageRight );
					bmpObsStereoRight->SetBitmap( wxBitmap(*imgRight) );
					bmpObsStereoRight->Refresh();
					delete imgRight;

					wxImage *imgDisp = mrpt::gui::MRPTImage2wxImage( obs->imageDisparity );
					bmpObsStereoDisp->SetBitmap( wxBitmap(*imgDisp) );
					bmpObsStereoDisp->Refresh();
					delete imgDisp;

				}
					else
					if ( classID == CLASS_ID(CActionRobotMovement2D) )
					{
						// ----------------------------------------------------------------------
						//              CActionRobotMovement2D
						// ----------------------------------------------------------------------
						Notebook1->ChangeSelection( 1 );

						CActionRobotMovement2DPtr act = CActionRobotMovement2DPtr( sel_obj );

						CPose2D			Ap;
						CMatrixDouble33 mat;
						act->poseChange->getCovarianceAndMean(mat,Ap);

						cout << "Robot Movement (as a gaussian pose change):\n";
						cout << " Mean = " << Ap << endl;

						cout << format(" Covariance:     DET=%e\n", mat.det());

						cout << format("      %e %e %e\n", mat(0,0), mat(0,1), mat(0,2) );
						cout << format("      %e %e %e\n", mat(1,0), mat(1,1), mat(1,2) );
						cout << format("      %e %e %e\n", mat(2,0), mat(2,1), mat(2,2) );

						cout << endl;

						cout << " Actual reading from the odometry increment = " << act->rawOdometryIncrementReading << endl;

						cout << format("Actual PDF class is: '%s'\n",
									   act->poseChange->GetRuntimeClass()->className );

						if (act->poseChange->GetRuntimeClass()==CLASS_ID(CPosePDFParticles))
						{
							CPosePDFParticlesPtr aux = CPosePDFParticlesPtr( act->poseChange );
							cout << format (" (Particle count = %u)\n", (unsigned)aux->m_particles.size() );
						}
						cout << endl;

						cout << "Estimation method: ";
						switch (act->estimationMethod)
						{
						case CActionRobotMovement2D::emOdometry:
							cout << "emOdometry\n";
							break;
						case CActionRobotMovement2D::emScan2DMatching:
							cout << "emScan2DMatching\n";
							break;
						default:
							cout <<  "(Unknown ID!)\n";
							break;
						};

						// Additional data:
						if (act->hasEncodersInfo)
						{
							cout << format(" Encoder info: deltaL=%i deltaR=%i\n", act->encoderLeftTicks, act->encoderRightTicks );
						}
						else    cout << "Encoder info: Not available!\n";

						if (act->hasVelocities)
						{
							cout << format(" Velocity info: v=%.03f m/s  w=%.03f deg/s\n", act->velocityLin, RAD2DEG(act->velocityAng) );
						}
						else    cout << "Velocity info: Not available!\n";


						// Plot the 2D pose samples:
						unsigned int                    N = 1000;
						vector<CVectorDouble>       samples;
						vector<float>                    xs(N),ys(N),ps(N),dumm(N,0.1f);

						// Draw a set of random (x,y,phi) samples:
						act->poseChange->drawManySamples( N, samples );

						// Pass to vectors and draw them:
						for (unsigned int i=0;i<N;i++)
						{
							xs[i] = samples[i][0];
							ys[i] = samples[i][1];
							ps[i] = RAD2DEG(samples[i][2]);
						}

						lyAction2D_XY->SetData(xs,ys);
						lyAction2D_PHI->SetData(ps,dumm);

						plotAct2D_XY->Fit();
						plotAct2D_PHI->Fit();
					}
					else
						if ( classID  == CLASS_ID(CObservationBeaconRanges) )
						{
							// ----------------------------------------------------------------------
							//              CObservationBeaconRanges
							// ----------------------------------------------------------------------
							Notebook1->ChangeSelection( 5 );
							CObservationBeaconRangesPtr obs = CObservationBeaconRangesPtr( sel_obj );
							curSelectedObservation = CObservationPtr( sel_obj );


							cout << "Auxiliary estimated pose (if available): " << obs->auxEstimatePose << endl;

							cout << format("minSensorDistance=%f m\n",obs->minSensorDistance);
							cout << format("maxSensorDistance=%f m\n",obs->maxSensorDistance);
							cout << format("stdError=%f m\n\n",obs->stdError);

							cout << format("There are %u range measurements:\n\n",(unsigned)obs->sensedData.size());

							cout << "  BEACON   RANGE     SENSOR POSITION ON ROBOT \n";
							cout << "------------------------------------------------\n";

							deque<CObservationBeaconRanges::TMeasurement>::iterator it;

							for (it=obs->sensedData.begin(); it!=obs->sensedData.end(); it++)
							{
								cout << format("   %i      %.04f      (%.03f,%.03f,%.03f)\n",
											   (int)it->beaconID,it->sensedDistance,
											   it->sensorLocationOnRobot.x(),it->sensorLocationOnRobot.y(),it->sensorLocationOnRobot.z());
							}
						}
						else
							if ( classID  == CLASS_ID(CObservationGasSensors) )
							{
								// ----------------------------------------------------------------------
								//              CObservationGasSensors
								// ----------------------------------------------------------------------
								Notebook1->ChangeSelection( 6 );
								CObservationGasSensorsPtr obs = CObservationGasSensorsPtr( sel_obj );
								curSelectedObservation = CObservationPtr( sel_obj );



								for (size_t j=0;j<obs->m_readings.size();j++)
								{
									cout << format("e-nose #%u:\n",(unsigned)j);

									vector<float>::iterator it;
									vector_int::iterator   itKind;

									ASSERT_( obs->m_readings[j].readingsVoltage.size() == obs->m_readings[j].sensorTypes.size());

									for (it=obs->m_readings[j].readingsVoltage.begin(),itKind=obs->m_readings[j].sensorTypes.begin();it!=obs->m_readings[j].readingsVoltage.end();it++,itKind++)
										cout << format( "%04X: %.03f ", *itKind, *it);

									cout << endl;

									cout << format("  Sensor pose on robot: (x,y,z)=(%.02f,%.02f,%.02f)\n",
												   obs->m_readings[j].eNosePoseOnTheRobot.x,
												   obs->m_readings[j].eNosePoseOnTheRobot.y,
												   obs->m_readings[j].eNosePoseOnTheRobot.z );

									cout << "Measured temperature: ";
									if (obs->m_readings[j].hasTemperature)
										cout << format("%.03f degC\n", obs->m_readings[j].temperature );
									else
										cout << "NOT AVAILABLE\n";
								}
							}
							else
								if ( classID  == CLASS_ID(CObservationGPS) )
								{
									// ----------------------------------------------------------------------
									//              CObservationGPS
									// ----------------------------------------------------------------------
									Notebook1->ChangeSelection( 7 );
									CObservationGPSPtr obs = CObservationGPSPtr( sel_obj );
									curSelectedObservation = CObservationPtr( sel_obj );

									if (obs->has_GGA_datum)
										cout << endl << "Satellite time: " << format("%02u:%02u:%02.3f",obs->GGA_datum.UTCTime.hour,obs->GGA_datum.UTCTime.minute,obs->GGA_datum.UTCTime.sec) << endl;

									cout << "Sensor position on the robot: " << obs->sensorPose << endl;

									obs->dumpToConsole();
								}
								else
									if ( classID  == CLASS_ID(CObservationBearingRange) )
									{
										// ----------------------------------------------------------------------
										//              CObservationBearingRange
										// ----------------------------------------------------------------------
										Notebook1->ChangeSelection( 8 );
										CObservationBearingRangePtr obs = CObservationBearingRangePtr( sel_obj );
										curSelectedObservation = CObservationPtr( sel_obj );


										cout << "Homogeneous matrix for the sensor's 3D pose, relative to robot base:\n";
										cout << obs->sensorLocationOnRobot.getHomogeneousMatrixVal()
										<< obs->sensorLocationOnRobot << endl << endl;

										cout << "Do observations have individual covariance matrices? " << (obs->validCovariances ? "YES":"NO") << endl << endl;

										cout << "Default noise sigmas:" << endl;
										cout << "sensor_std_range (m)   : " << obs->sensor_std_range << endl;
										cout << "sensor_std_yaw   (deg) : " << RAD2DEG(obs->sensor_std_yaw) << endl;
										cout << "sensor_std_pitch (deg) : " << RAD2DEG(obs->sensor_std_pitch) << endl;

										cout << endl;

										// For each entry in this sequence:
										cout << "  LANDMARK_ID    RANGE (m)    YAW (deg)    PITCH (deg)   COV. MATRIX (optional)" << endl;
										cout << "--------------------------------------------------------------------------------------" << endl;
										for (size_t q=0;q<obs->sensedData.size();q++)
										{

											cout << "      ";
											if (obs->sensedData[q].landmarkID==INVALID_LANDMARK_ID)
												cout << "(NO ID)";
											else cout << format("%7u",obs->sensedData[q].landmarkID);

											cout << format("   %10.03f  %10.03f %10.03f        ",
														   obs->sensedData[q].range,
														   RAD2DEG( wrapToPi( obs->sensedData[q].yaw)),
														   RAD2DEG( wrapToPi(obs->sensedData[q].pitch)) );

											if (obs->validCovariances)
												cout << obs->sensedData[q].covariance.inMatlabFormat() << endl;
											else
												cout << "  (N/A)\n";
										}

										// The plot:
										size_t nPts = obs->sensedData.size();
										vector<float>    Xs(nPts),Ys(nPts),Zs(nPts);
										for (size_t k=0;k<nPts;k++)
										{
											float R      = obs->sensedData[k].range;
											float yaw    = obs->sensedData[k].yaw;
											float pitch  = obs->sensedData[k].pitch;

											CPoint3D   local(
												R * cos(yaw) * cos(pitch),
												R * sin(yaw) * cos(pitch),
												R * sin(pitch) );
											CPoint3D relRobot( obs->sensorLocationOnRobot + local);
											Xs[k] = relRobot.x();
											Ys[k] = relRobot.y();
											Zs[k] = relRobot.z();
										}
										lyRangeBearingLandmarks->SetData(Xs,Ys);
										plotRangeBearing->LockAspect();
										plotRangeBearing->Fit();      // Update the window to show the new data fitted.
									}
									else
										if ( classID  == CLASS_ID(CObservationBatteryState) )
										{
											// ----------------------------------------------------------------------
											//              CObservationBatteryState
											// ----------------------------------------------------------------------
											CObservationBatteryStatePtr obs = CObservationBatteryStatePtr( sel_obj);
											curSelectedObservation = CObservationPtr( sel_obj );
											cout << endl;

											cout << format("Measured VoltageMainRobotBattery: %.02fV  isValid= %s \n",
												obs->voltageMainRobotBattery,
												(obs->voltageMainRobotBatteryIsValid == true)? "True":"False" );

											cout << format("Measured VoltageMainRobotComputer: %.02fV  isValid= %s \n",
												obs->voltageMainRobotComputer,
												(obs->voltageMainRobotComputerIsValid == true)? "True":"False" );

											cout << "VoltageOtherBatteries: \n";
											for(CVectorDouble::Index i=0; i<obs->voltageOtherBatteries.size(); i++)
											{
												cout << format("Index: %d --> %.02fV  isValid= %s \n",
												int(i),
												obs->voltageOtherBatteries[i],
												(obs->voltageOtherBatteriesValid[i] == true)? "True":"False" );
											}



										}
										else
										if ( classID  == CLASS_ID(CObservationWirelessPower) )
										{
											// ----------------------------------------------------------------------
											//              CObservationBatteryState
											// ----------------------------------------------------------------------
											CObservationWirelessPowerPtr obs = CObservationWirelessPowerPtr( sel_obj);
											curSelectedObservation = CObservationPtr( sel_obj );
											cout << endl;

											cout << format("Measured Power: %.02f/100\n",
												obs->power);
										}
										else
										if ( classID  == CLASS_ID(CObservationRFID) )
										{
											// ----------------------------------------------------------------------
											//              CObservationRFID
											// ----------------------------------------------------------------------
											CObservationRFIDPtr obs = CObservationRFIDPtr( sel_obj);
											curSelectedObservation = CObservationPtr( sel_obj );
											cout << endl;

											cout << "Number of RFID tags sensed: " << obs->tag_readings.size() << endl << endl;

											for (size_t i=0;i<obs->tag_readings.size();i++)
											{
											    const CObservationRFID::TTagReading &rfid = obs->tag_readings[i];

											    cout << "#"<< i
                                                    << ": Power=" << rfid.power
                                                    << " (dBm) | AntennaPort=" << rfid.antennaPort
                                                    << " | EPC=" << rfid.epc << endl;
											}
										}
										else
											if ( classID  == CLASS_ID(CObservationIMU) )
											{
												// ----------------------------------------------------------------------
												//              CObservationIMU
												// ----------------------------------------------------------------------
												Notebook1->ChangeSelection( 0 );
												CObservationIMUPtr obs = CObservationIMUPtr( sel_obj);
												curSelectedObservation = CObservationPtr( sel_obj );

												cout << "Sensor pose on the robot: " << obs->sensorPose << endl;

												cout << format("Orientation (degrees): (yaw,pitch,roll)=(%.06f, %.06f, %.06f)\n\n",
													RAD2DEG( obs->rawMeasurements[IMU_YAW] ),
													RAD2DEG( obs->rawMeasurements[IMU_PITCH] ),
													RAD2DEG( obs->rawMeasurements[IMU_ROLL] ) );

												// Units:
												// Use "COUNT_IMU_DATA_FIELDS" so a compile error happens if the sizes don't fit ;-)
												static const char * imu_units[ mrpt::slam::COUNT_IMU_DATA_FIELDS ] =
												{
													"m/s^2", //	IMU_X_ACC,
													"m/s^2", //	IMU_Y_ACC,
													"m/s^2", //	IMU_Z_ACC,
													"rad/s", //	IMU_YAW_VEL,
													"rad/s", //	IMU_PITCH_VEL,
													"rad/s", //	IMU_ROLL_VEL,
													"m/s", //	IMU_X_VEL,
													"m/s", //	IMU_Y_VEL,
													"m/s", //	IMU_Z_VEL,
													"rad", //	IMU_YAW,
													"rad", //	IMU_PITCH,
													"rad", //	IMU_ROLL,
													"m", //	IMU_X,
													"m", //	IMU_Y,
													"m",  //	IMU_Z
													"gauss", // IMU_MAG_X,
													"gauss", // IMU_MAG_Y,
													"gauss", // IMU_MAG_Z,
													"Pa", // IMU_PRESSURE,
													"m", // IMU_ALTITUDE,
													"deg." // IMU_TEMPERATURE,
												};

	#define DUMP_IMU_DATA(x)  \
		cout << format("%15s = ",#x); \
		if (obs->dataIsPresent[x]) \
			cout << format("%10f %s\n", obs->rawMeasurements[x], imu_units[x]); \
		else  	cout << "(not present)\n";


												DUMP_IMU_DATA(IMU_X_ACC)
												DUMP_IMU_DATA(IMU_Y_ACC)
												DUMP_IMU_DATA(IMU_Z_ACC)
												DUMP_IMU_DATA(IMU_YAW_VEL)
												DUMP_IMU_DATA(IMU_PITCH_VEL)
												DUMP_IMU_DATA(IMU_ROLL_VEL)
												DUMP_IMU_DATA(IMU_X_VEL)
												DUMP_IMU_DATA(IMU_Y_VEL)
												DUMP_IMU_DATA(IMU_Z_VEL)
												DUMP_IMU_DATA(IMU_YAW)
												DUMP_IMU_DATA(IMU_PITCH)
												DUMP_IMU_DATA(IMU_ROLL)
												DUMP_IMU_DATA(IMU_X)
												DUMP_IMU_DATA(IMU_Y)
												DUMP_IMU_DATA(IMU_Z)
												DUMP_IMU_DATA(IMU_MAG_X)
												DUMP_IMU_DATA(IMU_MAG_Y)
												DUMP_IMU_DATA(IMU_MAG_Z)
												DUMP_IMU_DATA(IMU_PRESSURE)
												DUMP_IMU_DATA(IMU_ALTITUDE)
												DUMP_IMU_DATA(IMU_TEMPERATURE)
											}
											else
												if ( classID  == CLASS_ID(CObservationOdometry) )
												{
													// ----------------------------------------------------------------------
													//              CObservationOdometry
													// ----------------------------------------------------------------------
													Notebook1->ChangeSelection( 0 );
													CObservationOdometryPtr obs = CObservationOdometryPtr( sel_obj );
													curSelectedObservation = CObservationPtr( sel_obj );

													cout << endl << "Odometry reading: " << obs->odometry << endl;

													// Additional data:
													if (obs->hasEncodersInfo)
													{
														cout << format(" Encoder info: deltaL=%i deltaR=%i\n", obs->encoderLeftTicks, obs->encoderRightTicks );
													}
													else    cout << "Encoder info: Not available!\n";

													if (obs->hasVelocities)
													{
														cout << format(" Velocity info: v=%.03f m/s  w=%.03f deg/s\n", obs->velocityLin, RAD2DEG(obs->velocityAng) );
													}
													else   cout << "Velocity info: Not available!\n";

												}
												else
													if ( classID  == CLASS_ID(CActionRobotMovement3D) )
													{
														// ----------------------------------------------------------------------
														//              CActionRobotMovement3D
														// ----------------------------------------------------------------------
														//Notebook1->ChangeSelection( 1 );
														CActionRobotMovement3DPtr act = CActionRobotMovement3DPtr( sel_obj );
														cout << "Robot Movement (as a gaussian pose change):\n";
														cout << act->poseChange << endl;
													}
													else
													if ( classID  == CLASS_ID(CObservationRange) )
													{
														// ----------------------------------------------------------------------
														//              CObservationRange
														// ----------------------------------------------------------------------
														Notebook1->ChangeSelection( 0 );
														CObservationRangePtr obs = CObservationRangePtr( sel_obj );
														curSelectedObservation = CObservationPtr( sel_obj );

														cout << endl;
														cout << "minSensorDistance   = " << obs->minSensorDistance << " m" << endl;
														cout << "maxSensorDistance   = " << obs->maxSensorDistance << " m" << endl;
														cout << "sensorConeApperture = " << RAD2DEG(obs->sensorConeApperture) << " deg" << endl;

														// For each entry in this sequence:
														cout << "  SENSOR_ID    RANGE (m)    SENSOR POSE (on the robot)" << endl;
														cout << "-------------------------------------------------------" << endl;
														for (size_t q=0;q<obs->sensedData.size();q++)
														{
															cout << format("     %7u",(unsigned int)obs->sensedData[q].sensorID );
															cout << format("    %4.03f   ",obs->sensedData[q].sensedDistance);
															cout << obs->sensedData[q].sensorPose << endl;

														}
													}
													else
													if ( classID  == CLASS_ID(CObservation3DRangeScan) )
													{
														// ----------------------------------------------------------------------
														//              CObservation3DRangeScan
														// ----------------------------------------------------------------------
														Notebook1->ChangeSelection( 9 );
														CObservation3DRangeScanPtr obs = CObservation3DRangeScanPtr( sel_obj );

														obs->load(); // Make sure the 3D point cloud, etc... are all loaded in memory.

														curSelectedObservation = CObservationPtr( sel_obj );


														cout << endl;
														cout << "maxRange = " << obs->maxRange << " m" << endl;

														const bool generate3Donthefly = !obs->hasPoints3D && mnuItemEnable3DCamAutoGenPoints->IsChecked();
														if (generate3Donthefly)
														{
															obs->project3DPointsFromDepthImage();
														}

														cout << "Has 3D point cloud? ";
														if (obs->hasPoints3D)
														{
															cout << "YES: " << obs->points3D_x.size() << " points";
															if (obs->points3D_isExternallyStored())
																cout << ". External file: " << obs->points3D_getExternalStorageFile() << endl;
															else cout << " (embedded)." << endl;
														}
														else	cout << "NO" << endl;

														if (generate3Donthefly)
															cout << "NOTICE: The stored observation didn't contain 3D points, but these have been generated on-the-fly just for visualization purposes.\n"
															"(You can disable this behavior from the menu Sensors->3D depth cameras\n\n";

														cout << "Has raw range data? " << (obs->hasRangeImage ? "YES": "NO");
														if (obs->hasRangeImage)
														{
															if (obs->rangeImage_isExternallyStored())
																 cout << ". External file: " << obs->rangeImage_getExternalStorageFile() << endl;
															else cout << " (embedded)." << endl;
														}

														cout << endl << "Has intensity data? " << (obs->hasIntensityImage ? "YES": "NO");
														if (obs->hasIntensityImage)
														{
															if (obs->intensityImage.isExternallyStored())
																cout << ". External file: " << obs->intensityImage.getExternalStorageFile() << endl;
															else cout << " (embedded).\n";
															// Channel?
															cout << "Source channel: " << mrpt::utils::TEnumType<CObservation3DRangeScan::TIntensityChannelID>::value2name(obs->intensityImageChannel) << endl;
														}

														cout << endl << "Has confidence data? " << (obs->hasConfidenceImage ? "YES": "NO");
														if (obs->hasConfidenceImage)
														{
															if (obs->confidenceImage.isExternallyStored())
																cout << ". External file: " << obs->confidenceImage.getExternalStorageFile() << endl;
															else cout << " (embedded)." << endl;
														}

														cout << endl << endl;
														cout << "Depth camera calibration parameters:" << endl;
														{
															CConfigFileMemory cfg;
															obs->cameraParams.saveToConfigFile("DEPTH_CAM_PARAMS",cfg);
															cout << cfg.getContent() << endl;
														}
														cout << endl << "Intensity camera calibration parameters:" << endl;
														{
															CConfigFileMemory cfg;
															obs->cameraParamsIntensity.saveToConfigFile("INTENSITY_CAM_PARAMS",cfg);
															cout << cfg.getContent() << endl;
														}
														cout << endl << endl << "Pose of the intensity cam. wrt the depth cam:\n"
															<< obs->relativePoseIntensityWRTDepth << endl
															<< obs->relativePoseIntensityWRTDepth.getHomogeneousMatrixVal() << endl;

														// Update 3D view ==========
													#if RAWLOGVIEWER_HAS_3D
														this->m_gl3DRangeScan->m_openGLScene->clear();
														//this->m_gl3DRangeScan->m_openGLScene->insert( mrpt::opengl::stock_objects::CornerXYZ() );
														this->m_gl3DRangeScan->m_openGLScene->insert( mrpt::opengl::CAxis::Create(-20,-20,-20,20,20,20,1,2,true ));

														mrpt::opengl::CPointCloudColouredPtr pnts = mrpt::opengl::CPointCloudColoured::Create();
														CColouredPointsMap  pointMap;
														pointMap.colorScheme.scheme = CColouredPointsMap::cmFromIntensityImage;

														if (obs->hasPoints3D)
														{
															// Assign only those points above a certain threshold:
															const int confThreshold =   obs->hasConfidenceImage ? slid3DcamConf->GetValue() : 0;

															if (confThreshold==0) // This includes when there is no confidence image.
															{
																pointMap.insertionOptions.minDistBetweenLaserPoints = 0; // don't drop any point
																pointMap.insertObservation(obs.pointer());
																pnts->loadFromPointsMap(&pointMap);
															}
															else
															{
																pnts->clear();

																const vector<float>  &obs_xs = obs->points3D_x;
																const vector<float>  &obs_ys = obs->points3D_y;
																const vector<float>  &obs_zs = obs->points3D_z;

																size_t i=0;

																const size_t W = obs->confidenceImage.getWidth();
																const size_t H = obs->confidenceImage.getHeight();

																ASSERT_(obs->confidenceImage.isColor()==false)
																ASSERT_(obs_xs.size() == H*W)

																for(size_t r=0;r<H;r++)
																{
																	unsigned char const * ptr_lin = obs->confidenceImage.get_unsafe(0,r,0);
																	for(size_t c=0;c<W;c++,  i++ )
																	{
																		unsigned char conf = *ptr_lin++;
																		if (conf>=confThreshold)
																			pnts->push_back(obs_xs[i],obs_ys[i],obs_zs[i],1,1,1);
																	}
																}
															}

															pnts->setPointSize(4.0);

															// Translate the 3D cloud since sensed points are relative to the camera, but the camera may be translated wrt the robot (our 0,0,0 here):
															pnts->setPose( obs->sensorPose );
														}
														this->m_gl3DRangeScan->m_openGLScene->insert(pnts);
														this->m_gl3DRangeScan->Refresh();

														// Free memory:
														if (generate3Donthefly)
														{
															obs->hasPoints3D = false;
															obs->resizePoints3DVectors(0);
														}

													#endif

														// Update intensity image ======
														{
															CImage im;
															if (obs->hasIntensityImage)
																 im = obs->intensityImage;
															else im.resize(10,10,CH_GRAY, true);
															wxImage *img = mrpt::gui::MRPTImage2wxImage( im );
															if (img->IsOk())
																bmp3Dobs_int->SetBitmap( wxBitmap(*img) );
															bmp3Dobs_int->Refresh();
															delete img;
															obs->intensityImage.unload(); // For externally-stored datasets
														}
														// Update depth image ======
														{
															CImage  auxImg;
															if (obs->hasRangeImage)
															{
																// Convert to range [0,255]
																mrpt::math::CMatrix normalized_range = obs->rangeImage;
																const float max_rang = std::max(obs->maxRange, normalized_range.maximum() );
																if (max_rang>0) normalized_range *= 255./max_rang;
																auxImg.setFromMatrix(normalized_range, false /* it's in range [0,255] */);
															}
															else auxImg.resize(10,10, CH_GRAY, true );

															wxImage *img = mrpt::gui::MRPTImage2wxImage( auxImg );
															if (img->IsOk())
																bmp3Dobs_depth->SetBitmap( wxBitmap(*img) );
															bmp3Dobs_depth->Refresh();
															delete img;
														}
														// Update confidence image ======
														{
															wxImage *img;
															if (obs->hasConfidenceImage)
																img = mrpt::gui::MRPTImage2wxImage( obs->confidenceImage );
															else
															{
																mrpt::utils::CImage dumm(10,10);
																img = mrpt::gui::MRPTImage2wxImage( dumm );
															}
															if (img->IsOk())
																bmp3Dobs_conf->SetBitmap( wxBitmap(*img) );
															bmp3Dobs_conf->Refresh();
															delete img;
															obs->confidenceImage.unload(); // For externally-stored datasets
														}

														obs->unload();

													}
													else
													if ( classID  == CLASS_ID(CObservationStereoImagesFeatures) )
													{
														// ----------------------------------------------------------------------
														//              CObservationStereoImagesFeatures
														// ----------------------------------------------------------------------
														CObservationStereoImagesFeaturesPtr obs = CObservationStereoImagesFeaturesPtr( sel_obj );
														curSelectedObservation = CObservationPtr( sel_obj );

														cout << "Homogeneous matrix for the sensor's 3D pose, relative to robot base:\n";
														cout << obs->cameraPoseOnRobot.getHomogeneousMatrixVal()
														<< obs->cameraPoseOnRobot << endl;

														cout << "Homogeneous matrix for the RIGHT camera's 3D pose, relative to LEFT camera reference system:\n";
														cout << obs->rightCameraPose.getHomogeneousMatrixVal()
														<< obs->rightCameraPose << endl;

														cout << "Intrinsic parameters matrix for the LEFT camera:"<< endl;
														CMatrixDouble33 aux = obs->cameraLeft.intrinsicParams;
														cout << aux.inMatlabFormat() << endl << aux << endl;

														cout << "Distortion parameters vector for the LEFT camera:"<< endl << "[ ";
														for( unsigned int i = 0; i < 5; ++i )
															cout << obs->cameraLeft.dist[i] << " ";
														cout << "]" << endl;

														cout << "Intrinsic parameters matrix for the RIGHT camera:"<< endl;
														aux = obs->cameraRight.intrinsicParams;
														cout << aux.inMatlabFormat() << endl << aux << endl;

														cout << "Distortion parameters vector for the RIGHT camera:"<< endl << "[ ";
														for( unsigned int i = 0; i < 5; ++i )
															cout << obs->cameraRight.dist[i] << " ";
														cout << "]"<< endl;

														cout << endl << format(" Image size: %ux%u pixels\n", (unsigned int)obs->cameraLeft.ncols, (unsigned int)obs->cameraLeft.nrows );
														cout << endl << format(" Number of features in images: %u\n", (unsigned int)obs->theFeatures.size() );

													}
													else
                                                    if ( classID  == CLASS_ID(CObservationCANBusJ1939) )
                                                    {
                                                        // ----------------------------------------------------------------------
                                                        //              CObservationCANBusJ1939
                                                        // ----------------------------------------------------------------------
                                                        Notebook1->ChangeSelection( 0 );
                                                        CObservationCANBusJ1939Ptr obs = CObservationCANBusJ1939Ptr( sel_obj );
                                                        curSelectedObservation = CObservationPtr( sel_obj );

                                                        cout << "Priority: " << format("0x%02X",obs->m_priority) << " [Dec: " << int(obs->m_priority) << "]" << endl;
                                                        cout << "Parameter Group Number (PGN): " << format("0x%04X",obs->m_pgn) << " [Dec: " << int(obs->m_pgn) << "]" << endl;
                                                        cout << "PDU Format: " << format("0x%02X",obs->m_pdu_format) << " [Dec: " << int(obs->m_pdu_format) << "]" << endl;
                                                        cout << "PDU Spec: " << format("0x%02X",obs->m_pdu_spec) << " [Dec: " << int(obs->m_pdu_spec) << "]" << endl;
                                                        cout << "Source address: " << format("0x%02X",obs->m_src_address) << " [Dec: " << int(obs->m_src_address) << "]" << endl;
                                                        cout << "Data length: " << format("0x%02X",obs->m_data_length) << " [Dec: " << int(obs->m_data_length) << "]" << endl;
                                                        cout << "Data: ";
                                                        for(uint8_t k = 0; k < obs->m_data.size(); ++k)
                                                            cout << format("0x%02X",obs->m_data[k]) << " ";
                                                        cout << " [Dec: ";
                                                        for(uint8_t k = 0; k < obs->m_data.size(); ++k)
                                                            cout << int(obs->m_data[k]) << " ";
                                                        cout << "]" << endl;

                                                        cout << "Raw frame: ";
                                                        for(uint8_t k = 0; k < obs->m_raw_frame.size(); ++k)
                                                            cout << obs->m_raw_frame[k];
                                                        cout << endl;

                                                    }
                                                    else
                                                    if ( classID  == CLASS_ID(CObservationRawDAQ ) )
                                                    {
                                                         // ----------------------------------------------------------------------
                                                         //              CObservationRawDAQ
                                                         // ----------------------------------------------------------------------
                                                         Notebook1->ChangeSelection( 0 );
                                                         CObservationRawDAQPtr obs = CObservationRawDAQPtr( sel_obj );
                                                         curSelectedObservation = CObservationPtr( sel_obj );

                                                         cout << "Sample rate             : " << obs->sample_rate << " Hz" << endl;
                                                         cout << "Analog IN Channel count : " << obs->AIN_channel_count << endl;
                                                         cout << "Analog IN interleaved?  : " << (obs->AIN_interleaved ? "yes":"no")<< endl;

#define RAWDAQ_SHOW_FIRSTS(_VEC) \
    cout << "Raw data in " #_VEC " ("<< obs->_VEC.size() <<" entries): First values ["; \
    if (!obs->_VEC.empty()) { \
        for (size_t i=1;i<=std::min(obs->_VEC.size(),static_cast<size_t>(10));i++) \
            cout << obs->_VEC[i-1] << " "; \
        cout << " ... "; \
        } \
    cout << "]\n";


                                                        RAWDAQ_SHOW_FIRSTS(AIN_8bits)
                                                        RAWDAQ_SHOW_FIRSTS(AIN_16bits)
                                                        RAWDAQ_SHOW_FIRSTS(AIN_32bits)
                                                        RAWDAQ_SHOW_FIRSTS(AIN_float)
                                                        RAWDAQ_SHOW_FIRSTS(AIN_double)
                                                        RAWDAQ_SHOW_FIRSTS(AOUT_8bits)
                                                        RAWDAQ_SHOW_FIRSTS(AOUT_16bits)
                                                        RAWDAQ_SHOW_FIRSTS(AOUT_float)
                                                        RAWDAQ_SHOW_FIRSTS(AOUT_double)
                                                        RAWDAQ_SHOW_FIRSTS(DIN)
                                                        RAWDAQ_SHOW_FIRSTS(DOUT)
                                                        RAWDAQ_SHOW_FIRSTS(CNTRIN_32bits)
                                                        RAWDAQ_SHOW_FIRSTS(CNTRIN_double)

                                                        cout << endl;

                                                    }
                                                    else
                                                    if ( classID  == CLASS_ID(CObservation6DFeatures) )
                                                    {
                                                        // ----------------------------------------------------------------------
                                                        //              CObservation6DFeatures
                                                        // ----------------------------------------------------------------------
                                                        CObservation6DFeaturesPtr obs = CObservation6DFeaturesPtr( sel_obj);
                                                        curSelectedObservation = CObservationPtr( sel_obj );
                                                        cout << endl;

                                                        cout << "Sensor pose: " << obs->sensorPose << endl;
                                                        cout << "Min range  : " << obs->minSensorDistance << endl;
                                                        cout << "Max range  : " << obs->maxSensorDistance << endl << endl;

                                                        cout << "Observation count : " << obs->sensedFeatures.size() << endl << endl;

                                                        for (size_t k=0;k<obs->sensedFeatures.size();k++)
                                                        {
                                                            const CObservation6DFeatures::TMeasurement & m = obs->sensedFeatures[k];
                                                            cout << "#" << k << ": ID=" << m.id << "; value=" << m.pose << "; inf=" <<m.inf_matrix.inMatlabFormat() << endl;
                                                        }
                                                    }
                                                    else
													{
														// Other selections:
														Notebook1->ChangeSelection( 0 );
													}



		} // scope for redirector
		memo->ShowPosition(0);
	}
	else
	{
		Notebook1->ChangeSelection( 0 );

		memo->Freeze();                 // Freeze the window to prevent scrollbar jumping
		memo->Clear();
		{
			CMyRedirector myRedirector( memo );

			// Show comments of the rawlog:
			string s;
			rawlog.getCommentText( s );

			if (s.empty())
			{
				cout << "(The rawlog has no comments)" << endl;
			}
			else
			{
				cout << s;
			}
		}
		// Set focus on the first line:
		memo->ShowPosition(0);
		memo->Thaw();  // Allow the window to redraw
	}

  }
  catch( utils::CExceptionExternalImageNotFound &e )
  {
	  wxMessageBox( _U(e.what()), _("Error with a delayed load image"), wxOK, this );

	  if (wxYES==wxMessageBox(
		  _U(format( "The current directory for relative images is:\n%s\n\nDo you want to set it to a different one?", CImage::IMAGES_PATH_BASE.c_str() ).c_str()),
		_("Error with delayed loading image"), wxYES_NO, this) )
	  {
			// Change CImage::IMAGES_PATH_BASE
			wxDirDialog dirDialog(
				this,
				_("Choose the base directory for relative image paths"),
				_U(CImage::IMAGES_PATH_BASE.c_str()), 0, wxDefaultPosition );
			if (dirDialog.ShowModal()==wxID_OK)
			{
				CImage::IMAGES_PATH_BASE = string( dirDialog.GetPath().mb_str() );
			}
	  }
  }
  catch(std::exception &e)
  {
		wxMessageBox( wxString(e.what(),wxConvUTF8), wxT("Exception"), wxOK, this);
  }
}

