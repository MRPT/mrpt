/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
   */
#include "CLandmarksConfig.h"
#include "ui_CLandmarksConfig.h"
#include "TypeOfConfig.h"

using namespace mrpt;
using namespace maps;
using namespace vision;

CLandmarksConfig::CLandmarksConfig()
	: CBaseConfig(), m_ui(std::make_unique<Ui::CLandmarksConfig>())
{
	m_ui->setupUi(this);
	m_ui->TFeatureType->addItem("featNotDefined", TFeatureType::featNotDefined);
	m_ui->TFeatureType->addItem("featKLT", TFeatureType::featKLT);
	m_ui->TFeatureType->addItem("featHarris", TFeatureType::featHarris);
	m_ui->TFeatureType->addItem("featBCD", TFeatureType::featBCD);
	m_ui->TFeatureType->addItem("featSIFT", TFeatureType::featSIFT);
	m_ui->TFeatureType->addItem("featSURF", TFeatureType::featSURF);
	m_ui->TFeatureType->addItem("featBeacon", TFeatureType::featBeacon);
	m_ui->TFeatureType->addItem("featFAST", TFeatureType::featFAST);
	m_ui->TFeatureType->addItem("featFASTER9", TFeatureType::featFASTER9);
	m_ui->TFeatureType->addItem("featFASTER10", TFeatureType::featFASTER10);
	m_ui->TFeatureType->addItem("featFASTER12", TFeatureType::featFASTER12);
	m_ui->TFeatureType->addItem("featORB", TFeatureType::featORB);

	m_ui->implementation->addItem("LoweBinary", CFeatureExtraction::LoweBinary);
	m_ui->implementation->addItem("CSBinary", CFeatureExtraction::CSBinary);
	m_ui->implementation->addItem(
		"VedaldiBinary", CFeatureExtraction::VedaldiBinary);
	m_ui->implementation->addItem("Hess", CFeatureExtraction::Hess);
	m_ui->implementation->addItem("OpenCV", CFeatureExtraction::OpenCV);

	setInsertOpt();
	setLikelihoodOpt();
}

const QString CLandmarksConfig::getName()
{
	return QString::fromStdString(typeToName(TypeOfConfig::Landmarks));
}

void CLandmarksConfig::updateConfiguration(
	mrpt::maps::TMetricMapInitializer* options)
{
	auto* mapDefination = dynamic_cast<CLandmarksMap::TMapDefinition*>(options);
	ASSERT_(mapDefination);

	mapDefination->genericMapParams.enableSaveAs3DObject =
		m_ui->enableSaveAs3DObject;
	mapDefination->genericMapParams.enableObservationLikelihood =
		m_ui->enableObservationLikelihood;
	mapDefination->genericMapParams.enableObservationInsertion =
		m_ui->enableObservationInsertion;

	mapDefination->insertionOpts.insert_SIFTs_from_monocular_images =
		m_ui->insert_SIFTs_from_monocular_images->isChecked();
	mapDefination->insertionOpts.insert_SIFTs_from_stereo_images =
		m_ui->insert_SIFTs_from_stereo_images->isChecked();
	mapDefination->insertionOpts.insert_Landmarks_from_range_scans =
		m_ui->insert_Landmarks_from_range_scans->isChecked();

	mapDefination->insertionOpts.SiftCorrRatioThreshold =
		m_ui->SiftCorrRatioThreshold->value();
	mapDefination->insertionOpts.SiftLikelihoodThreshold =
		m_ui->SiftLikelihoodThreshold->value();
	mapDefination->insertionOpts.SiftEDDThreshold =
		m_ui->SiftEDDThreshold->value();
	mapDefination->insertionOpts.SIFTMatching3DMethod =
		m_ui->SIFTMatching3DMethod->value();

	mapDefination->insertionOpts.SIFTsLoadDistanceOfTheMean =
		m_ui->SIFTsLoadDistanceOfTheMean->value();
	mapDefination->insertionOpts.SIFTsLoadEllipsoidWidth =
		m_ui->SIFTsLoadEllipsoidWidth->value();
	mapDefination->insertionOpts.SIFTs_stdXY = m_ui->SIFTs_stdXY->value();
	mapDefination->insertionOpts.SIFTs_stdDisparity =
		m_ui->SIFTs_stdDisparity->value();
	mapDefination->insertionOpts.SIFTs_numberOfKLTKeypoints =
		m_ui->SIFTs_numberOfKLTKeypoints->value();
	mapDefination->insertionOpts.SIFTs_stereo_maxDepth =
		m_ui->SIFTs_stereo_maxDepth->value();
	mapDefination->insertionOpts.SIFTs_epipolar_TH =
		m_ui->SIFTs_epipolar_TH->value();
	mapDefination->insertionOpts.PLOT_IMAGES = m_ui->PLOT_IMAGES->isChecked();

	mapDefination->likelihoodOpts.rangeScan2D_decimation =
		m_ui->rangeScan2D_decimation->value();
	mapDefination->likelihoodOpts.SIFTs_sigma_euclidean_dist =
		m_ui->SIFTs_sigma_euclidean_dist->value();
	mapDefination->likelihoodOpts.SIFTs_sigma_descriptor_dist =
		m_ui->SIFTs_sigma_descriptor_dist->value();
	mapDefination->likelihoodOpts.SIFTs_mahaDist_std =
		m_ui->SIFTs_mahaDist_std->value();
	mapDefination->likelihoodOpts.SIFTnullCorrespondenceDistance =
		m_ui->SIFTnullCorrespondenceDistance->value();
	mapDefination->likelihoodOpts.SIFTs_decimation =
		m_ui->SIFTs_decimation->value();  // int

	mapDefination->likelihoodOpts.beaconRangesStd =
		m_ui->beaconRangesStd->value();
	mapDefination->likelihoodOpts.beaconRangesUseObservationStd =
		m_ui->beaconRangesUseObservationStd->isChecked();
	mapDefination->likelihoodOpts.extRobotPoseStd =
		m_ui->extRobotPoseStd->value();
	mapDefination->likelihoodOpts.GPS_sigma = m_ui->GPS_sigma->value();

	mapDefination->likelihoodOpts.GPSOrigin.longitude =
		m_ui->longitude->value();
	mapDefination->likelihoodOpts.GPSOrigin.latitude = m_ui->latitude->value();
	mapDefination->likelihoodOpts.GPSOrigin.altitude = m_ui->altitude->value();
	mapDefination->likelihoodOpts.GPSOrigin.ang = m_ui->ang->value();
	mapDefination->likelihoodOpts.GPSOrigin.x_shift = m_ui->x_shift->value();
	mapDefination->likelihoodOpts.GPSOrigin.y_shift = m_ui->y_shift->value();
	mapDefination->likelihoodOpts.GPSOrigin.min_sat = m_ui->min_sat->value();

	mapDefination->likelihoodOpts.SIFT_feat_options.featsType =
		static_cast<TFeatureType>(m_ui->TFeatureType->currentData().toInt());
	mapDefination->likelihoodOpts.SIFT_feat_options.patchSize =
		m_ui->patchSize->value();

	mapDefination->likelihoodOpts.SIFT_feat_options.useMask =
		m_ui->useMask->isChecked();
	mapDefination->likelihoodOpts.SIFT_feat_options.addNewFeatures =
		m_ui->addNewFeatures->isChecked();
	mapDefination->likelihoodOpts.SIFT_feat_options.FIND_SUBPIXEL =
		m_ui->FIND_SUBPIXEL->isChecked();

	mapDefination->likelihoodOpts.SIFT_feat_options.KLTOptions.radius =
		m_ui->radiusKLTOptions->value();
	mapDefination->likelihoodOpts.SIFT_feat_options.KLTOptions.threshold =
		m_ui->thresholdKLTOptions->value();
	mapDefination->likelihoodOpts.SIFT_feat_options.KLTOptions.min_distance =
		m_ui->min_distanceKLTOptions->value();
	mapDefination->likelihoodOpts.SIFT_feat_options.KLTOptions.tile_image =
		m_ui->tile_imageKLTOptions->isChecked();

	mapDefination->likelihoodOpts.SIFT_feat_options.harrisOptions.threshold =
		m_ui->thresholdHarris->value();
	mapDefination->likelihoodOpts.SIFT_feat_options.harrisOptions.k =
		m_ui->k->value();
	mapDefination->likelihoodOpts.SIFT_feat_options.harrisOptions.sigma =
		m_ui->sigma->value();
	mapDefination->likelihoodOpts.SIFT_feat_options.harrisOptions.radius =
		m_ui->radiusHarris->value();
	mapDefination->likelihoodOpts.SIFT_feat_options.harrisOptions.min_distance =
		m_ui->min_distanceHarris->value();
	mapDefination->likelihoodOpts.SIFT_feat_options.harrisOptions.tile_image =
		m_ui->tile_imageHarris->isChecked();

	mapDefination->likelihoodOpts.SIFT_feat_options.FASTOptions.threshold =
		m_ui->thresholdFASTOptions->value();
	mapDefination->likelihoodOpts.SIFT_feat_options.FASTOptions.min_distance =
		m_ui->min_distanceFASTOptions->value();
	mapDefination->likelihoodOpts.SIFT_feat_options.FASTOptions
		.nonmax_suppression = m_ui->nonmax_suppression->isChecked();
	mapDefination->likelihoodOpts.SIFT_feat_options.FASTOptions
		.use_KLT_response = m_ui->use_KLT_response->isChecked();

	mapDefination->likelihoodOpts.SIFT_feat_options.ORBOptions.n_levels =
		m_ui->n_levels->value();
	mapDefination->likelihoodOpts.SIFT_feat_options.ORBOptions.min_distance =
		m_ui->min_distanceORBOptions->value();
	mapDefination->likelihoodOpts.SIFT_feat_options.ORBOptions.scale_factor =
		m_ui->scale_factor->value();
	mapDefination->likelihoodOpts.SIFT_feat_options.ORBOptions.extract_patch =
		m_ui->extract_patch->isChecked();

	mapDefination->likelihoodOpts.SIFT_feat_options.SIFTOptions.implementation =
		static_cast<CFeatureExtraction::TSIFTImplementation>(
			m_ui->implementation->currentData().toInt());
	mapDefination->likelihoodOpts.SIFT_feat_options.SIFTOptions.threshold =
		m_ui->thresholdSIFTOptions->value();
	mapDefination->likelihoodOpts.SIFT_feat_options.SIFTOptions.edgeThreshold =
		m_ui->edgeThreshold->value();

	mapDefination->likelihoodOpts.SIFT_feat_options.SURFOptions
		.rotation_invariant = m_ui->rotation_invariant->isChecked();
	mapDefination->likelihoodOpts.SIFT_feat_options.SURFOptions
		.hessianThreshold = m_ui->hessianThreshold->value();
	mapDefination->likelihoodOpts.SIFT_feat_options.SURFOptions.nOctaves =
		m_ui->nOctaves->value();
	mapDefination->likelihoodOpts.SIFT_feat_options.SURFOptions
		.nLayersPerOctave = m_ui->nLayersPerOctave->value();

	mapDefination->likelihoodOpts.SIFT_feat_options.SpinImagesOptions
		.hist_size_intensity = m_ui->hist_size_intensity->value();
	mapDefination->likelihoodOpts.SIFT_feat_options.SpinImagesOptions
		.hist_size_distance = m_ui->hist_size_distance->value();
	mapDefination->likelihoodOpts.SIFT_feat_options.SpinImagesOptions.std_dist =
		m_ui->std_dist->value();
	mapDefination->likelihoodOpts.SIFT_feat_options.SpinImagesOptions
		.std_intensity = m_ui->std_intensity->value();
	mapDefination->likelihoodOpts.SIFT_feat_options.SpinImagesOptions.radius =
		m_ui->radiusSpinImagesOptions->value();

	mapDefination->likelihoodOpts.SIFT_feat_options.PolarImagesOptions
		.bins_angle = m_ui->bins_angle->value();
	mapDefination->likelihoodOpts.SIFT_feat_options.PolarImagesOptions
		.bins_distance = m_ui->bins_distance->value();
	mapDefination->likelihoodOpts.SIFT_feat_options.PolarImagesOptions.radius =
		m_ui->radiusPolarImagesOptions->value();

	mapDefination->likelihoodOpts.SIFT_feat_options.LogPolarImagesOptions
		.radius = m_ui->radiusLogPolarImagesOptions->value();
	mapDefination->likelihoodOpts.SIFT_feat_options.LogPolarImagesOptions
		.num_angles = m_ui->num_angles->value();
	mapDefination->likelihoodOpts.SIFT_feat_options.LogPolarImagesOptions
		.rho_scale = m_ui->rho_scale->value();
}

TypeOfConfig CLandmarksConfig::type() const { return TypeOfConfig::Landmarks; }
void CLandmarksConfig::setInsertOpt(
	const CLandmarksMap::TInsertionOptions& insertOpt)
{
	m_ui->insert_SIFTs_from_monocular_images->setChecked(
		insertOpt.insert_SIFTs_from_monocular_images);
	m_ui->insert_SIFTs_from_stereo_images->setChecked(
		insertOpt.insert_SIFTs_from_stereo_images);
	m_ui->insert_Landmarks_from_range_scans->setChecked(
		insertOpt.insert_Landmarks_from_range_scans);

	m_ui->SiftCorrRatioThreshold->setValue(
		insertOpt.insert_Landmarks_from_range_scans);
	m_ui->SiftLikelihoodThreshold->setValue(
		insertOpt.insert_Landmarks_from_range_scans);
	m_ui->SiftEDDThreshold->setValue(
		insertOpt.insert_Landmarks_from_range_scans);
	m_ui->SIFTMatching3DMethod->setValue(insertOpt.SIFTMatching3DMethod);

	m_ui->SIFTsLoadDistanceOfTheMean->setValue(
		insertOpt.SIFTsLoadDistanceOfTheMean);
	m_ui->SIFTsLoadEllipsoidWidth->setValue(insertOpt.SIFTsLoadEllipsoidWidth);
	m_ui->SIFTs_stdXY->setValue(insertOpt.SIFTs_stdXY);
	m_ui->SIFTs_stdDisparity->setValue(insertOpt.SIFTs_stdDisparity);
	m_ui->SIFTs_numberOfKLTKeypoints->setValue(
		insertOpt.SIFTs_numberOfKLTKeypoints);
	m_ui->SIFTs_stereo_maxDepth->setValue(insertOpt.SIFTs_stereo_maxDepth);
	m_ui->SIFTs_epipolar_TH->setValue(insertOpt.SIFTs_epipolar_TH);
	m_ui->PLOT_IMAGES->setChecked(insertOpt.PLOT_IMAGES);
}

void CLandmarksConfig::setLikelihoodOpt(
	const CLandmarksMap::TLikelihoodOptions& likelihoodOpt)
{
	m_ui->rangeScan2D_decimation->setValue(
		likelihoodOpt.rangeScan2D_decimation);
	m_ui->SIFTs_sigma_euclidean_dist->setValue(
		likelihoodOpt.SIFTs_sigma_euclidean_dist);
	m_ui->SIFTs_sigma_descriptor_dist->setValue(
		likelihoodOpt.SIFTs_sigma_descriptor_dist);
	m_ui->SIFTs_mahaDist_std->setValue(likelihoodOpt.SIFTs_mahaDist_std);
	m_ui->SIFTnullCorrespondenceDistance->setValue(
		likelihoodOpt.SIFTnullCorrespondenceDistance);
	m_ui->SIFTs_decimation->setValue(likelihoodOpt.SIFTs_decimation);

	m_ui->beaconRangesStd->setValue(likelihoodOpt.beaconRangesStd);
	m_ui->beaconRangesUseObservationStd->setChecked(
		likelihoodOpt.beaconRangesUseObservationStd);
	m_ui->extRobotPoseStd->setValue(likelihoodOpt.extRobotPoseStd);
	m_ui->GPS_sigma->setValue(likelihoodOpt.GPS_sigma);

	m_ui->longitude->setValue(likelihoodOpt.GPSOrigin.longitude);
	m_ui->latitude->setValue(likelihoodOpt.GPSOrigin.latitude);
	m_ui->altitude->setValue(likelihoodOpt.GPSOrigin.altitude);
	m_ui->ang->setValue(likelihoodOpt.GPSOrigin.ang);
	m_ui->x_shift->setValue(likelihoodOpt.GPSOrigin.x_shift);
	m_ui->y_shift->setValue(likelihoodOpt.GPSOrigin.y_shift);
	m_ui->min_sat->setValue(likelihoodOpt.GPSOrigin.min_sat);

	m_ui->TFeatureType->setCurrentIndex(
		likelihoodOpt.SIFT_feat_options.featsType + 1);

	m_ui->patchSize->setValue(likelihoodOpt.SIFT_feat_options.patchSize);

	m_ui->useMask->setChecked(likelihoodOpt.SIFT_feat_options.useMask);
	m_ui->addNewFeatures->setChecked(
		likelihoodOpt.SIFT_feat_options.addNewFeatures);
	m_ui->FIND_SUBPIXEL->setChecked(
		likelihoodOpt.SIFT_feat_options.FIND_SUBPIXEL);

	m_ui->radiusKLTOptions->setValue(
		likelihoodOpt.SIFT_feat_options.KLTOptions.radius);
	m_ui->thresholdKLTOptions->setValue(
		likelihoodOpt.SIFT_feat_options.KLTOptions.threshold);
	m_ui->min_distanceKLTOptions->setValue(
		likelihoodOpt.SIFT_feat_options.KLTOptions.min_distance);
	m_ui->tile_imageKLTOptions->setChecked(
		likelihoodOpt.SIFT_feat_options.KLTOptions.tile_image);

	m_ui->thresholdHarris->setValue(
		likelihoodOpt.SIFT_feat_options.harrisOptions.threshold);
	m_ui->k->setValue(likelihoodOpt.SIFT_feat_options.harrisOptions.k);
	m_ui->sigma->setValue(likelihoodOpt.SIFT_feat_options.harrisOptions.sigma);
	m_ui->radiusHarris->setValue(
		likelihoodOpt.SIFT_feat_options.harrisOptions.radius);
	m_ui->min_distanceHarris->setValue(
		likelihoodOpt.SIFT_feat_options.harrisOptions.min_distance);
	m_ui->tile_imageHarris->setChecked(
		likelihoodOpt.SIFT_feat_options.harrisOptions.tile_image);

	m_ui->thresholdFASTOptions->setValue(
		likelihoodOpt.SIFT_feat_options.FASTOptions.threshold);
	m_ui->min_distanceFASTOptions->setValue(
		likelihoodOpt.SIFT_feat_options.FASTOptions.min_distance);
	m_ui->nonmax_suppression->setChecked(
		likelihoodOpt.SIFT_feat_options.FASTOptions.nonmax_suppression);
	m_ui->use_KLT_response->setChecked(
		likelihoodOpt.SIFT_feat_options.FASTOptions.use_KLT_response);

	m_ui->n_levels->setValue(
		likelihoodOpt.SIFT_feat_options.ORBOptions.n_levels);
	m_ui->min_distanceORBOptions->setValue(
		likelihoodOpt.SIFT_feat_options.ORBOptions.min_distance);
	m_ui->scale_factor->setValue(
		likelihoodOpt.SIFT_feat_options.ORBOptions.scale_factor);
	m_ui->extract_patch->setChecked(
		likelihoodOpt.SIFT_feat_options.ORBOptions.extract_patch);

	m_ui->implementation->setCurrentIndex(
		likelihoodOpt.SIFT_feat_options.SIFTOptions.implementation);

	m_ui->thresholdSIFTOptions->setValue(
		likelihoodOpt.SIFT_feat_options.SIFTOptions.threshold);
	m_ui->edgeThreshold->setValue(
		likelihoodOpt.SIFT_feat_options.SIFTOptions.edgeThreshold);

	m_ui->rotation_invariant->setChecked(
		likelihoodOpt.SIFT_feat_options.SURFOptions.rotation_invariant);
	m_ui->hessianThreshold->setValue(
		likelihoodOpt.SIFT_feat_options.SURFOptions.hessianThreshold);
	m_ui->nOctaves->setValue(
		likelihoodOpt.SIFT_feat_options.SURFOptions.nOctaves);
	m_ui->nLayersPerOctave->setValue(
		likelihoodOpt.SIFT_feat_options.SURFOptions.nLayersPerOctave);

	m_ui->hist_size_intensity->setValue(
		likelihoodOpt.SIFT_feat_options.SpinImagesOptions.hist_size_intensity);
	m_ui->hist_size_distance->setValue(
		likelihoodOpt.SIFT_feat_options.SpinImagesOptions.hist_size_distance);
	m_ui->std_dist->setValue(
		likelihoodOpt.SIFT_feat_options.SpinImagesOptions.std_dist);
	m_ui->std_intensity->setValue(
		likelihoodOpt.SIFT_feat_options.SpinImagesOptions.std_intensity);
	m_ui->radiusSpinImagesOptions->setValue(
		likelihoodOpt.SIFT_feat_options.SpinImagesOptions.radius);

	m_ui->bins_angle->setValue(
		likelihoodOpt.SIFT_feat_options.PolarImagesOptions.bins_angle);
	m_ui->bins_distance->setValue(
		likelihoodOpt.SIFT_feat_options.PolarImagesOptions.bins_distance);
	m_ui->radiusPolarImagesOptions->setValue(
		likelihoodOpt.SIFT_feat_options.PolarImagesOptions.radius);

	m_ui->radiusLogPolarImagesOptions->setValue(
		likelihoodOpt.SIFT_feat_options.LogPolarImagesOptions.radius);
	m_ui->num_angles->setValue(
		likelihoodOpt.SIFT_feat_options.LogPolarImagesOptions.num_angles);
	m_ui->rho_scale->setValue(
		likelihoodOpt.SIFT_feat_options.LogPolarImagesOptions.rho_scale);
}
