/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
   */
#include "COccupancyConfig.h"
#include "ui_COccupancyConfig.h"
#include "TypeOfConfig.h"

#include <mrpt/io/CFileOutputStream.h>

using namespace mrpt;
using namespace maps;

COccupancyConfig::COccupancyConfig()
	: CBaseConfig(), m_ui(std::make_unique<Ui::COccupancyConfig>())
{
	m_ui->setupUi(this);

	m_ui->likelihoodMethod->addItem(
		"lmMeanInformation", COccupancyGridMap2D::lmMeanInformation);
	m_ui->likelihoodMethod->addItem(
		"lmRayTracing", COccupancyGridMap2D::lmRayTracing);
	m_ui->likelihoodMethod->addItem(
		"lmConsensus", COccupancyGridMap2D::lmConsensus);
	m_ui->likelihoodMethod->addItem(
		"lmCellsDifference", COccupancyGridMap2D::lmCellsDifference);
	m_ui->likelihoodMethod->addItem(
		"lmLikelihoodField_Thrun",
		COccupancyGridMap2D::lmLikelihoodField_Thrun);
	m_ui->likelihoodMethod->addItem(
		"lmLikelihoodField_II", COccupancyGridMap2D::lmLikelihoodField_II);
	m_ui->likelihoodMethod->addItem(
		"lmConsensusOWA", COccupancyGridMap2D::lmConsensusOWA);

	auto* def = new COccupancyGridMap2D::TMapDefinition();
	setCreationOpt(
		def->min_x, def->max_x, def->min_y, def->max_y, def->resolution);
	setInsertOpt(def->insertionOpts);
	setLikelihoodOpt(def->likelihoodOpts);
}

const QString COccupancyConfig::getName()
{
	return QString::fromStdString(typeToName(TypeOfConfig::Occupancy));
}

void COccupancyConfig::updateConfiguration(TMetricMapInitializer* options)
{
	auto* mapDefination =
		dynamic_cast<COccupancyGridMap2D::TMapDefinition*>(options);
	ASSERT_(mapDefination);

	mapDefination->genericMapParams.enableSaveAs3DObject =
		m_ui->enableSaveAs3DObject;
	mapDefination->genericMapParams.enableObservationLikelihood =
		m_ui->enableObservationLikelihood;
	mapDefination->genericMapParams.enableObservationInsertion =
		m_ui->enableObservationInsertion;

	mapDefination->min_x = m_ui->min_x->value();
	mapDefination->max_x = m_ui->max_x->value();
	mapDefination->min_y = m_ui->min_y->value();
	mapDefination->max_y = m_ui->max_y->value();
	mapDefination->resolution = m_ui->resolution->value();

	mapDefination->insertionOpts.mapAltitude = m_ui->mapAltitude->value();
	mapDefination->insertionOpts.useMapAltitude =
		m_ui->useMapAltitude->isChecked();
	mapDefination->insertionOpts.maxDistanceInsertion =
		m_ui->maxDistanceInsertion->value();
	mapDefination->insertionOpts.maxOccupancyUpdateCertainty =
		m_ui->maxOccupancyUpdateCertainty->value();
	mapDefination->insertionOpts.considerInvalidRangesAsFreeSpace =
		m_ui->considerInvalidRangesAsFreeSpace->isChecked();
	mapDefination->insertionOpts.decimation = m_ui->decimation->value();
	mapDefination->insertionOpts.horizontalTolerance =
		m_ui->horizontalTolerance->value();
	mapDefination->insertionOpts.CFD_features_gaussian_size =
		m_ui->CFD_features_gaussian_size->value();
	mapDefination->insertionOpts.CFD_features_median_size =
		m_ui->CFD_features_median_size->value();
	mapDefination->insertionOpts.wideningBeamsWithDistance =
		m_ui->wideningBeamsWithDistance->isChecked();

	mapDefination->likelihoodOpts.likelihoodMethod =
		static_cast<COccupancyGridMap2D::TLikelihoodMethod>(
			m_ui->likelihoodMethod->currentData().toInt());
	mapDefination->likelihoodOpts.LF_stdHit = m_ui->LF_stdHit->value();
	mapDefination->likelihoodOpts.LF_zHit = m_ui->LF_zHit->value();
	mapDefination->likelihoodOpts.LF_zRandom = m_ui->LF_zRandom->value();
	mapDefination->likelihoodOpts.LF_maxRange = m_ui->LF_maxRange->value();
	mapDefination->likelihoodOpts.LF_decimation = m_ui->LF_decimation->value();
	mapDefination->likelihoodOpts.LF_maxCorrsDistance =
		m_ui->LF_maxCorrsDistance->value();
	mapDefination->likelihoodOpts.LF_useSquareDist =
		m_ui->LF_useSquareDist->isChecked();
	mapDefination->likelihoodOpts.LF_alternateAverageMethod =
		m_ui->LF_alternateAverageMethod->isChecked();
	mapDefination->likelihoodOpts.MI_exponent = m_ui->MI_exponent->value();
	mapDefination->likelihoodOpts.MI_skip_rays = m_ui->MI_skip_rays->value();
	mapDefination->likelihoodOpts.MI_ratio_max_distance =
		m_ui->MI_ratio_max_distance->value();
	mapDefination->likelihoodOpts.rayTracing_useDistanceFilter =
		m_ui->rayTracing_useDistanceFilter->isChecked();
	mapDefination->likelihoodOpts.rayTracing_decimation =
		m_ui->rayTracing_decimation->value();
	mapDefination->likelihoodOpts.rayTracing_stdHit =
		m_ui->rayTracing_stdHit->value();
	mapDefination->likelihoodOpts.consensus_takeEachRange =
		m_ui->consensus_takeEachRange->value();
	mapDefination->likelihoodOpts.consensus_pow = m_ui->consensus_pow->value();
	mapDefination->likelihoodOpts.enableLikelihoodCache =
		m_ui->enableLikelihoodCache->isChecked();
}

TypeOfConfig COccupancyConfig::type() const { return TypeOfConfig::Occupancy; }
void COccupancyConfig::setCreationOpt(
	float min_x, float max_x, float min_y, float max_y, float resolution)
{
	m_ui->min_x->setValue(min_x);
	m_ui->max_x->setValue(max_x);
	m_ui->min_y->setValue(min_y);
	m_ui->max_y->setValue(max_y);
	m_ui->resolution->setValue(resolution);
}

void COccupancyConfig::setInsertOpt(
	const COccupancyGridMap2D::TInsertionOptions& insertOpt)
{
	m_ui->mapAltitude->setValue(insertOpt.mapAltitude);
	m_ui->useMapAltitude->setChecked(insertOpt.useMapAltitude);
	m_ui->maxDistanceInsertion->setValue(insertOpt.maxDistanceInsertion);
	m_ui->maxOccupancyUpdateCertainty->setValue(
		insertOpt.maxOccupancyUpdateCertainty);
	m_ui->considerInvalidRangesAsFreeSpace->setChecked(
		insertOpt.considerInvalidRangesAsFreeSpace);
	m_ui->decimation->setValue(insertOpt.decimation);
	m_ui->horizontalTolerance->setValue(insertOpt.horizontalTolerance);
	m_ui->CFD_features_gaussian_size->setValue(
		insertOpt.CFD_features_gaussian_size);
	m_ui->CFD_features_median_size->setValue(
		insertOpt.CFD_features_median_size);
	m_ui->wideningBeamsWithDistance->setChecked(
		insertOpt.wideningBeamsWithDistance);
}

void COccupancyConfig::setLikelihoodOpt(
	const COccupancyGridMap2D::TLikelihoodOptions& likelihoodOpt)
{
	m_ui->likelihoodMethod->setCurrentIndex(likelihoodOpt.likelihoodMethod);
	m_ui->LF_stdHit->setValue(likelihoodOpt.LF_stdHit);
	m_ui->LF_zHit->setValue(likelihoodOpt.LF_zHit);
	m_ui->LF_zRandom->setValue(likelihoodOpt.LF_zRandom);
	m_ui->LF_maxRange->setValue(likelihoodOpt.LF_maxRange);
	m_ui->LF_decimation->setValue(likelihoodOpt.LF_decimation);
	m_ui->LF_maxCorrsDistance->setValue(likelihoodOpt.LF_maxCorrsDistance);
	m_ui->LF_useSquareDist->setChecked(likelihoodOpt.LF_useSquareDist);
	m_ui->LF_alternateAverageMethod->setChecked(
		likelihoodOpt.LF_alternateAverageMethod);
	m_ui->MI_exponent->setValue(likelihoodOpt.MI_exponent);
	m_ui->MI_skip_rays->setValue(likelihoodOpt.MI_skip_rays);
	m_ui->MI_ratio_max_distance->setValue(likelihoodOpt.MI_ratio_max_distance);
	m_ui->rayTracing_useDistanceFilter->setChecked(
		likelihoodOpt.rayTracing_useDistanceFilter);
	m_ui->rayTracing_decimation->setValue(likelihoodOpt.rayTracing_decimation);
	m_ui->rayTracing_stdHit->setValue(likelihoodOpt.rayTracing_stdHit);
	m_ui->consensus_takeEachRange->setValue(
		likelihoodOpt.consensus_takeEachRange);
	m_ui->consensus_pow->setValue(likelihoodOpt.consensus_pow);
	m_ui->enableLikelihoodCache->setChecked(
		likelihoodOpt.enableLikelihoodCache);
}
