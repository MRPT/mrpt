/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "COccupancyConfig.h"
#include "ui_COccupancyConfig.h"

#include <mrpt/utils/CFileOutputStream.h>


using namespace mrpt;
using namespace maps;

COccupancyConfig::COccupancyConfig(QWidget *parent)
	: CBaseConfig(parent)
	, m_ui(std::make_unique<Ui::COccupancyConfig>())
{
	m_ui->setupUi(this);

	//connect(m_ui->mapAltitude, SIGNAL(valueChanged(double)), this, SLOT(mapAltitudeChanged(double)));
}

COccupancyConfig::~COccupancyConfig()
{

}

const std::string COccupancyConfig::getName()
{
	return "occupancyGrid";
}

void COccupancyConfig::updateConfiguration(TMetricMapInitializer *options)
{
	COccupancyGridMap2D::TMapDefinition *mapDefination = dynamic_cast<COccupancyGridMap2D::TMapDefinition *>(options);
	assert(mapDefination);

	mapDefination->genericMapParams.enableSaveAs3DObject = m_ui->enableSaveAs3DObject;
	mapDefination->genericMapParams.enableObservationLikelihood = m_ui->enableObservationLikelihood;
	mapDefination->genericMapParams.enableObservationInsertion = m_ui->enableObservationInsertion;

	mapDefination->min_x = m_ui->min_x->value();
	mapDefination->max_x= m_ui->max_x->value();
	mapDefination->min_y = m_ui->min_y->value();
	mapDefination->max_y = m_ui->max_y->value();

	mapDefination->insertionOpts.mapAltitude = m_ui->mapAltitude->value();
	mapDefination->insertionOpts.useMapAltitude = m_ui->useMapAltitude->isChecked();
	mapDefination->insertionOpts.maxDistanceInsertion = m_ui->maxDistanceInsertion->value();
	mapDefination->insertionOpts.maxOccupancyUpdateCertainty = m_ui->maxOccupancyUpdateCertainty->value();
	mapDefination->insertionOpts.considerInvalidRangesAsFreeSpace = m_ui->considerInvalidRangesAsFreeSpace->isChecked();
	mapDefination->insertionOpts.decimation = m_ui->decimation->value();
	mapDefination->insertionOpts.horizontalTolerance = m_ui->horizontalTolerance->value();
	mapDefination->insertionOpts.CFD_features_gaussian_size = m_ui->CFD_features_gaussian_size->value();
	mapDefination->insertionOpts.CFD_features_median_size = m_ui->CFD_features_median_size->value();
	mapDefination->insertionOpts.wideningBeamsWithDistance = m_ui->wideningBeamsWithDistance->isChecked();


	mapDefination->likelihoodOpts.likelihoodMethod = static_cast<COccupancyGridMap2D::TLikelihoodMethod>(m_ui->likelihoodMethod->value());

	mapDefination->likelihoodOpts.LF_stdHit = m_ui->LF_stdHit->value();
	mapDefination->likelihoodOpts.LF_zHit = m_ui->LF_zHit->value();
	mapDefination->likelihoodOpts.LF_zRandom = m_ui->LF_zRandom->value();
	mapDefination->likelihoodOpts.LF_maxRange = m_ui->LF_maxRange->value();
	mapDefination->likelihoodOpts.LF_decimation = m_ui->LF_decimation->value();
	mapDefination->likelihoodOpts.LF_maxCorrsDistance = m_ui->LF_maxCorrsDistance->value();
	mapDefination->likelihoodOpts.LF_useSquareDist = m_ui->LF_useSquareDist->isChecked();
	mapDefination->likelihoodOpts.LF_alternateAverageMethod = m_ui->LF_alternateAverageMethod->isChecked();
	mapDefination->likelihoodOpts.MI_exponent = m_ui->MI_exponent->value();
	mapDefination->likelihoodOpts.MI_skip_rays = m_ui->MI_skip_rays->value();
	mapDefination->likelihoodOpts.MI_ratio_max_distance = m_ui->MI_ratio_max_distance->value();
	mapDefination->likelihoodOpts.rayTracing_useDistanceFilter = m_ui->rayTracing_useDistanceFilter->isChecked();
	mapDefination->likelihoodOpts.rayTracing_decimation = m_ui->rayTracing_decimation->value();
	mapDefination->likelihoodOpts.rayTracing_stdHit = m_ui->rayTracing_stdHit->value();
	mapDefination->likelihoodOpts.consensus_takeEachRange = m_ui->consensus_takeEachRange->value();
	mapDefination->likelihoodOpts.consensus_pow = m_ui->consensus_pow->value();
	mapDefination->likelihoodOpts.enableLikelihoodCache = m_ui->enableLikelihoodCache->isChecked();
}



/*
void COccupancyConfig::
{
	m_map.insertionOptions.mapAltitude = d;
	m_map.insertionOptions.useMapAltitude = checked;
	m_map.insertionOptions.maxDistanceInsertion = d;
	m_map.insertionOptions.maxOccupancyUpdateCertainty = d;
	m_map.insertionOptions.considerInvalidRangesAsFreeSpace = checked;
	m_map.insertionOptions.decimation = decimation;
	m_map.insertionOptions.horizontalTolerance = d;
	m_map.insertionOptions.CFD_features_gaussian_size = d;
	m_map.insertionOptions.CFD_features_median_size = d;
	m_map.insertionOptions.wideningBeamsWithDistance = checked;
}*/
