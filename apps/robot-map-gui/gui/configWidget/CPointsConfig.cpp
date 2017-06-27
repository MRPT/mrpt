/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "CPointsConfig.h"
#include "ui_CPointsConfig.h"

#include <mrpt/maps/CSimplePointsMap.h>


CPointsConfig::CPointsConfig(QWidget *parent)
	: CBaseConfig(parent)
	, m_ui(std::make_unique<Ui::CPointsConfig>())
{
	m_ui->setupUi(this);
}


CPointsConfig::~CPointsConfig()
{

}

const std::string CPointsConfig::getName()
{
	return "pointsMap";
}

void CPointsConfig::updateConfiguration(mrpt::maps::TMetricMapInitializer *options)
{
	mrpt::maps::CSimplePointsMap::TMapDefinition *mapDefination = dynamic_cast<mrpt::maps::CSimplePointsMap::TMapDefinition *>(options);
	assert(mapDefination);

	mapDefination->genericMapParams.enableSaveAs3DObject = m_ui->enableSaveAs3DObject;
	mapDefination->genericMapParams.enableObservationLikelihood = m_ui->enableObservationLikelihood;
	mapDefination->genericMapParams.enableObservationInsertion = m_ui->enableObservationInsertion;


	mapDefination->insertionOpts.minDistBetweenLaserPoints = m_ui->minDistBetweenLaserPoints.value();
	mapDefination->insertionOpts.addToExistingPointsMap = m_ui->addToExistingPointsMap->isChecked();
	mapDefination->insertionOpts.also_interpolate = m_ui->also_interpolate->isChecked();
	mapDefination->insertionOpts.disableDeletion = m_ui->disableDeletion->isChecked();
	mapDefination->insertionOpts.fuseWithExisting = m_ui->fuseWithExisting->isChecked();
	mapDefination->insertionOpts.isPlanarMap = m_ui->isPlanarMap->isChecked();
	mapDefination->insertionOpts.horizontalTolerance = m_ui->horizontalTolerance.value();
	mapDefination->insertionOpts.maxDistForInterpolatePoints = m_ui->maxDistForInterpolatePoints.value();
	mapDefination->insertionOpts.insertInvalidPoints = m_ui->insertInvalidPoints->isChecked();

	mapDefination->likelihoodOpts.sigma_dist = m_ui->sigma_dist.value();
	mapDefination->likelihoodOpts.max_corr_distance = m_ui->max_corr_distance.value();
	mapDefination->likelihoodOpts.decimation = m_ui->decimation.value();
}
