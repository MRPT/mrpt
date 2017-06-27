/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "CBeaconConfig.h"
#include "ui_CBeaconConfig.h"

#include <mrpt/maps/CBeaconMap.h>


CBeaconConfig::CBeaconConfig(QWidget *parent)
	: CBaseConfig(parent)
	, m_ui(std::make_unique<Ui::CBeaconConfig>())
{
	m_ui->setupUi(this);
}

CBeaconConfig::~CBeaconConfig()
{

}

void CBeaconConfig::updateConfiguration(mrpt::maps::TMetricMapInitializer *options)
{
	mrpt::maps::CBeaconMap::TMapDefinition *mapDefination = dynamic_cast<mrpt::maps::CBeaconMap::TMapDefinition *>(options);
	assert(mapDefination);

	mapDefination->genericMapParams.enableSaveAs3DObject = m_ui->enableSaveAs3DObject;
	mapDefination->genericMapParams.enableObservationLikelihood = m_ui->enableObservationLikelihood;
	mapDefination->genericMapParams.enableObservationInsertion = m_ui->enableObservationInsertion;

	mapDefination->insertionOpts.insertAsMonteCarlo = m_ui->insertAsMonteCarlo->isChecked();
	mapDefination->insertionOpts.maxElevation_deg = m_ui->maxElevation_deg->value();
	mapDefination->insertionOpts.minElevation_deg = m_ui->minElevation_deg->value();
	mapDefination->insertionOpts.MC_numSamplesPerMeter = m_ui->MC_numSamplesPerMeter->value();
	mapDefination->insertionOpts.MC_maxStdToGauss = m_ui->MC_maxStdToGauss->value();
	mapDefination->insertionOpts.MC_thresholdNegligible = m_ui->MC_thresholdNegligible->value();
	mapDefination->insertionOpts.MC_performResampling = m_ui->MC_performResampling->isChecked();
	mapDefination->insertionOpts.MC_afterResamplingNoise = m_ui->MC_afterResamplingNoise->value();
	mapDefination->insertionOpts.SOG_thresholdNegligible = m_ui->SOG_thresholdNegligible->value();
	mapDefination->insertionOpts.SOG_maxDistBetweenGaussians = m_ui->SOG_maxDistBetweenGaussians->value();
	mapDefination->insertionOpts.SOG_separationConstant = m_ui->SOG_separationConstant->value();
}

const std::string CBeaconConfig::getName()
{
	return "beaconMap";
}
