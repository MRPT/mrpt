/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
   */
#include "CBeaconConfig.h"
#include "ui_CBeaconConfig.h"

using namespace mrpt;
using namespace maps;

CBeaconConfig::CBeaconConfig()
	: CBaseConfig(), m_ui(std::make_unique<Ui::CBeaconConfig>())
{
	m_ui->setupUi(this);

	setInsertOpt();
	setLikelihoodOpt();
}

void CBeaconConfig::updateConfiguration(
	mrpt::maps::TMetricMapInitializer* options)
{
	auto* mapDefination =
		dynamic_cast<mrpt::maps::CBeaconMap::TMapDefinition*>(options);
	ASSERT_(mapDefination);

	mapDefination->genericMapParams.enableSaveAs3DObject =
		m_ui->enableSaveAs3DObject;
	mapDefination->genericMapParams.enableObservationLikelihood =
		m_ui->enableObservationLikelihood;
	mapDefination->genericMapParams.enableObservationInsertion =
		m_ui->enableObservationInsertion;

	mapDefination->insertionOpts.insertAsMonteCarlo =
		m_ui->insertAsMonteCarlo->isChecked();
	mapDefination->insertionOpts.maxElevation_deg =
		m_ui->maxElevation_deg->value();
	mapDefination->insertionOpts.minElevation_deg =
		m_ui->minElevation_deg->value();
	mapDefination->insertionOpts.MC_numSamplesPerMeter =
		m_ui->MC_numSamplesPerMeter->value();
	mapDefination->insertionOpts.MC_maxStdToGauss =
		m_ui->MC_maxStdToGauss->value();
	mapDefination->insertionOpts.MC_thresholdNegligible =
		m_ui->MC_thresholdNegligible->value();
	mapDefination->insertionOpts.MC_performResampling =
		m_ui->MC_performResampling->isChecked();
	mapDefination->insertionOpts.MC_afterResamplingNoise =
		m_ui->MC_afterResamplingNoise->value();
	mapDefination->insertionOpts.SOG_thresholdNegligible =
		m_ui->SOG_thresholdNegligible->value();
	mapDefination->insertionOpts.SOG_maxDistBetweenGaussians =
		m_ui->SOG_maxDistBetweenGaussians->value();
	mapDefination->insertionOpts.SOG_separationConstant =
		m_ui->SOG_separationConstant->value();

	mapDefination->likelihoodOpts.rangeStd = m_ui->rangeStd->value();
}

const QString CBeaconConfig::getName()
{
	return QString::fromStdString(typeToName(TypeOfConfig::Beacon));
}

TypeOfConfig CBeaconConfig::type() const { return TypeOfConfig::Beacon; }
void CBeaconConfig::setInsertOpt(
	const mrpt::maps::CBeaconMap::TInsertionOptions& insertOpt)
{
	m_ui->insertAsMonteCarlo->setChecked(insertOpt.insertAsMonteCarlo);
	m_ui->maxElevation_deg->setValue(insertOpt.maxElevation_deg);
	m_ui->minElevation_deg->setValue(insertOpt.minElevation_deg);
	m_ui->MC_numSamplesPerMeter->setValue(insertOpt.MC_numSamplesPerMeter);
	m_ui->MC_maxStdToGauss->setValue(insertOpt.MC_maxStdToGauss);
	m_ui->MC_thresholdNegligible->setValue(insertOpt.MC_thresholdNegligible);
	m_ui->MC_performResampling->setChecked(insertOpt.MC_performResampling);
	m_ui->MC_afterResamplingNoise->setValue(insertOpt.MC_afterResamplingNoise);
	m_ui->SOG_thresholdNegligible->setValue(insertOpt.SOG_thresholdNegligible);
	m_ui->SOG_maxDistBetweenGaussians->setValue(
		insertOpt.SOG_maxDistBetweenGaussians);
	m_ui->SOG_separationConstant->setValue(insertOpt.SOG_separationConstant);
}

void CBeaconConfig::setLikelihoodOpt(
	const mrpt::maps::CBeaconMap::TLikelihoodOptions& likelihoodOpt)
{
	m_ui->rangeStd->setValue(likelihoodOpt.rangeStd);
}
