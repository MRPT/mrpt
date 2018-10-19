/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
   */
#include "CPointsConfig.h"
#include "ui_CPointsConfig.h"
#include "TypeOfConfig.h"

CPointsConfig::CPointsConfig()
	: CBaseConfig(), m_ui(std::make_unique<Ui::CPointsConfig>())
{
	m_ui->setupUi(this);

	setInsertOpt();
	setLikelihoodOpt();
}

CPointsConfig::~CPointsConfig() = default;
const QString CPointsConfig::getName()
{
	return QString::fromStdString(typeToName(TypeOfConfig::PointsMap));
}

void CPointsConfig::updateConfiguration(
	mrpt::maps::TMetricMapInitializer* options)
{
	auto* mapDefination =
		dynamic_cast<mrpt::maps::CSimplePointsMap::TMapDefinition*>(options);
	ASSERT_(mapDefination);

	mapDefination->genericMapParams.enableSaveAs3DObject =
		m_ui->enableSaveAs3DObject;
	mapDefination->genericMapParams.enableObservationLikelihood =
		m_ui->enableObservationLikelihood;
	mapDefination->genericMapParams.enableObservationInsertion =
		m_ui->enableObservationInsertion;

	mapDefination->insertionOpts.minDistBetweenLaserPoints =
		m_ui->minDistBetweenLaserPoints->value();
	mapDefination->insertionOpts.addToExistingPointsMap =
		m_ui->addToExistingPointsMap->isChecked();
	mapDefination->insertionOpts.also_interpolate =
		m_ui->also_interpolate->isChecked();
	mapDefination->insertionOpts.disableDeletion =
		m_ui->disableDeletion->isChecked();
	mapDefination->insertionOpts.fuseWithExisting =
		m_ui->fuseWithExisting->isChecked();
	mapDefination->insertionOpts.isPlanarMap = m_ui->isPlanarMap->isChecked();
	mapDefination->insertionOpts.horizontalTolerance =
		m_ui->horizontalTolerance->value();
	mapDefination->insertionOpts.maxDistForInterpolatePoints =
		m_ui->maxDistForInterpolatePoints->value();
	mapDefination->insertionOpts.insertInvalidPoints =
		m_ui->insertInvalidPoints->isChecked();
}

TypeOfConfig CPointsConfig::type() const { return TypeOfConfig::PointsMap; }
void CPointsConfig::setInsertOpt(
	const mrpt::maps::CPointsMap::TInsertionOptions& insertOpt)
{
	m_ui->minDistBetweenLaserPoints->setValue(
		insertOpt.minDistBetweenLaserPoints);
	m_ui->addToExistingPointsMap->setChecked(insertOpt.addToExistingPointsMap);
	m_ui->also_interpolate->setChecked(insertOpt.also_interpolate);
	m_ui->disableDeletion->setChecked(insertOpt.disableDeletion);
	m_ui->fuseWithExisting->setChecked(insertOpt.fuseWithExisting);
	m_ui->isPlanarMap->setChecked(insertOpt.isPlanarMap);
	m_ui->horizontalTolerance->setValue(insertOpt.horizontalTolerance);
	m_ui->maxDistForInterpolatePoints->setValue(
		insertOpt.maxDistForInterpolatePoints);
	m_ui->insertInvalidPoints->setChecked(insertOpt.insertInvalidPoints);
}

void CPointsConfig::setLikelihoodOpt(
	const mrpt::maps::CPointsMap::TLikelihoodOptions& likelihoodOpt)
{
	m_ui->sigma_dist->setValue(likelihoodOpt.sigma_dist);
	m_ui->max_corr_distance->setValue(likelihoodOpt.max_corr_distance);
	m_ui->decimation->setValue(likelihoodOpt.decimation);
}
