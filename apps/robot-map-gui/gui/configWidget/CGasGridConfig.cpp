/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
   */
#include "CGasGridConfig.h"
#include "ui_CGasGridConfig.h"
#include "TypeOfConfig.h"

using namespace mrpt;
using namespace maps;

CGasGridConfig::CGasGridConfig()
	: CBaseConfig(), m_ui(std::make_unique<Ui::CGasGridConfig>())
{
	m_ui->setupUi(this);
	auto* def = new CGasConcentrationGridMap2D::TMapDefinition();
	setCreationOpt(
		def->min_x, def->max_x, def->min_y, def->max_y, def->resolution);
	setInsertOpt(def->insertionOpts);
}

const QString CGasGridConfig::getName()
{
	return QString::fromStdString(typeToName(TypeOfConfig::GasGrid));
}

void CGasGridConfig::updateConfiguration(
	mrpt::maps::TMetricMapInitializer* options)
{
	auto* mapDefination =
		dynamic_cast<CGasConcentrationGridMap2D::TMapDefinition*>(options);
	ASSERT_(mapDefination);

	Q_UNUSED(options);
	mapDefination->min_x = m_ui->min_x->value();
	mapDefination->max_x = m_ui->max_x->value();
	mapDefination->min_y = m_ui->min_y->value();
	mapDefination->max_y = m_ui->max_y->value();
	mapDefination->resolution = m_ui->resolution->value();

	mapDefination->insertionOpts.gasSensorLabel =
		m_ui->gasSensorLabel->text().toStdString();
	mapDefination->insertionOpts.enose_id = m_ui->enose_id->value();
	mapDefination->insertionOpts.gasSensorType = m_ui->gasSensorType->value();
	mapDefination->insertionOpts.windSensorLabel =
		m_ui->windSensorLabel->text().toStdString();

	mapDefination->insertionOpts.useWindInformation =
		m_ui->useWindInformation->isChecked();
	mapDefination->insertionOpts.advectionFreq = m_ui->advectionFreq->value();
	mapDefination->insertionOpts.std_windNoise_phi =
		m_ui->std_windNoise_phi->value();
	mapDefination->insertionOpts.std_windNoise_mod =
		m_ui->std_windNoise_mod->value();
	mapDefination->insertionOpts.default_wind_direction =
		m_ui->default_wind_direction->value();
	mapDefination->insertionOpts.default_wind_speed =
		m_ui->default_wind_speed->value();
}

TypeOfConfig CGasGridConfig::type() const { return TypeOfConfig::GasGrid; }
void CGasGridConfig::setCreationOpt(
	float min_x, float max_x, float min_y, float max_y, float resolution)
{
	m_ui->min_x->setValue(min_x);
	m_ui->max_x->setValue(max_x);
	m_ui->min_y->setValue(min_y);
	m_ui->max_y->setValue(max_y);
	m_ui->resolution->setValue(resolution);
}

void CGasGridConfig::setInsertOpt(
	const mrpt::maps::CGasConcentrationGridMap2D::TInsertionOptions& insertOpt)
{
	m_ui->gasSensorLabel->setText(
		QString::fromStdString(insertOpt.gasSensorLabel));
	m_ui->enose_id->setValue(insertOpt.enose_id);
	m_ui->gasSensorType->setValue(insertOpt.gasSensorType);
	m_ui->windSensorLabel->setText(
		QString::fromStdString(insertOpt.windSensorLabel));

	m_ui->useWindInformation->setChecked(insertOpt.useWindInformation);
	m_ui->advectionFreq->setValue(insertOpt.advectionFreq);
	m_ui->std_windNoise_phi->setValue(insertOpt.std_windNoise_phi);
	m_ui->std_windNoise_mod->setValue(insertOpt.std_windNoise_mod);
	m_ui->default_wind_direction->setValue(insertOpt.default_wind_direction);
	m_ui->default_wind_speed->setValue(insertOpt.default_wind_speed);
}

void CGasGridConfig::setMapTypeOpt(
	const mrpt::maps::CRandomFieldGridMap2D::TMapRepresentation& /*mapType*/)
{
}
