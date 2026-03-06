/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+
   */
#include "CLandmarksConfig.h"

#include "TypeOfConfig.h"
#include "ui_CLandmarksConfig.h"

using namespace mrpt;
using namespace maps;

CLandmarksConfig::CLandmarksConfig() : CBaseConfig(), m_ui(std::make_unique<Ui::CLandmarksConfig>())
{
  m_ui->setupUi(this);
  // CLandmarksMap TInsertionOptions/TLikelihoodOptions removed in MRPT 3
  setInsertOpt();
  setLikelihoodOpt();
}

const QString CLandmarksConfig::getName()
{
  return QString::fromStdString(typeToName(TypeOfConfig::Landmarks));
}

void CLandmarksConfig::updateConfiguration(mrpt::maps::TMetricMapInitializer* options)
{
  // CLandmarksMap::TMapDefinition removed in MRPT 3 — stub implementation
  (void)options;

}

TypeOfConfig CLandmarksConfig::type() const { return TypeOfConfig::Landmarks; }
void CLandmarksConfig::setInsertOpt()
{
  // CLandmarksMap::TInsertionOptions removed in MRPT 3 — stub
}

void CLandmarksConfig::setLikelihoodOpt()
{
  // CLandmarksMap::TLikelihoodOptions removed in MRPT 3 — stub
}
