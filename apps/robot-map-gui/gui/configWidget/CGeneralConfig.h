/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once
#include "CBaseConfig.h"
#include "ui_CGeneralConfig.h"

#include <memory>


class CGeneralConfig: public CBaseConfig
{
	Q_OBJECT
public:
	CGeneralConfig();
	virtual ~CGeneralConfig() = default;

	virtual const QString getName() override;
	virtual void updateConfiguration(mrpt::maps::TMetricMapInitializer *options) override;
	virtual TypeOfConfig type() const override;

signals:
	void backgroundColorChanged(QColor);
	void gridColorChanged(QColor);
	void gridVisibleChanged(bool is);
	void currentBotChanged(int value);

private slots:
	void openGridColor();
	void openBackgroundColor();
	void stateChanged(int state);

private:
	std::unique_ptr<Ui::CGeneralConfig> m_ui;
};
