/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "gui-precomp.h"  // Precompiled headers

#include <mrpt/config.h>
#if MRPT_HAS_Qt5

#include "CAboutBoxQt.h"
#include <QVBoxLayout>
#include <QLabel>
#include <QTextEdit>
#include <QTabWidget>

CAboutBoxQt::CAboutBoxQt(
	const std::string& appName, const std::string& additionalInfo,
	const bool showStandardInfo)
	: CAboutBoxBase(appName, additionalInfo, showStandardInfo)
{
	auto lay = new QVBoxLayout();

	auto buildText = QString::fromStdString(MRPTBuildVersion());
	auto label = new QLabel(buildText, this);
	lay->addWidget(label);

	auto tabs = new QTabWidget(this);

	auto infoText = information("Qt", QT_VERSION_STR);
	tabs->addTab(widgetForTabs(infoText, tabs), tr("Information"));
	tabs->addTab(widgetForTabs(license(), tabs), tr("License"));
	tabs->addTab(widgetForTabs(tutorial(), tabs), tr("Tutorial"));

	lay->addWidget(tabs);
	setLayout(lay);
}

QTextEdit* CAboutBoxQt::widgetForTabs(
	const std::string& str, QTabWidget* parent) const
{
	auto text = QString::fromStdString(str);
	auto action = new QTextEdit(parent);
	action->setText(text);
	return action;
}

#endif  // MRPT_HAS_Qt5
