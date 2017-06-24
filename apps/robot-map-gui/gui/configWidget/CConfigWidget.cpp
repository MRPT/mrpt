/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "CConfigWidget.h"
#include "ui_CConfigWidget.h"

#include <QFileDialog>
#include <QMessageBox>


CConfigWidget::CConfigWidget(QWidget *parent)
	: QWidget(parent)
	, m_ui(std::make_unique<Ui::CConfigWidget>())
{
	m_ui->setupUi(this);
	QObject::connect(m_ui->m_loadConfig, SIGNAL(released()), SLOT(openConfig()));
	QObject::connect(m_ui->m_saveConfig, SIGNAL(released()), SLOT(saveConfig()));
}

CConfigWidget::~CConfigWidget()
{

}

void CConfigWidget::openConfig()
{
	QString configName = QFileDialog::getOpenFileName(this, tr("Open Config File"), "", tr("Files (*.ini)"));
	if (configName.isEmpty())
		return;
	QFile file(configName);
	if (!file.open(QIODevice::ReadOnly))
	{
		QMessageBox::information(this, tr("Unable to open file"), file.errorString());
		return;
	}


}

void CConfigWidget::saveConfig()
{
	QString configName = QFileDialog::getSaveFileName(this, QObject::tr("Save Config File"), "", "Files (*.ini)");
	if (configName.isEmpty())
		return;

	QFile file(configName);
	if (!file.open(QIODevice::WriteOnly))
	{
		QMessageBox::information(this, tr("Unable to open file"), file.errorString());
		return;
	}

	QDataStream out(&file);
	out.setVersion(QDataStream::Qt_5_8);
}
