#pragma once
#include <QMainWindow>
#include <memory>

namespace Ui
{
class CMainWindow;
}
class CGlWidget;
class CDocument;

class CMainWindow : public QMainWindow {
	Q_OBJECT

public:
	CMainWindow(QWidget *parent = 0);
	virtual ~CMainWindow();

private slots:
	void openMap();

private:
	CDocument *m_document;
	//QTabWidget *m_tabwidget;

	std::unique_ptr<Ui::CMainWindow> m_ui;
};
