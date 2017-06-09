#pragma once
#include <QMainWindow>
#include <memory>

namespace Ui
{
class CMainWindow;
}
class CGlWidget;
class CDocument;
class CObservationTreeModel;
class QTreeWidgetItem;

class CMainWindow : public QMainWindow {
	Q_OBJECT

public:
	CMainWindow(QWidget *parent = 0);
	virtual ~CMainWindow();

private slots:
	void openMap();
	void itemClicked(const QModelIndex &index);

private:
	CDocument *m_document;
	CObservationTreeModel *m_model;

	std::unique_ptr<Ui::CMainWindow> m_ui;
};
