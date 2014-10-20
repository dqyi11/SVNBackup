#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <list>
#include "interactivelabel.h"

class QAction;
class QMenu;
//class QScrollArea;
class QPoint;
class QActionGroup;
class ConfigDialog;
class ParameterManager;

class InteractiveWindow : public QMainWindow
{
    Q_OBJECT

public:



    explicit InteractiveWindow(QWidget *parent = 0);
    ~InteractiveWindow();


private slots:
    void on_open_clicked();
    void on_export_clicked();
    void on_clear_clicked();
    void on_segment_clicked();
    void on_graphcut_clicked();
    void on_grabcut_clicked();
    void on_config_clicked();

private:
    QAction * mpOpenAction;
    QAction * mpExportAction;
    QAction * mpClearAction;
    QAction * mpSegmentAction;
    QAction * mpConfigAction;

    QActionGroup * mpWorkStateGroup;
    QAction * mpGraphCutAction;
    QAction * mpGrabCutAction;

    QMenu * mpFileMenu;
    QMenu * mpEditMenu;
    QMenu * mpModeMenu;
    InteractiveLabel *mpImageLabel;
    //QScrollArea *mpScrollArea;

    QString mFilename;

    ParameterManager * mpParamMgr;
    ConfigDialog * mpConfigDialog;


};

#endif // MAINWINDOW_H
