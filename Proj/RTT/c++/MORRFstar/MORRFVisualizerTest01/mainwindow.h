#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMenu>
#include <QMenuBar>
#include <QAction>
#include <QPixmap>
#include <QProgressBar>
#include "morrfvisualizer.h"

class ConfigObjDialog;
class MORRF;

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

    MORRFVisualizer * mpViz;

protected:
    void createMenuBar();
    void createActions();
    bool openMap(QString filename);

    void keyPressEvent(QKeyEvent *event);
    void updateStatus();
private:

    void updateTitle();

    QMenu *mpFileMenu;
    QAction *mpOpenAction;
    QAction *mpSaveAction;

    QMenu *mpEditMenu;
    QAction *mpLoadMapAction;
    QAction *mpLoadObjAction;
    QAction *mpRunAction;

    QMenu *mpContextMenu;
    QAction *mpAddStartAction;
    QAction *mpAddGoalAction;

    QLabel * mpStatusLabel;
    QProgressBar * mpStatusProgressBar;

    QPixmap * mpMap;

    QPoint mCursorPoint;

    ConfigObjDialog * mpConfigObjDialog;
    MORRF           * mpMORRF;


private slots:
    void contextMenuRequested(QPoint point);
    void onOpen();
    void onSave();
    void onLoadMap();
    void onLoadObj();
    void onRun();
    void onAddStart();
    void onAddGoal();
};

#endif // MAINWINDOW_H
