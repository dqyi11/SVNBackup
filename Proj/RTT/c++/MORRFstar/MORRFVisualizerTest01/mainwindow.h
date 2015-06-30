#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMenu>
#include <QMenuBar>
#include <QAction>
#include <QPixmap>
#include "morrfvisualizer.h"

class ConfigObjDialog;

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

private:

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

    QPixmap * mpMap;

    QPoint mCursorPoint;

    ConfigObjDialog * mpConfigObjDialog;


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
