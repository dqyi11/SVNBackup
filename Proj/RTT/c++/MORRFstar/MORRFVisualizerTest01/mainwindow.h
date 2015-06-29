#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtGui/QMainWindow>
#include <QMenu>
#include <QMenuBar>
#include <QAction>
#include <QPixmap>
#include "morrfvisualizer.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

protected:
    void createMenuBar();
    void createActions();

private:
    MORRFVisualizer * mpViz;
    QMenu *mpFileMenu;
    QMenu *mpEditMenu;

    QAction *mpOpenAction;
    QAction *mpSaveAction;
    QAction *mpLoadMapAction;
    QAction *mpLoadObjAction;
    QAction *mpRunAction;

    QPixmap * mpMap;
    QString mMapFilename;

    int mObjectiveNum;

private slots:
    void onOpen();
    void onSave();
    void onLoadMap();
    void onLoadObj();
    void onRun();
};

#endif // MAINWINDOW_H
