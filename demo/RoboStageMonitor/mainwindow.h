#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "mapviewer.h"
#include "playerctrl.h"
#include "discretepath.h"
#include "pathloader.h"
#include "motioncontroller.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    double transFromDispX(double x);
    double transFromDispY(double y);
    double transToDispX(double x);
    double transToDispY(double y);

private slots:
    void on_action_Open_triggered();

    void on_action_Play_triggered();

private:
    Ui::MainWindow *ui;
    MapViewer * mpMapViewer;
    PlayerCtrl * mCtrl;
    MotionController * mMotionCtrl;

    PathLoader * mpPathLoader;

    int timerId;

    int mMapWidth;
    int mMapHeight;

    bool mCtrlOn;
    int mInterval;
protected:
    void timerEvent(QTimerEvent *event);
};

#endif // MAINWINDOW_H
