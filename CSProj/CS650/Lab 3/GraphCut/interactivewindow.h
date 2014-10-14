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

class InteractiveWindow : public QMainWindow
{
    Q_OBJECT

public:



    explicit InteractiveWindow(QWidget *parent = 0);
    ~InteractiveWindow();


private slots:
    void on_open_clicked();
    void on_clear_clicked();
    void on_segment_clicked();
    void on_graphcut_clicked();
    void on_grabcut_clicked();

private:
    QAction * mpOpenAction;
    QAction * mpClearAction;
    QAction * mpSegmentAction;

    QActionGroup * mpWorkStateGroup;
    QAction * mpGraphCutAction;
    QAction * mpGrabCutAction;

    QMenu * mpFileMenu;
    QMenu * mpEditMenu;
    QMenu * mpModeMenu;
    InteractiveLabel *mpImageLabel;
    //QScrollArea *mpScrollArea;

    QString mFilename;


};

#endif // MAINWINDOW_H
