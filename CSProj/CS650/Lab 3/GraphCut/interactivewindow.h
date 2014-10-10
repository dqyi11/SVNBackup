#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <list>
#include "interactivelabel.h"

class QAction;
class QMenu;
//class QScrollArea;
class QPoint;

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

private:
    QAction * mpOpenAction;
    QAction * mpClearAction;
    QAction * mpSegmentAction;

    QMenu * mpFileMenu;
    QMenu * mpEditMenu;
    InteractiveLabel *mpImageLabel;
    //QScrollArea *mpScrollArea;




};

#endif // MAINWINDOW_H
