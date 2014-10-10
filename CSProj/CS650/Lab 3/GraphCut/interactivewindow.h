#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <list>

class QAction;
class QLabel;
class QMenu;
//class QScrollArea;
class QPoint;

class InteractiveWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit InteractiveWindow(QWidget *parent = 0);
    ~InteractiveWindow();

    enum WindowState {NORMAL, ADD_FOREGROUND, ADD_BACKGROUND};

protected:
    void mouseReleaseEvent ( QMouseEvent * e );
    void mousePressEvent ( QMouseEvent * e );
    void mouseMoveEvent( QMouseEvent * e );

    void paintEvent(QPaintEvent* e);

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
    QLabel *mpImageLabel;
    //QScrollArea *mpScrollArea;

    WindowState mCurrentState;

    std::list<QPoint> mForegroundSeeds;
    std::list<QPoint> mBackgroundSeeds;


};

#endif // MAINWINDOW_H
