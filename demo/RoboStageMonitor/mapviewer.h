#ifndef MAPVIEWER_H
#define MAPVIEWER_H

#include <QLabel>
#include <QPoint>
#include "discretepath.h"

class MapViewer : public QLabel
{
    Q_OBJECT
public:
    explicit MapViewer(QWidget *parent = 0);

    void setPoint(int x, int y);
    void setPointYaw(double yaw);
    void showPoint(bool show);
    void loadPath(DiscretePath path);

private slots:
    void paintEvent(QPaintEvent * e);
private:
    bool mShowPoint;
    double mPointYaw;
    QPoint * mPoint;
    DiscretePath * mpPath;

signals:

public slots:


};

#endif // MAPVIEWER_H
