#ifndef INTERACTIVELABEL_H
#define INTERACTIVELABEL_H

#include <QLabel>

class SeedManager;

class InteractiveLabel : public QLabel
{
    friend class InteractiveWindow;
    Q_OBJECT
public:
    explicit InteractiveLabel (QWidget * parent = 0);
    ~InteractiveLabel();

    enum LabelingState {NORMAL, ADD_FOREGROUND, ADD_BACKGROUND};
    enum WorkingState {GRAPH_CUT_SEGMENTATION, GRAB_CUT_SEGMENTATION};

    QPixmap mColorPixmap;
    QPixmap mGrayPixmap;

    void setWorkingState(WorkingState state);

protected:
    void mouseReleaseEvent(QMouseEvent * e);
    void mousePressEvent(QMouseEvent * e);
    void mouseMoveEvent(QMouseEvent * e);

    void paintEvent(QPaintEvent* e);

    SeedManager * mpForegroundSeedMgr;
    SeedManager * mpBackgroundSeedMgr;
    WorkingState mCurrentWorkingState;
private:
    LabelingState mCurrentLabelingState;
    int mPointerRadius;

    QPoint * mpInitPoint;
    QPoint * mpEndPoint;

signals:

public slots:


};

#endif // INTERACTIVELABEL_H
