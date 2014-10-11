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

    enum WidgetState {NORMAL, ADD_FOREGROUND, ADD_BACKGROUND};

    QPixmap mColorPixmap;
    QPixmap mGrayPixmap;



protected:
    void mouseReleaseEvent(QMouseEvent * e);
    void mousePressEvent(QMouseEvent * e);
    void mouseMoveEvent(QMouseEvent * e);

    void paintEvent(QPaintEvent* e);

    SeedManager * mpForegroundSeedMgr;
    SeedManager * mpBackgroundSeedMgr;
private:
    WidgetState mCurrentState;
    int mPointerRadius;


signals:

public slots:


};

#endif // INTERACTIVELABEL_H
