#ifndef MORRFVISUALIZER_H
#define MORRFVISUALIZER_H

#include <QLabel>
#include "morrf.h"
#include "multiobjpathplanninginfo.h"

class MORRFVisualizer : public QLabel
{
    Q_OBJECT
public:
    explicit MORRFVisualizer(QWidget *parent = 0);

    void setMORRF(MORRF* pMorrf);

    void prevTree();
    void nextTree();
    int getCurrentTreeIndex() { return mCurrentTreeIdx; }

    MultiObjPathPlanningInfo mMOPPInfo;
signals:
    
public slots:

private:
    MORRF* mpMORRF;
    int mCurrentTreeIdx;

private slots:
    void paintEvent(QPaintEvent * e);


    
};

#endif // MORRFVISUALIZER_H
