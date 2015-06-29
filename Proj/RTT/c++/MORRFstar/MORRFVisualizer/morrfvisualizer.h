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

    MultiObjPathPlanningInfo mMOPPInfo;
signals:
    
public slots:

private:
    MORRF* mpMORRF;

private slots:
    void paintEvent(QPaintEvent * e);

    
};

#endif // MORRFVISUALIZER_H
