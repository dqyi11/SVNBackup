#ifndef MORRFVISUALIZER_H
#define MORRFVISUALIZER_H

#include <QLabel>
#include "morrf.h"

class MORRFVisualizer : public QLabel
{
    Q_OBJECT
public:
    explicit MORRFVisualizer(QWidget *parent = 0);

    void setMORRF(MORRF* pMorrf);
    
signals:
    
public slots:

private:
    MORRF* mpMORRF;
    
};

#endif // MORRFVISUALIZER_H
