#ifndef MORRFVISUALIZER_H
#define MORRFVISUALIZER_H

#include <QLabel>
#include "morrf.h"

class MORRFVisualizer : public QLabel
{
    Q_OBJECT
public:
    explicit MORRFVisualizer(MORRF* morrf, QWidget *parent = 0);
    
signals:
    
public slots:

private:
    MORRF* mpMORRF;
    
};

#endif // MORRFVISUALIZER_H
