#include "morrfvisualizer.h"

MORRFVisualizer::MORRFVisualizer(MORRF* morrf, QWidget *parent) :
    QLabel(parent)
{
    mpMORRF = morrf;
}
