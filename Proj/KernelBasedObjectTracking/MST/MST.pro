#-------------------------------------------------
#
# Project created by QtCreator 2014-11-23T17:34:24
#
#-------------------------------------------------

TARGET = MST

INCLUDEPATH += J:\\opencv\build\include

LIBS += -LJ:\opencv\build\x86\vc10\lib \
    -lopencv_core246 \
    -lopencv_highgui246 \
    -lopencv_imgproc246 \
    -lopencv_features2d246 \
    -lopencv_calib3d246


QT       += core gui widgets \
            multimedia \
            multimediawidgets


SOURCES += main.cpp
