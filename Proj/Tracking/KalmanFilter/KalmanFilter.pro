#-------------------------------------------------
#
# Project created by QtCreator 2014-12-17T11:37:48
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = KalmanFilter
CONFIG   += console
CONFIG   -= app_bundle

INCLUDEPATH += J:\\opencv\build\include

LIBS += -LJ:\opencv\build\x86\vc10\lib \
    -lopencv_core246 \
    -lopencv_highgui246 \
    -lopencv_imgproc246 \
    -lopencv_features2d246 \
    -lopencv_calib3d246 \
    -lopencv_video246

TEMPLATE = app


SOURCES += main.cpp
