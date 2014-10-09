#-------------------------------------------------
#
# Project created by QtCreator 2014-10-09T14:13:12
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = testGraphCut
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

INCLUDEPATH += J:\\opencv\build\include

LIBS += -LJ:\opencv\build\x86\vc10\lib \
    -lopencv_core246 \
    -lopencv_highgui246 \
    -lopencv_imgproc246 \
    -lopencv_features2d246 \
    -lopencv_calib3d246

SOURCES += main.cpp \
    maxflow/graph.cpp \
    maxflow/maxflow.cpp

HEADERS += \
    maxflow/block.h \
    maxflow/graph.h
