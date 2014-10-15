#-------------------------------------------------
#
# Project created by QtCreator 2014-10-09T13:32:23
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = GraphCut
TEMPLATE = app

INCLUDEPATH += C:\\opencv\build\include

LIBS += -LC:\opencv\build\x86\vc10\lib \
    -lopencv_core249 \
    -lopencv_highgui249 \
    -lopencv_imgproc249 \
    -lopencv_features2d249 \
    -lopencv_calib3d249

SOURCES += main.cpp\
    maxflow/graph.cpp \
    maxflow/maxflow.cpp \
    imagedatagraph.cpp \
    interactivewindow.cpp \
    interactivelabel.cpp \
    segmentation.cpp \
    kerneldensityestimator.cpp

HEADERS  += \
    maxflow/block.h \
    maxflow/graph.h \
    imagedatagraph.h \
    interactivewindow.h \
    interactivelabel.h \
    segmentation.h \
    kerneldensityestimator.h

FORMS    +=
