#-------------------------------------------------
#
# Project created by QtCreator 2014-10-09T13:32:23
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = GraphCut
TEMPLATE = app

INCLUDEPATH += J:\\opencv\build\include

LIBS += -LJ:\opencv\build\x86\vc10\lib \
    -lopencv_core246 \
    -lopencv_highgui246 \
    -lopencv_imgproc246 \
    -lopencv_features2d246 \
    -lopencv_calib3d246

SOURCES += main.cpp\
    maxflow/graph.cpp \
    maxflow/maxflow.cpp \
    imagedatagraph.cpp \
    interactivewindow.cpp \
    interactivelabel.cpp \
    segmentation.cpp \
    kerneldensityestimator.cpp \
    configdialog.cpp \
    parametermanager.cpp \
    densityestimator.cpp \
    gmmdensityestimator.cpp \
    kmean.cpp \
    gmm.cpp

HEADERS  += \
    maxflow/block.h \
    maxflow/graph.h \
    imagedatagraph.h \
    interactivewindow.h \
    interactivelabel.h \
    segmentation.h \
    kerneldensityestimator.h \
    configdialog.h \
    parametermanager.h \
    densityestimator.h \
    gmmdensityestimator.h \
    globaldef.h \
    kmean.h \
    gmm.h

FORMS    +=
