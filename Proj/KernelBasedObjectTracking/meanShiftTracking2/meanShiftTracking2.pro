#-------------------------------------------------
#
# Project created by QtCreator 2014-10-27T17:24:59
#
#-------------------------------------------------

TEMPLATE = app
TARGET = meanShiftTracking

LIBS += -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_calib3d -lopencv_imgproc


QT       += core

SOURCES += \
    targetdetector.cpp \
    main.cpp

HEADERS += \
    targetdetector.h
