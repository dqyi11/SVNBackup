#-------------------------------------------------
#
# Project created by QtCreator 2014-11-26T14:23:24
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = Mean_Shift_Tracking2
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

LIBS += -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_calib3d -lopencv_imgproc

SOURCES += main.cpp \
    meanshift.cpp

HEADERS += \
    meanshift.h
