#-------------------------------------------------
#
# Project created by QtCreator 2014-02-06T14:45:36
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = RoboStageMonitor
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    mapviewer.cpp \
    discretepath.cpp \
    pathloader.cpp \
    motioncontroller.cpp

HEADERS  += mainwindow.h \
    mapviewer.h \
    discretepath.h \
    pathloader.h \
    motioncontroller.h

FORMS    += mainwindow.ui

INCLUDEPATH += ../PlayerCtrl

LIBS += ../build-PlayerCtrl-Desktop-Debug/libPlayerCtrl.a

unix: CONFIG += link_pkgconfig
unix: PKGCONFIG += playerc++
