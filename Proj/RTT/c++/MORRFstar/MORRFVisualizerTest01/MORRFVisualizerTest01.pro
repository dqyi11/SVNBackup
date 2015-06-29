#-------------------------------------------------
#
# Project created by QtCreator 2015-06-29T10:43:31
#
#-------------------------------------------------

QT       += core gui
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = MORRFVisualizerTest01
TEMPLATE = app

DESTDIR = ../bin

INCLUDEPATH += ../MORRF \
               ../MORRFVisualizer

LIBS += -L../lib -lMORRF -lMORRFViz

SOURCES += main.cpp\
        mainwindow.cpp \
    configobjdialog.cpp

HEADERS  += mainwindow.h \
    configobjdialog.h
