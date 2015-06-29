#-------------------------------------------------
#
# Project created by QtCreator 2015-06-29T10:43:31
#
#-------------------------------------------------

QT       += core gui

TARGET = MORRFVisualizerTest01
TEMPLATE = app

DESTDIR = ../bin

INCLUDEPATH += ../MORRF \
               ../MORRFVisualizer

LIBS += -L../lib -lMORRF -lMORRFViz

SOURCES += main.cpp\
        mainwindow.cpp

HEADERS  += mainwindow.h
