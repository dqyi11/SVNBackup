#-------------------------------------------------
#
# Project created by QtCreator 2015-06-29T09:46:33
#
#-------------------------------------------------

QT       += core gui
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = MORRFViz
TEMPLATE = lib
CONFIG += staticlib

DESTDIR = ../lib

INCLUDEPATH += ../MORRF

LIBS += -L../lib -lMORRF

SOURCES += morrfvisualizer.cpp \
    multiobjpathplanninginfo.cpp

HEADERS  += morrfvisualizer.h \
    multiobjpathplanninginfo.h

