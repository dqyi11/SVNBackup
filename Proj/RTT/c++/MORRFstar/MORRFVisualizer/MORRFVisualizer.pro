#-------------------------------------------------
#
# Project created by QtCreator 2015-06-29T09:46:33
#
#-------------------------------------------------

QT       += core gui

TARGET = MORRFViz
TEMPLATE = lib
CONFIG += staticlib

DESTDIR = ../lib

INCLUDEPATH += ../MORRF

LIBS += -L../lib -lMORRF

SOURCES += morrfvisualizer.cpp

HEADERS  += morrfvisualizer.h

