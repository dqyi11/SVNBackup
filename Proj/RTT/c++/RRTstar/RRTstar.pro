#-------------------------------------------------
#
# Project created by QtCreator 2014-12-13T21:03:44
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = RRTstar
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp \
    system_single_integrator.cpp \
    kdtree.c

HEADERS += \
    kdtree.h \
    rrts.h \
    rrts.hpp \
    system.h \
    system_single_integrator.h
