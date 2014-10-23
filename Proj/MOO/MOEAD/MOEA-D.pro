#-------------------------------------------------
#
# Project created by QtCreator 2014-10-23T11:15:47
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = MOEA-D
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp

HEADERS += \
    common/benchmark.h \
    common/common.h \
    common/global.h \
    common/mylib.h \
    common/objective.h \
    common/random.h \
    common/recombination.h \
    DMOEA/dmoeaclass.h \
    DMOEA/dmoeafunc.h \
    NSGA2/nsga2class.h \
    NSGA2/nsga2func.h
