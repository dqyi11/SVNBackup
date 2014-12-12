#-------------------------------------------------
#
# Project created by QtCreator 2014-12-12T01:31:09
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = particle_filter_c_plusplus
CONFIG   += console
CONFIG   -= app_bundle

QMAKE_CXXFLAGS += -std=c++11

TEMPLATE = app

INCLUDEPATH += include


SOURCES += main.cpp

HEADERS += \
    include/ternary.h \
    include/statefun.h \
    include/setting.h \
    include/sampler.h \
    include/resampler.h \
    include/ran_generator.h \
    include/proposal.h \
    include/pfilter.h \
    include/obsvfun.h \
    include/compose3.h \
    include/binder3rd.h
