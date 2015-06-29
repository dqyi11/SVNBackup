#-------------------------------------------------
#
# Project created by QtCreator 2015-06-27T17:21:48
#
#-------------------------------------------------

QT       += core
QT       -= gui

TARGET = MORRF
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app


SOURCES += main.cpp\
    morrf.cpp \
    subtree.cpp

HEADERS  += \
    kdtree++/region.hpp \
    kdtree++/node.hpp \
    kdtree++/kdtree.hpp \
    kdtree++/iterator.hpp \
    kdtree++/function.hpp \
    kdtree++/allocator.hpp \
    subtree.h \
    morrf.h \
    KDTree2D.h

