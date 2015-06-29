#-------------------------------------------------
#
# Project created by QtCreator 2015-06-27T17:21:48
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = MORRF
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    morrf.cpp \
    subtree.cpp

HEADERS  += mainwindow.h \
    kdtree++/region.hpp \
    kdtree++/node.hpp \
    kdtree++/kdtree.hpp \
    kdtree++/iterator.hpp \
    kdtree++/function.hpp \
    kdtree++/allocator.hpp \
    morrf.h \
    KDTree2D.h \
    subtree.h

FORMS    += mainwindow.ui
