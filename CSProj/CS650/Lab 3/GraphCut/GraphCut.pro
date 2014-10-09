#-------------------------------------------------
#
# Project created by QtCreator 2014-10-09T13:32:23
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = GraphCut
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    maxflow/graph.cpp \
    maxflow/maxflow.cpp \
    imagedatagraph.cpp

HEADERS  += mainwindow.h \
    maxflow/block.h \
    maxflow/graph.h \
    imagedatagraph.h

FORMS    += mainwindow.ui
