TARGET = KDTreeTest
#CONFIG   += console
#CONFIG   -= app_bundle

TEMPLATE = app

INCLUDEPATH += ../MORRF

DESTDIR = ../bin

LIBS += -L../lib -lMORRF

SOURCES += main.cpp

