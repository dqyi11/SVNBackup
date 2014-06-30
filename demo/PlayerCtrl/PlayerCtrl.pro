#-------------------------------------------------
#
# Project created by QtCreator 2014-02-09T06:57:37
#
#-------------------------------------------------

QT       -= gui

TARGET = PlayerCtrl
TEMPLATE = lib
CONFIG += staticlib

SOURCES += playerctrl.cpp \
    playercommunicator.cpp

HEADERS += playerctrl.h \
    playercommunicator.h
unix {
    target.path = /usr/lib
    INSTALLS += target
}

INCLUDEPATH += 'pkg-config --cflags playerc++ '

unix: CONFIG += link_pkgconfig
unix: PKGCONFIG += playerc++
