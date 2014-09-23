#-------------------------------------------------
#
# Project created by QtCreator 2014-09-23T13:47:36
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = mean_shift
CONFIG   += console
CONFIG   -= app_bundle

INCLUDEPATH += /usr/local/include  \
               /usr/local/include/opencv  \
               /usr/local/include/opencv2

LIBS += /usr/local/lib/libopencv_video.so  \
       # /usr/local/lib/libopencv_ts.so \
        /usr/local/lib/libopencv_objdetect.so \
        /usr/local/lib/libopencv_ml.so \
        /usr/local/lib/libopencv_legacy.so \
        /usr/local/lib/libopencv_core.so \
        /usr/local/lib/libopencv_features2d.so  \
        /usr/local/lib/libopencv_imgproc.so \
        /usr/local/lib/libopencv_highgui.so \
        /usr/local/lib/libopencv_gpu.so \
        /usr/local/lib/libopencv_flann.so   \
        /usr/local/lib/libopencv_contrib.so \
        /usr/local/lib/libopencv_calib3d.so


TEMPLATE = app


SOURCES += main.cpp \
    meanshift.cpp

HEADERS += \
    meanshift.h
