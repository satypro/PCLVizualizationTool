#-------------------------------------------------
#
# Project created by QtCreator 2017-08-20T15:31:07
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = VizualizationTool
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    passthroughfiltercpp.cpp

HEADERS  += mainwindow.h \
    passthroughfiltercpp.h

FORMS    += mainwindow.ui

DISTFILES +=

INCLUDEPATH += /usr/local/include /usr/local/include/pcl-1.7 /usr/include/eigen3 /usr/local/include/teem /usr/include/vtk-5.10

QMAKE_LIBDIR += /usr/local/lib /usr/lib /usr/local/lib/libteem /usr/local/include
LIBS += -lboost_system -lpcl_common -lpcl_io_ply -lpcl_io -lpcl_kdtree -lpcl_keypoints -lpcl_octree -lpcl_filters -lpcl_visualization
LIBS += -lpcl_search -lpcl_filters -lpcl_segmentation -lpcl_features -lpcl_search -lGL -lGLU
LIBS += -lteem
LIBS += -L"/usr/include" -lCGAL -lgmp
