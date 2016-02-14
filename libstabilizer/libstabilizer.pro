TEMPLATE = lib
CONFIG += staticlib
CONFIG += c++11
TARGET = stabilizer

#CONFIG -= qt

HEADERS += \
    MserStabilizer.h \
    PointStabilizer.h \
    Stabilizer.h \
    PatchStabilizer.h \
    PixelStabilizer.h

SOURCES += \
    MserStabilizer.cpp \
    PointStabilizer.cpp \
    Stabilizer.cpp \
    PatchStabilizer.cpp \
    PixelStabilizer.cpp

INCLUDEPATH += $${_PRO_FILE_PWD_}/../

LIBS += -L$${OUT_PWD}/../libmsertools/ -lmsertools

LIBS += -lopencv_core -lopencv_highgui
LIBS += -lopencv_imgproc -lopencv_imgcodecs
LIBS += -lopencv_video -lopencv_videoio
LIBS += -lopencv_features2d
LIBS += -lopencv_videostab -lopencv_stitching -lopencv_calib3d

QMAKE_CXXFLAGS_RELEASE -= -O0
QMAKE_CXXFLAGS_RELEASE -= -O1
QMAKE_CXXFLAGS_RELEASE *= -O2
QMAKE_CXXFLAGS_RELEASE -= -O3

QMAKE_CXXFLAGS_DEBUG *= -D_DEBUG
