TEMPLATE = lib
CONFIG += staticlib
CONFIG += c++11
#CONFIG -= qt

HEADERS += \
    MserStabilizer.h \
    PointStabilizer.h \
    Stabilizer.h

SOURCES += \
    MserStabilizer.cpp \
    PointStabilizer.cpp \
    Stabilizer.cpp

INCLUDEPATH += $${_PRO_FILE_PWD_}/../

LIBS += -L$${OUT_PWD}/../mser_tools/ -lmser_tools

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
