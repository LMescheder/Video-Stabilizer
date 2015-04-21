TEMPLATE = lib
CONFIG += staticlib
CONFIG += c++11
CONFIG -= qt

HEADERS += \
    ComponentTreeParser.hpp \
    MatAccessor.hpp \
    MatAnalyzer.hpp \
    MatMser.hpp \
    MatMserTracker.hpp \
    VideoStabilizer.hpp \
    MatComponentStats.hpp

SOURCES += \
    MatMser.cpp \
    MatAccessor.cpp \
    MatMserTracker.cpp \
    MatAnalyzer.cpp \
    VideoStabilizer.cpp \
    MatComponentStats.cpp

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
