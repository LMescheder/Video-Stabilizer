TEMPLATE = app
CONFIG += c++11
#CONFIG -= qt

HEADERS += \
    stabilizer.hpp

SOURCES += \
    main.cpp \
    stabilizer.cpp


LIBS += -lopencv_core -lopencv_highgui
LIBS += -lopencv_imgproc
LIBS += -lopencv_video
LIBS += -lopencv_features2d
LIBS += -lopencv_videostab -lopencv_stitching -lopencv_calib3d

QMAKE_CXXFLAGS_RELEASE -= -O0
QMAKE_CXXFLAGS_RELEASE -= -O1
QMAKE_CXXFLAGS_RELEASE *= -O2
QMAKE_CXXFLAGS_RELEASE -= -O3

QMAKE_CXXFLAGS_DEBUG *= -D_DEBUG
