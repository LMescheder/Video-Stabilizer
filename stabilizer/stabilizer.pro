CONFIG+=c++11
TEMPLATE=app

HEADERS += \
    ComponentTreeParser.hpp OpenCVMatAccessor.hpp

SOURCES += \
    main.cpp

LIBS += -lopencv_core -lopencv_highgui
LIBS += -lopencv_imgproc -lopencv_imgcodecs
LIBS += -lopencv_video -lopencv_videoio
LIBS += -lopencv_features2d

QMAKE_CXXFLAGS_RELEASE -= -O0
QMAKE_CXXFLAGS_RELEASE -= -O1
QMAKE_CXXFLAGS_RELEASE += -O2
QMAKE_CXXFLAGS_RELEASE -= -O3

QMAKE_CXXFLAGS_DEBUG *= -D_DEBUG
