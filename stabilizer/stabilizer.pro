CONFIG+=c++11
TEMPLATE=app

HEADERS += \
    ComponentTreeParser.hpp OpenCVMatAccessor.hpp

SOURCES += \
    main.cpp OpenCVMatAccessor.cpp

LIBS += -lopencv_core -lopencv_highgui
LIBS += -lopencv_imgproc -lopencv_imgcodecs
LIBS += -lopencv_video -lopencv_videoio
