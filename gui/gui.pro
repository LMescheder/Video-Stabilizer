CONFIG += c++11
TEMPLATE = app

SOURCES = main.cpp

LIBS += -lopencv_core -lopencv_highgui
LIBS += -lopencv_imgproc -lopencv_imgcodecs
LIBS += -lopencv_video -lopencv_videoio
