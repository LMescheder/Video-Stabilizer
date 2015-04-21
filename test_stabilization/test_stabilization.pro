TEMPLATE = app
CONFIG += console
CONFIG += c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp

INCLUDEPATH += $${_PRO_FILE_PWD_}/../
LIBS += -L$${OUT_PWD}/../stabilizer/ -lstabilizer

LIBS += -lopencv_core -lopencv_highgui
LIBS += -lopencv_imgproc -lopencv_imgcodecs
LIBS += -lopencv_video -lopencv_videoio
LIBS += -lopencv_features2d
LIBS += -lopencv_videostab -lopencv_stitching -lopencv_calib3d
