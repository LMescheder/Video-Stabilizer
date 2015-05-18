TEMPLATE = app
#CONFIG += console
CONFIG += c++11
CONFIG -= app_bundle
#CONFIG -= qt

PRE_TARGETDEPS += $${OUT_PWD}/../stabilizer/libstabilizer.a

SOURCES += main.cpp \
    AccuracyEvaluator.cpp

INCLUDEPATH += $${_PRO_FILE_PWD_}/../
LIBS += -L$${OUT_PWD}/../stabilizer/ -lstabilizer
LIBS += -L$${OUT_PWD}/../mser_tools/ -lmser_tools

LIBS += -lopencv_core -lopencv_highgui
LIBS += -lopencv_imgproc
LIBS += -lopencv_video
LIBS += -lopencv_features2d
LIBS += -lopencv_videostab -lopencv_stitching -lopencv_calib3d

HEADERS += \
    AccuracyEvaluator.h
