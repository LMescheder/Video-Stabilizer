#-------------------------------------------------
#
# Project created by QtCreator 2015-05-07T15:22:07
#
#-------------------------------------------------

QT       -= core gui

TARGET = mser_tools
TEMPLATE = lib
CONFIG += staticlib
CONFIG += c++11
#CONFIG -= qt


HEADERS += \
    ComponentTreeParser.hpp \
    MatAccessor.hpp \
    MatAnalyzer.hpp \
    MatMser.hpp \
    MatMserTracker.hpp \
    MatComponentStats.hpp

SOURCES += \
    MatMser.cpp \
    MatAccessor.cpp \
    MatMserTracker.cpp \
    MatAnalyzer.cpp \
    MatComponentStats.cpp

unix {
    target.path = /usr/lib
    INSTALLS += target
}




