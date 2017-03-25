#-------------------------------------------------
#
# Project created by QtCreator 2016-12-11T12:52:00
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = RoomScanner
TEMPLATE = app


SOURCES += main.cpp\
        application.cpp \
    mesh.cpp \
    registration.cpp \
    texturing.cpp \
    filters.cpp

HEADERS  += application.h \
    parameters.h \
    filters.h \
    types.h \
    pointrepr.h \
    mesh.h \
    registration.h \
    texturing.h

FORMS    += application.ui

DISTFILES +=

RESOURCES += \
    Resources/Resources.qrc
