QT -= gui
QT += core serialport
! include( ../common.pri ) {
    error( "$${TARGET} Couldn't find the common.pri file!" )
}

TARGET = RobotEngine
TEMPLATE = lib
CONFIG += staticlib

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    motor.cpp \
    motorkernel.cpp \
    robotengine.cpp

HEADERS += \
    motor.h \
    motorkernel.h \
    robotengine.h \
    utils.h

# Default rules for deployment.
unix {
    target.path = $$[QT_INSTALL_PLUGINS]/generic
}
!isEmpty(target.path): INSTALLS += target



