#-------------------------------------------------
#
# Project created by QtCreator 2014-12-22T18:08:32
#
#-------------------------------------------------

QT       -= core gui

TARGET = sec
TEMPLATE = lib

QMAKE_CXXFLAGS += -std=c++11 -gdwarf-3


isEmpty(INSTALL_PATH) {
    LIBS += -L$$system(echo ~/usr/lib)
    INCLUDEPATH += $$system(echo ~/usr/include)
} else {
    LIBS += -L$$INSTALL_PATH/lib
    INCLUDEPATH += $$INSTALL_PATH/include
}

LIBS += -lutilities


SOURCES += \
    src/commonsources.cpp \
    src/commoncontrollers.cpp \
    src/synchronization.cpp \
    src/sleeper.cpp \
    src/experimentlogger.cpp

HEADERS += \
    src/commonsources.h \
    src/source.h \
    src/controller.h \
    src/threadslink.h \
    src/basiccontroller.h \
    src/commoncontrollers.h \
    src/synchronization.h \
    src/sleeper.h \
    src/controllersequence.h \
    src/mainthread.h \
    src/experimentlogger.h \
    src/listener.h \
    src/controllerfork.h

unix {

    isEmpty(INSTALL_PATH) {
        INSTALL_PATH="~/usr"
    }

    target.path = $$INSTALL_PATH/lib
    INSTALLS += target

    headers_folder.path = $$INSTALL_PATH/include
    headers_folder.files = sec

    headers_files.path = $$INSTALL_PATH/include/sec
    headers_files.files = $$HEADERS

    INSTALLS += headers_folder headers_files
}
