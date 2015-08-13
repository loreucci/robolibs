#-------------------------------------------------
#
# Project created by QtCreator 2015-01-20T12:03:07
#
#-------------------------------------------------

QT       -= core gui

TARGET = utilities
TEMPLATE = lib

QMAKE_CXXFLAGS += -std=c++14 -gdwarf-3

SOURCES += \
    src/utilities.cpp

HEADERS += \
    src/utilities.h \
    src/message.h

unix {

    isEmpty(INSTALL_PATH) {
        INSTALL_PATH="~/usr"
    }
    
    target.path = $$INSTALL_PATH/lib
    INSTALLS += target

    headers_folder.path = $$INSTALL_PATH/include
    headers_folder.files = utilities

    headers_files.path = $$INSTALL_PATH/include/utilities
    headers_files.files = $$HEADERS

    INSTALLS += headers_folder headers_files
}
