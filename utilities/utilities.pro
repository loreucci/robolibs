#-------------------------------------------------
#
# Project created by QtCreator 2015-01-20T12:03:07
#
#-------------------------------------------------

QT       -= core gui

TARGET = utilities
TEMPLATE = lib

QMAKE_CXXFLAGS += -std=c++11 -gdwarf-3

SOURCES += \
    utilities.cpp

HEADERS += \
    utilities.h \
    message.h

unix {

    isEmpty(INSTALL_PATH) {
        warning("Installing to default path (~/usr).")
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
