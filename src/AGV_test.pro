QT       += core gui
QT += core gui serialport
QT += network  # 添加网络模块
CONFIG += c++17
unix:!macx: LIBS += -lstdc++fs

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

TARGET = AGV_test
TEMPLATE = app

SOURCES += \
    LED.cpp \
    agv.cpp \
    aubo_robot.cpp \
    main.cpp \
    mainwindow.cpp

HEADERS += \
    LED.h \
    agv.h \
    aubo_robot.h \
    mainwindow.h \
    util.h \
    zcan.h

FORMS += \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

#AGV小车相关库（需自行获取 USB-CAN SDK）
#unix:!macx: LIBS += -L$$PWD/lib/ -lusb-1.0
#INCLUDEPATH += $$PWD/include
#DEPENDPATH += $$PWD/include

unix:!macx: LIBS += -L$$PWD/lib/ -lusbcanfd
INCLUDEPATH += $$PWD/include
DEPENDPATH += $$PWD/include

#aubo_robot相关库（需自行获取 AUBO Robot SDK）
# 将 AUBO SDK 解压到 dependents/robotSDK/ 目录
# 将 log4cplus 库放到 dependents/log4cplus/ 目录
unix{
    #64bit OS
    contains(QT_ARCH, x86_64){
    INCLUDEPATH += $$PWD/dependents/robotSDK/inc
    #LIBS += $$PWD/dependents/protobuf/linux-x64/lib/libprotobuf.a
    #LIBS += $$PWD/dependents/robotController/lib-linux64/libour_alg_i5p.a
    LIBS += -L$$PWD/dependents/log4cplus/linux_x64/lib -llog4cplus
    #LIBS += -L$$PWD/dependents/libconfig/linux_x64/lib/ -lconfig
    LIBS += -L$$PWD/dependents/robotSDK/lib/linux_x64/ -lauborobotcontroller
    LIBS += -lpthread
    }
}

#CONFIG += link_pkgconfig
PKGCONFIG += roscpp geometry_msgs
PKGCONFIG += tf2
PKGCONFIG += tf2_ros
#PKGCONFIG += tf2_geometry_msgs #error
CONFIG += link_pkgconfig
# 添加actionlib支持
PKGCONFIG += actionlib
PKGCONFIG += actionlib_msgs
PKGCONFIG += move_base_msgs

# OpenCV库
unix:!macx: LIBS += -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_videoio -lopencv_imgcodecs
unix:!macx: INCLUDEPATH += /usr/include/opencv4
# ZED SDK库
unix:!macx: LIBS += -L/usr/local/zed/lib -lsl_zed 
unix:!macx: INCLUDEPATH += /usr/local/zed/include
# CUDA库
unix:!macx: LIBS += -L/usr/local/cuda/lib64 -lcudart -lcuda
unix:!macx: INCLUDEPATH += /usr/local/cuda/include

