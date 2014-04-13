#-------------------------------------------------
#
# Project created by QtCreator 2014-04-13T15:33:19
#
#-------------------------------------------------

QT       -= core gui

TARGET = avb
TEMPLATE = lib
CONFIG += staticlib
INCLUDEPATH += ../include

SOURCES += \
   $$files(../drivers/misc/labx/*.c) \
   $$files(../drivers/char/labx*.c) \
   $$files(../drivers/net/labx_avb/*.c) \
   $$files(../drivers/net/labx_eth_locallink/*.c) \
   $$files(../drivers/net/labx_ethertnet/*.c) \
   $$files(../drivers/net/labx_ptp/*.c) \

HEADERS += \
   $$files(../drivers/net/labx_avb/*.h) \
   $$files(../drivers/net/labx_eth_locallink/*.h) \
   $$files(../drivers/net/labx_ethernet/*.h) \
   $$files(../drivers/net/labx_ptp/*.h) \
   $$files(../include/linux/labx*.h) \
   $$files(../include/net/labx_avb/*.h) \
   $$files(../include/net/labx_ethernet/*.h) \
   $$files(../include/net/labx_ptp/*.h) \

