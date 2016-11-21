TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

unix:LIBS +=-L/usr/local/lib -lopencv_core \
            -L/usr/local/lib -lopencv_highgui \
            -L/usr/local/lib -lopencv_video \
            -L/usr/local/lib -lopencv_videoio \
            -L/usr/local/lib -lopencv_imgproc \
            -L/usr/local/lib -lopencv_features2d \
            -L/usr/local/lib -lopencv_objdetect \
            -L/usr/local/lib -lpng12 \
            -L/usr/local/lib -lpthread \
            -L/usr/local/lib -lfreenect \
            -I/usr/local/include/libfreenect \

SOURCES += main.cpp \
    myfreenectdevice.cpp \
    objfollowing.cpp

HEADERS += \
    myfreenectdevice.h \
    objfollowing.h
