QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    ../ImProConsole2/uigetfile.cpp \
    main.cpp \
    ConsoleG.cpp \
    qcustomplot.cpp \
    QcustomplotTestDialog.cpp \
    UserPointCalibrationDialog.cpp \
    ../ImProConsole2/FileSeq.cpp \
    ../ImProConsole2/ImagePointsPicker.cpp \
    ../ImProConsole2/ImageSequence.cpp \
    ../ImProConsole2/IntrinsicCalibrator.cpp \
    ../ImProConsole2/improCalib.cpp \
    ../ImProConsole2/improEdgeEnhancement.cpp \
    ../ImProConsole2/improFileIO.cpp \
    ../ImProConsole2/improMath.cpp \
    ../ImProConsole2/improStrings.cpp \
    ../ImProConsole2/improTargetTracking.cpp \
    ../ImProConsole2/impro_util.cpp \
    ../ImProConsole2/pickAPoint.cpp \
    ../ImProConsole2/smoothZoomAndShow.cpp \
    ../ImProConsole2/matchTemplateWithRot.cpp \
    ../ImProConsole2/matchTemplateWithRotPyr.cpp \
    ../ImProConsole2/enhancedCorrelationWithReference.cpp


HEADERS += \
    ../ImProConsole2/uigetfile.h \
    ConsoleG.h \
    qcustomplot.h \
    QcustomplotTestDialog.h \
    UserPointCalibrationDialog.h \
    ../ImProConsole2/FileSeq.h \
    ../ImProConsole2/ImagePointsPicker.h \
    ../ImProConsole2/ImageSequence.h \
    ../ImProConsole2/IntrinsicCalibrator.h \
    ../ImProConsole2/improCalib.h \
    ../ImProConsole2/improEdgeEnhancement.h \
    ../ImProConsole2/improFileIO.h \
    ../ImProConsole2/improMath.h \
    ../ImProConsole2/improStrings.h \
    ../ImProConsole2/improTargetTracking.h \
    ../ImProConsole2/impro_util.h \
    ../ImProConsole2/pickAPoint.h \
    ../ImProConsole2/smoothZoomAndShow.h \
    ../ImProConsole2/matchTemplateWithRot.h \
    ../ImProConsole2/matchTemplateWithRotPyr.h \
    ../ImProConsole2/enhancedCorrelationWithReference.h

INCLUDEPATH += ../ImProConsole2/

FORMS += \
    ConsoleG.ui \
    QcustomplotTestDialog.ui \
    UserPointCalibrationDialog.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

win32 {
    win32:CONFIG(release, debug|release): LIBS += -LC:/opencv/opencv451x/opencv-4.5.1/build/install/x64/vc16/lib/ -lopencv_world451
    else:win32:CONFIG(debug, debug|release): LIBS += -LC:/opencv/opencv451x/opencv-4.5.1/build/install/x64/vc16/lib/ -lopencv_world451d
    INCLUDEPATH += C:/opencv/opencv451x/opencv-4.5.1/build/install/include
    DEPENDPATH += C:/opencv/opencv451x/opencv-4.5.1/build/install/include
}
macx {
    macx: LIBS += -L /usr/local/Cellar/opencv/4.5.0_5/lib -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_imgcodecs
    INCLUDEPATH += /usr/local/Cellar/opencv/4.5.0_5/include/opencv4
    DEPENDPATH += /usr/local/Cellar/opencv/4.5.0_5/include/opencv4v
}

