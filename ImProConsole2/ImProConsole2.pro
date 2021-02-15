TEMPLATE = app
CONFIG += console c++17
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        CamMoveCorrector.cpp \
        FileSeq.cpp \
        FuncCalibInLabOnSite.cpp \
        FuncCalibOnSiteUserPoints.cpp \
        FuncCalibOnlyExtrinsic.cpp \
        FuncCalibStereoOnSite.cpp \
        FuncCamMoveCorrelation.cpp \
        FuncConstraintOnPolySurface.cpp \
        FuncDrawGrid.cpp \
        FuncDrawHouse.cpp \
        FuncDrawRotatingTemplate.cpp \
        FuncImshowOnline.cpp \
        FuncMandelbrot.cpp \
        FuncOptflowSeq.cpp \
        FuncPip.cpp \
        FuncQ4TemplatesPicking.cpp \
        FuncSingleCamWallMonitoring.cpp \
        FuncStoryDisp.cpp \
        FuncStoryDispV2.cpp \
        FuncStoryDispV3.cpp \
        FuncStoryDispV4.cpp \
        FuncStoryDispV6.cpp \
        FuncStoryDispV7.cpp \
        FuncStoryDispV8.cpp \
        FuncSyncTwoCams.cpp \
        FuncTemplatesPicking.cpp \
        FuncTestPlotCamRotNewDisp.cpp \
        FuncTrackingPointsEcc.cpp \
        FuncTrackingPyrTmpltMatch.cpp \
        FuncTriangulationAllSteps.cpp \
        FuncTryCamFocusExposure.cpp \
        FuncUndistortOnline.cpp \
        FuncVidImPointsQ4.cpp \
        FuncVidOptflowToVelocity.cpp \
        FuncVideo2Pics.cpp \
        FuncVideoDenceTracking.cpp \
        FuncWallDisp.cpp \
        FuncWallDispCam.cpp \
        FuncWallSingleCam.cpp \
        ImagePointsPicker.cpp \
        ImageSequence.cpp \
        IntrinsicCalibrator.cpp \
        IoData.cpp \
        Points2fHistoryData.cpp \
        Points3dHistoryData.cpp \
        RollingPlot.cpp \
        Submenu.cpp \
        enhancedCorrelationWithReference.cpp \
        estimateStoryDisp.cpp \
        estimateStoryDispV4.cpp \
        generalConstrained3dPosition_newton.cpp \
        improCalib.cpp \
        improDraw.cpp \
        improEdgeEnhancement.cpp \
        improFileIO.cpp \
        improMath.cpp \
        improStrings.cpp \
        improTargetTracking.cpp \
        impro_util.cpp \
        main.cpp \
        matchTemplateWithRot.cpp \
        matchTemplateWithRotPyr.cpp \
        pickAPoint.cpp \
        smoothZoomAndShow.cpp \
        sync.cpp \
        trackings.cpp \
        triangulatePoints2.cpp \
        uigetfile.cpp

DISTFILES += \
    ImProConsole2.pro.user

HEADERS += \
    CamMoveCorrector.h \
    FileSeq.h \
    ImagePointsPicker.h \
    ImageSequence.h \
    IntrinsicCalibrator.h \
    IoData.h \
    Points2fHistoryData.h \
    Points3dHistoryData.h \
    RollingPlot.h \
    Submenu.h \
    enhancedCorrelationWithReference.h \
    improCalib.h \
    improDraw.h \
    improEdgeEnhancement.h \
    improFileIO.h \
    improMath.h \
    improStrings.h \
    improTargetTracking.h \
    impro_util.h \
    matchTemplateWithRot.h \
    matchTemplateWithRotPyr.h \
    pickAPoint.h \
    smoothZoomAndShow.h \
    sync.h \
    trackings.h \
    triangulatepoints2.h \
    uigetfile.h

# OpenCV
win32:CONFIG(release, debug|release): LIBS += -LC:/opencv/opencv451x/opencv-4.5.1/build/install/x64/vc16/lib/ -lopencv_world451
else:win32:CONFIG(debug, debug|release): LIBS += -LC:/opencv/opencv451x/opencv-4.5.1/build/install/x64/vc16/lib/ -lopencv_world451d
else:unix: LIBS += -LC:/opencv/opencv451x/opencv-4.5.1/build/install/x64/vc16/lib/ -lopencv_world451
INCLUDEPATH += C:/opencv/opencv451x/opencv-4.5.1/build/install/include
DEPENDPATH += C:/opencv/opencv451x/opencv-4.5.1/build/install/include
