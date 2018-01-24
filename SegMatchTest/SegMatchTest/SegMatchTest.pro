TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    ../src/hdl_grabber_.cpp \
    ../src/vlp_grabber_.cpp \
    ../src/TrjFile.cpp \
    ../src/MapConstruct.cpp \
    ../src/TrjLogReader.cpp \
    ../src/SegmentRegionGrow.cpp \
    main_segment_match.cpp

QMAKE_CXXFLAGS += -std=c++11

DEFINES += HAVE_PCAP

INCLUDEPATH += /usr/include
INCLUDEPATH += /usr/include/opencv
INCLUDEPATH += /usr/local/include/pcl-1.8
INCLUDEPATH += /usr/include/eigen3
INCLUDEPATH += /usr/local/include/vtk-8.1
INCLUDEPATH += ../src


LIBS += /usr/lib/x86_64-linux-gnu/libpthread.so

LIBS += /usr/local/lib/libvtkRenderingCore-8.1.so
LIBS += /usr/local/lib/libvtkCommonDataModel-8.1.so
LIBS += /usr/local/lib/libvtkCommonMath-8.1.so
LIBS += /usr/local/lib/libvtkCommonCore-8.1.so

LIBS += /usr/lib/x86_64-linux-gnu/libboost_system.so
LIBS += /usr/lib/x86_64-linux-gnu/libboost_thread.so

LIBS += /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.9
LIBS += /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.9
LIBS += /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.9
LIBS += /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.9
LIBS += /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.9
LIBS += /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.9
LIBS += /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.9
LIBS += /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.9
LIBS += /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.9
LIBS += /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.9
LIBS += /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.9
LIBS += /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.9
LIBS += /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.9
LIBS += /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.9
LIBS += /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.9
LIBS += /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.9
LIBS += /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.9




LIBS += /usr/local/lib/libpcl_common.so
LIBS += /usr/local/lib/libpcl_features.so
LIBS += /usr/local/lib/libpcl_filters.so
LIBS += /usr/local/lib/libpcl_io.so
LIBS += /usr/local/lib/libpcl_io_ply.so
LIBS += /usr/local/lib/libpcl_kdtree.so
LIBS += /usr/local/lib/libpcl_keypoints.so
LIBS += /usr/local/lib/libpcl_ml.so
LIBS += /usr/local/lib/libpcl_octree.so
LIBS += /usr/local/lib/libpcl_outofcore.so
LIBS += /usr/local/lib/libpcl_people.so
LIBS += /usr/local/lib/libpcl_recognition.so
LIBS += /usr/local/lib/libpcl_registration.so
LIBS += /usr/local/lib/libpcl_sample_consensus.so
LIBS += /usr/local/lib/libpcl_search.so
LIBS += /usr/local/lib/libpcl_segmentation.so
LIBS += /usr/local/lib/libpcl_stereo.so
LIBS += /usr/local/lib/libpcl_surface.so
LIBS += /usr/local/lib/libpcl_tracking.so
LIBS += /usr/local/lib/libpcl_visualization.so

LIBS += /usr/lib/x86_64-linux-gnu/libpcap.so.0.8

HEADERS += \
    ../src/hdl_grabber_.h \
    ../src/HDL32Grab.h \
    ../src/vlp_grabber_.h \
    ../src/VLP16Graber.h \
    ../src/SegmentRegionGrow.h \
    ../src/TrjFile.h \
    ../src/MapConstruct.h \
    ../src/TrjLogReader.h \
    ../src/SegmentDefine.h
