cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(correspondence_grouping C CXX)

find_package(PCL 1.5 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package( OpenCV 2.4.9 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
#target_link_libraries( ${OpenCV_LIBS} )

#find_package(Pcap REQUIRED)
#include_directories(${PCAP_INCLUDE_DIRS})
#link_directories(${PCAP_LIBRARIES})

include_directories(/usr/include/pcap)
include_directories(../src)
link_directories(/usr/lib/x86_64-linux-gnu)

ADD_DEFINITIONS(-std=c++11)
ADD_DEFINITIONS(-DHAVE_PCAP)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../build-SegMatchTest-Desktop-Release/)

add_library (vlp_grabber_ ../src/vlp_grabber_.cpp)
add_library (TrjLogReader ../src/TrjLogReader.cpp)
add_library (TrjFile ../src/TrjFile.cpp)
add_library (SegmentRegionGrow ../src/SegmentRegionGrow.cpp)
add_library (MapConstruct ../src/MapConstruct.cpp)
add_library (hdl_grabber_ ../src/hdl_grabber_.cpp)

link_libraries (vlp_grabber_)
link_libraries (TrjLogReader)
link_libraries (TrjFile)
link_libraries (SegmentRegionGrow)
link_libraries (MapConstruct)
link_libraries (hdl_grabber_)

add_executable (main_segment_match_cmake ../SegMatchTest/main_segment_match.cpp)
target_link_libraries (main_segment_match_cmake ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable (main_segment_extract_cmake ../SegMatchTest/main_segment_extract.cpp)
#target_link_libraries (main_segment_extract_cmake ${PCL_LIBRARIES} ${OpenCV_LIBS} ${PCAP_LIBRARIES})
target_link_libraries (main_segment_extract_cmake ${PCL_LIBRARIES} ${OpenCV_LIBS} libpcap.so)
