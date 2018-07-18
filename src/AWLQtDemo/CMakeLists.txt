CMAKE_MINIMUM_REQUIRED(VERSION 2.8.7)
PROJECT(LinuxCute)
# Let's talk
SET(CMAKE_VERBOSE_MAKEFILE ON)
# Automaticaly call moc when necessary
SET(CMAKE_AUTOMOC ON)
# moc genreated files are located in the bin directory,
# tell CMake to always include header files in this directory
SET(CMAKE_INCLUDE_CURRENT_DIR ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
# QtWidgets always finds its dependencies (QtGui and QtCore)
FIND_PACKAGE(Qt5Widgets)
find_package(Boost COMPONENTS thread system filesystem date_time signals REQUIRED)
find_package (Threads)
if(Boost_FOUND)
	message("Boost Found!")
endif()
#SET(BOOST_INCLUDE_DIRS "/usr/include/boost")
file(GLOB QWT_INCLUDE_DIR "/usr/include/qt5/qwt")
file(GLOB QWT_LIBRARIES "/usr/lib64/libqwt-qt5*")
# Qt5Widgets_INCLUDES also includes header files for its dependencies (QtCore et QtGui)
INCLUDE_DIRECTORIES(${Qt5Widgets_INCLUDES})
message("QT5 INC")
message("${Qt5Widgets_INCLUDES}")

#  -DQT_WIDGETS_LIB is necessary when one uses QtWidgets in Qt5
#file(GLOB QWT_INCLUDE_DIR "/usr/include/qwt")
include_directories(include ${QWT_INCLUDE_DIR} )
include_directories(include "../AWLCoreClasses")
include_directories(include "../AWLQtScope")
ADD_DEFINITIONS(${Qt5Widgets_DEFINITIONS})
message("QT5 DEF")
message(${Qt5Widgets_DEFINITIONS})
# Creation of executable fails in Qt5 with default configuration
SET(CMAKE_CXX_FLAGS "${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
message("QT5 CFLAGS")
message("${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
ADD_EXECUTABLE( LinuxCute main.cpp awlqtdemo.cpp  AWLSettings.cpp  fov_2dscan.cpp
      	TableView.cpp awlqtdemo.ui fov_2dscan.ui TableView.ui
../AWLCoreClasses/awlcoord.cpp          ../AWLCoreClasses/Publisher.cpp                ../AWLCoreClasses/ReceiverPostProcessor.cpp
../AWLCoreClasses/CoordinateSystem.cpp  ../AWLCoreClasses/ReceiverCANCapture.cpp       ../AWLCoreClasses/ReceiverSimulatorCapture.cpp
../AWLCoreClasses/DebugPrintf.cpp       ../AWLCoreClasses/ReceiverCapture.cpp          ../AWLCoreClasses/Subscription.cpp
../AWLCoreClasses/DetectionStruct.cpp   ../AWLCoreClasses/ThreadedWorker.cpp
../AWLCoreClasses/LoopedWorker.cpp      
../AWLQtScope/awlqtscope.cpp    ../AWLQtScope/knob.cpp        
../AWLQtScope/awlscopeplot.cpp  ../AWLQtScope/signaldata.cpp
../AWLQtScope/curvedata.cpp     ../AWLQtScope/wheelbox.cpp
../AWLQtScope/awlqtscope.ui 
)

# Comment next 2 lines to disable EasySync support
TARGET_COMPILE_DEFINITIONS( LinuxCute PUBLIC USE_CAN_EASYSYNC )
TARGET_SOURCES( LinuxCute PUBLIC ../AWLCoreClasses/ReceiverEasySyncCapture.cpp )

# Comment next 6 lines to disable Kvaser support
#file(GLOB CAN_INCLUDE_DIR "/usr/local/include/canlib")
#file(GLOB CAN_LIBRARIES "/usr/local/lib/libcan*")
#include_directories(include ${CAN_INCLUDE_DIR})
#TARGET_COMPILE_DEFINITIONS( LinuxCute PUBLIC USE_CAN_KVASER )
#TARGET_SOURCES( LinuxCute PUBLIC ../AWLCoreClasses/ReceiverKvaserCapture.cpp )
#TARGET_LINK_LIBRARIES(LinuxCute ${CAN_LIBRARIES} )

# Comment next 2 lines to disable SocketCAN support WARNING Linux Only
TARGET_COMPILE_DEFINITIONS( LinuxCute PUBLIC USE_CAN_SOCKETCAN )
TARGET_SOURCES( LinuxCute PUBLIC ../AWLCoreClasses/ReceiverSocketCANCapture.cpp )

# Comment next 2 lines to disable PosixUDP support WARNING Linux Only
TARGET_COMPILE_DEFINITIONS( LinuxCute PUBLIC USE_POSIXUDP )
TARGET_SOURCES( LinuxCute PUBLIC ../AWLCoreClasses/ReceiverPosixUDPCapture.cpp )

# Comment next 2 lines to disable PosixTTY support WARNING Linux Only
TARGET_COMPILE_DEFINITIONS( LinuxCute PUBLIC USE_POSIXTTY )
TARGET_SOURCES( LinuxCute PUBLIC ../AWLCoreClasses/ReceiverPosixTTYCapture.cpp )

# Comment next 2 lines to disable LibUSB support
TARGET_COMPILE_DEFINITIONS( LinuxCute PUBLIC USE_LIBUSB )
TARGET_LINK_LIBRARIES( LinuxCute /usr/lib64/libusb-1.0.so.0 )
TARGET_SOURCES( LinuxCute PUBLIC ../AWLCoreClasses/ReceiverLibUSBCapture.cpp )

# Comment next 4 lines to disable OpenCV Video (and OpenCV in general)
TARGET_COMPILE_DEFINITIONS( LinuxCute PUBLIC USE_OPENCV_VIDEO )
FIND_PACKAGE(OpenCV REQUIRED)
TARGET_SOURCES( LinuxCute PUBLIC VideoCapture.cpp  VideoViewer.cpp )
TARGET_LINK_LIBRARIES(LinuxCute ${OpenCV_LIBS} )

# Qt5Widgets_LIBRARIES contient aussi QtGui et QtCore
message ("QT5_LIB")
message (${Qt5Widgets_LIBRARIES})
message ("QT5:LIB")
message (Qt5::Widgets)
message ("QT5 END")
TARGET_LINK_LIBRARIES(LinuxCute ${Qt5Widgets_LIBRARIES} ${QWT_LIBRARIES}
${Boost_DATE_TIME_LIBRARY} ${Boost_FILESYSTEM_LIBRARY}
${Boost_SIGNALS_LIBRARY} ${Boost_THREAD_LIBRARY} )
#target_link_libraries(LinuxCute Qt5::Widgets)
target_link_libraries(LinuxCute Threads::Threads)
