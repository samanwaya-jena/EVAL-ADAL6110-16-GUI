CMAKE_MINIMUM_REQUIRED(VERSION 2.8.7)
PROJECT(LinuxCute)
# Ici on jase
SET(CMAKE_VERBOSE_MAKEFILE ON)
# Appeler automatique le moc quand nécessaire
SET(CMAKE_AUTOMOC ON)
# Les fichiers générés par le moc sont générés dans le dossier bin, dire à CMake de toujours 	
# inclure les entêtes de ce dossier
SET(CMAKE_INCLUDE_CURRENT_DIR ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
# Le module QtWidgets trouve ses propres dépendances (QtGui and QtCore)
FIND_PACKAGE(Qt5Widgets)
find_package(Boost COMPONENTS thread system filesystem date_time signals REQUIRED)
find_package (Threads)
if(Boost_FOUND)
	message("Boost Found!")
endif()
#SET(BOOST_INCLUDE_DIRS "/usr/include/boost")
file(GLOB QWT_INCLUDE_DIR "/usr/include/qt5/qwt")
file(GLOB QWT_LIBRARIES "/usr/lib64/libqwt-qt5*")
file(GLOB CAN_INCLUDE_DIR "/usr/local/include/canlib")
file(GLOB CAN_LIBRARIES "/usr/local/lib/libcan*")
# Qt5Widgets_INCLUDES inclut aussi les entêtes pour ses dépendances (QtCore et QtGui)
INCLUDE_DIRECTORIES(${Qt5Widgets_INCLUDES})
#  -DQT_WIDGETS_LIB est nécessaire quand on utilise QtWidgets dans Qt5.
#file(GLOB QWT_INCLUDE_DIR "/usr/include/qwt")
#include_directories(include ${catkin_INCLUDE_DIRS} ${QWT_INCLUDE_DIR} ${CAN_INCLUDE_DIR})
include_directories(include ${QWT_INCLUDE_DIR} ${CAN_INCLUDE_DIR})
include_directories(include "../AWLCoreClasses")
include_directories(include "../AWLQtScope")
ADD_DEFINITIONS(${Qt5Widgets_DEFINITIONS})
# La création de l'exécutable échoue dans Qt5 avec la configuration par défaut
SET(CMAKE_CXX_FLAGS "${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
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

# Comment next 3 lines to disable Kvaser support
TARGET_COMPILE_DEFINITIONS( LinuxCute PUBLIC USE_CAN_KVASER )
TARGET_SOURCES( LinuxCute PUBLIC ../AWLCoreClasses/ReceiverKvaserCapture.cpp )
TARGET_LINK_LIBRARIES(LinuxCute ${CAN_LIBRARIES} )

# Comment next 2 lines to disable SocketCAN support
TARGET_COMPILE_DEFINITIONS( LinuxCute PUBLIC USE_CAN_SOCKETCAN )
TARGET_SOURCES( LinuxCute PUBLIC ../AWLCoreClasses/ReceiverSocketCANCapture.cpp )

# Comment next 4 lines to disable OpenCV Video (and OpenCV in general)
TARGET_COMPILE_DEFINITIONS( LinuxCute PUBLIC USE_OPENCV_VIDEO )
FIND_PACKAGE(OpenCV REQUIRED)
TARGET_SOURCES( LinuxCute PUBLIC VideoCapture.cpp  VideoViewer.cpp )
TARGET_LINK_LIBRARIES(LinuxCute ${OpenCV_LIBS} )

# Qt5Widgets_LIBRARIES contient aussi QtGui et QtCore
TARGET_LINK_LIBRARIES(LinuxCute ${Qt5Widgets_LIBRARIES} ${QWT_LIBRARIES}
${Boost_DATE_TIME_LIBRARY} ${Boost_FILESYSTEM_LIBRARY}
${Boost_SIGNALS_LIBRARY} ${Boost_THREAD_LIBRARY} )
#target_link_libraries(LinuxCute Qt5::Widgets)
target_link_libraries(LinuxCute Threads::Threads)
