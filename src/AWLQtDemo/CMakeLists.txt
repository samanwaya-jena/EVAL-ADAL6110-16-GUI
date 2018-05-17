CMAKE_MINIMUM_REQUIRED(VERSION 2.8.7)
PROJECT(LinuxCute)
# Ici on jase
SET(CMAKE_VERBOSE_MAKEFILE ON)
# Appeler automatique le moc quand nécessaire
SET(CMAKE_AUTOMOC ON)
# Les fichiers générés par le moc sont générés dans le dossier bin, dire à CMake de toujours 	
# inclure les entêtes de ce dossier
SET(CMAKE_INCLUDE_CURRENT_DIR ON)
# Le module QtWidgets trouve ses propres dépendances (QtGui and QtCore)
FIND_PACKAGE(Qt5Widgets)
FIND_PACKAGE(OpenCV REQUIRED)
find_package(Boost COMPONENTS system filesystem date_time REQUIRED)
#FIND_PACKAGE(Qwt)
file(GLOB QWT_LIBRARIES "/usr/lib/libqwt*")
# Qt5Widgets_INCLUDES inclut aussi les entêtes pour ses dépendances (QtCore et QtGui)
INCLUDE_DIRECTORIES(${Qt5Widgets_INCLUDES})
#  -DQT_WIDGETS_LIB est nécessaire quand on utilise QtWidgets dans Qt5.
file(GLOB QWT_INCLUDE_DIR "/usr/include/qwt")
include_directories(include ${catkin_INCLUDE_DIRS} ${QWT_INCLUDE_DIR})
include_directories(include "../AWLCoreClasses")
include_directories(include "../AWLQtScope")
ADD_DEFINITIONS(${Qt5Widgets_DEFINITIONS})
# La création de l'exécutable échoue dans Qt5 avec la configuration par défaut
SET(CMAKE_CXX_FLAGS "${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
ADD_EXECUTABLE( LinuxCute main.cpp awlqtdemo.cpp  AWLSettings.cpp  fov_2dscan.cpp  TableView.cpp  VideoCapture.cpp  VideoViewer.cpp awlqtdemo.ui  fov_2dscan.ui  TableView.ui )
# Qt5Widgets_LIBRARIES contient aussi QtGui et QtCore
TARGET_LINK_LIBRARIES(LinuxCute ${Qt5Widgets_LIBRARIES} ${OpenCV_LIBS} ${QWT} 
	${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})
#target_link_libraries(Linuxcute Qt5::Widgets)
