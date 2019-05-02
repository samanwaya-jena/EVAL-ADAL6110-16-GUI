cmake_minimum_required(VERSION 3.0.2) # Mendatory because of Qt5 --> https://doc.qt.io/qt-5/cmake-manual.html#using-qt-5-with-cmake-older-than-2-8-9
project(CuteDemo C CXX)
set(CMAKE_COLOR_MAKEFILE ON)

#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#%%    Helper functions       %%%%%%%%%%%%%%%     START       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#%%     > to simplify reading and writing           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

include(CheckCXXCompilerFlag)
function(enable_cxx_compiler_flag_if_supported flag)
    string(FIND "${CMAKE_CXX_FLAGS}" "${flag}" flag_already_set)
    if(flag_already_set EQUAL -1)
        message(STATUS "Flag:      ${flag}")
        check_cxx_compiler_flag("${flag}" flag_supported)
        if(flag_supported)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${flag}" PARENT_SCOPE)
        endif()
        unset(flag_supported CACHE)
    endif()
endfunction()

# Return the date (yyyy-mm-dd)
macro(get_current_date RESULT)
    if(WIN32)
        execute_process(COMMAND "cmd" " /C date /T" OUTPUT_VARIABLE ${RESULT})
        string(REGEX REPLACE "(..)/(..)/(....).*" "\\3-\\2-\\1" ${RESULT} ${${RESULT}})
    elseif(UNIX)
        execute_process(COMMAND "date" "+%Y-%m-%d" OUTPUT_VARIABLE ${RESULT})
    else()
        message(SEND_ERROR "Unable to detect date")
        set(${RESULT} UNKNOWN)
    endif()
endmacro()

# Return the time (hh-mm-ss)
macro(get_current_time RESULT)
    if(WIN32)
        execute_process(COMMAND "cmd" " /C echo %TIME%" OUTPUT_VARIABLE ${RESULT})
        string(REGEX REPLACE ".?(..):(..):(..).*" "\\1-\\2-\\3" ${RESULT} ${${RESULT}})
    elseif(UNIX)
        execute_process(COMMAND "date" "+%H-%M-%S" OUTPUT_VARIABLE ${RESULT})
    else()
        message(SEND_ERROR "Unable to detect time")
        set(${RESULT} UNKNOWN)
    endif()
endmacro()

macro(print_package_info package_name)
    set(NAME ${package_name})
    message(STATUS)
    message(STATUS "---------------------------------------------------------------")
    message(STATUS "---------------------------------------------------------------")
    message(STATUS "Printing info for... ${NAME}")
    message(STATUS "---------------------------------------------------------------")
    if(${NAME}_FOUND)
        message(STATUS "${NAME} Found!   (version          : ${${NAME}_VERSION})")
        message(STATUS "${NAME} directory                  : ${${NAME}_DIR}")
        message(STATUS "${NAME} definitions                : ${${NAME}_DEFINITIONS}")
        message(STATUS "${NAME} execution compile flags    : ${${NAME}_EXECUTABLE_COMPILE_FLAGS}")
        message(STATUS "${NAME} include dirs               : ${${NAME}_INCLUDE_DIRS}")
        message(STATUS "${NAME} includes                   : ${${NAME}_INCLUDES}")
        message(STATUS "${NAME} library dirs               : ${${NAME}_LIBRARY_DIRS}")
        message(STATUS "${NAME} libraries                  : ${${NAME}_LIBRARIES}")
    else()
        message(STATUS "${NAME} not Found...")
    endif()
    message(STATUS "---------------------------------------------------------------")
    message(STATUS "---------------------------------------------------------------")
    message(STATUS)
    unset(NAME)
endmacro()

macro(print_general_cmake_info)
    message(STATUS "---------------------------------------------------------------")
    message(STATUS "CMake info: ")
    message(STATUS "---------------------------------------------------------------")
    message(STATUS)
    message(STATUS "CMAKE_VERSION                        :  ${CMAKE_VERSION}")
    message(STATUS "CMAKE_COMMAND                        :  ${CMAKE_COMMAND}") 
    message(STATUS "CMAKE_AR                             :  ${CMAKE_AR}") 
    message(STATUS "CMAKE_ARGC                           :  ${CMAKE_ARGC}") 
    message(STATUS "CMAKE_ARGV0                          :  ${CMAKE_ARGV0}") 
    message(STATUS "BUILD_SHARED_LIBS                    :  ${BUILD_SHARED_LIBS}") 
    message(STATUS "CMAKE_ROOT                           :  ${CMAKE_ROOT}")
    message(STATUS "CMAKE_FOLDER                         :  ${CMAKE_FOLDER}") 
    message(STATUS "CMAKE_SYSROOT                        :  ${CMAKE_SYSROOT}") 
    message(STATUS "CMAKE_SYSROOT_COMPILE                :  ${CMAKE_SYSROOT_COMPILE}") 
    message(STATUS "CMAKE_SYSROOT_LINK                   :  ${CMAKE_SYSROOT_LINK}") 
    message(STATUS "CMAKE_HOME_DIRECTORY                 :  ${CMAKE_HOME_DIRECTORY}") 
    message(STATUS "CMAKE_IGNORE_PATH                    :  ${CMAKE_IGNORE_PATH}") 
    message(STATUS "CMAKE_PREFIX_PATH                    :  ${CMAKE_PREFIX_PATH}") 
    message(STATUS "CMAKE_PROGRAM_PATH                   :  ${CMAKE_PROGRAM_PATH}") 
    message(STATUS "CMAKE_CONFIGURATION_TYPES            :  ${CMAKE_CONFIGURATION_TYPES}") 
    message(STATUS "CMAKE_POSITION_INDEPENDENT_CODE      :  ${CMAKE_POSITION_INDEPENDENT_CODE}") 
    message(STATUS "CMAKE_JOB_POOLS                      :  ${CMAKE_JOB_POOLS}") 
    message(STATUS "CMAKE_JOB_POOL_COMPILE               :  ${CMAKE_JOB_POOL_COMPILE}") 
    message(STATUS "CMAKE_JOB_POOL_LINK                  :  ${CMAKE_JOB_POOL_LINK}") 
    message(STATUS "CMAKE_COLOR_MAKEFILE                 :  ${CMAKE_COLOR_MAKEFILE}") 
    message(STATUS "CMAKE_OBJECT_PATH_MAX                :  ${CMAKE_OBJECT_PATH_MAX}") 
    message(STATUS "CMAKE_SOURCE_DIR                     :  ${CMAKE_SOURCE_DIR}") 
    message(STATUS "CMAKE_BINARY_DIR                     :  ${CMAKE_BINARY_DIR}") 
    message(STATUS "CMAKE_CURRENT_SOURCE_DIR             :  ${CMAKE_CURRENT_SOURCE_DIR}") 
    message(STATUS "CMAKE_CURRENT_BINARY_DIR             :  ${CMAKE_CURRENT_BINARY_DIR}") 
    message(STATUS "CMAKE_BUILD_TOOL                     :  ${CMAKE_BUILD_TOOL}") 
    message(STATUS "CMAKE_BUILD_TYPE                     :  ${CMAKE_BUILD_TYPE}") 
    message(STATUS "CMAKE_MAKE_PROGRAM                   :  ${CMAKE_MAKE_PROGRAM}")
    message(STATUS "CMAKE_GENERATOR                      :  ${CMAKE_GENERATOR}")
    message(STATUS "CMAKE_GENERATOR_INSTANCE             :  ${CMAKE_GENERATOR_INSTANCE}")
    message(STATUS "CMAKE_GENERATOR_PLATFORM             :  ${CMAKE_GENERATOR_PLATFORM}")
    message(STATUS "CMAKE_GENERATOR_TOOLSET              :  ${CMAKE_GENERATOR_TOOLSET}")
    message(STATUS "CMAKE_DIRECTORY_LABELS               :  ${CMAKE_DIRECTORY_LABELS}")
    message(STATUS "CMAKE_LIBRARY_ARCHITECTURE           :  ${CMAKE_LIBRARY_ARCHITECTURE}") 
    message(STATUS "CMAKE_LIBRARY_ARCHITECTURE_REGEX     :  ${CMAKE_LIBRARY_ARCHITECTURE_REGEX}") 
    message(STATUS)
endmacro()
macro(print_cmake_x_build_info)
    message(STATUS "---------------------------------------------------------------")
    message(STATUS "CMake x-platform build informations: ")
    message(STATUS "---------------------------------------------------------------")
    message(STATUS)
    message(STATUS "CMAKE_CROSSCOMPILING                 :  ${CMAKE_CROSSCOMPILING}") 
    message(STATUS "CMAKE_TOOLCHAIN_FILE                 :  ${CMAKE_TOOLCHAIN_FILE}")
    message(STATUS "CMAKE_SYSTEM                         :  ${CMAKE_SYSTEM}")
    message(STATUS "CMAKE_SYSTEM_NAME                    :  ${CMAKE_SYSTEM_NAME}")
    message(STATUS "CMAKE_SYSTEM_VERSION                 :  ${CMAKE_SYSTEM_VERSION}")
    message(STATUS "CMAKE_SYSTEM_PROCESSOR               :  ${CMAKE_SYSTEM_PROCESSOR}")
    message(STATUS "CMAKE_HOST_SYSTEM_NAME               :  ${CMAKE_HOST_SYSTEM_NAME}")
    message(STATUS "CMAKE_HOST_SYSTEM_NAME               :  ${CMAKE_HOST_SYSTEM_NAME}")
    message(STATUS "CMAKE_HOST_SYSTEM_VERSION            :  ${CMAKE_HOST_SYSTEM_VERSION}")
    message(STATUS "CMAKE_HOST_SYSTEM_PROCESSOR          :  ${CMAKE_HOST_SYSTEM_PROCESSOR}")
    message(STATUS "CMAKE_CXX_COMPILER_ARCHITECTURE_ID   :  ${CMAKE_CXX_COMPILER_ARCHITECTURE_ID}")
    message(STATUS)
endmacro()
macro(print_cpack_info)
    message(STATUS "---------------------------------------------------------------")
    message(STATUS "CPack & installation info: ")
    message(STATUS "---------------------------------------------------------------")
    message(STATUS)
    message(STATUS "CMAKE_CPACK_COMMAND                            :  ${CMAKE_CPACK_COMMAND}") 
    message(STATUS "CPACK_INSTALL_SCRIPT                           :  ${CPACK_INSTALL_SCRIPT}") 
    message(STATUS "CPACK_INSTALL_DEFAULT_DIRECTORY_PERMISSIONS    :  ${CPACK_INSTALL_DEFAULT_DIRECTORY_PERMISSIONS}") 
    message(STATUS "CPACK_PACKAGING_INSTALL_PREFIX                 :  ${CPACK_PACKAGING_INSTALL_PREFIX}") 
    message(STATUS "CPACK_ABSOLUTE_DESTINATION_FILES               :  ${CPACK_ABSOLUTE_DESTINATION_FILES}") 
    message(STATUS "CPACK_WARN_ON_ABSOLUTE_INSTALL_DESTINATION     :  ${CPACK_WARN_ON_ABSOLUTE_INSTALL_DESTINATION}") 
    message(STATUS "CPACK_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION    :  ${CPACK_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION}") 
    message(STATUS "CPACK_SET_DESTDIR                              :  ${CPACK_SET_DESTDIR}") 
    message(STATUS "CPACK_INCLUDE_TOPLEVEL_DIRECTORY               :  ${CPACK_INCLUDE_TOPLEVEL_DIRECTORY}") 
    message(STATUS "CPACK_COMPONENT_INCLUDE_TOPLEVEL_DIRECTORY     :  ${CPACK_COMPONENT_INCLUDE_TOPLEVEL_DIRECTORY}") 
    message(STATUS "INSTALL_BIN_DIR                                :  ${INSTALL_BIN_DIR}") 
    message(STATUS "INSTALL_LIB_DIR                                :  ${INSTALL_LIB_DIR}") 
    message(STATUS "INSTALL_CMAKE_DIR                              :  ${INSTALL_CMAKE_DIR}") 
    message(STATUS "INSTALL_INCLUDE_DIR                            :  ${INSTALL_INCLUDE_DIR}") 
    message(STATUS "CMAKE_INSTALL_PREFIX                           :  ${CMAKE_INSTALL_PREFIX}") 
    message(STATUS)
endmacro()
macro(print_system_variables)
    message(STATUS "---------------------------------------------------------------")
    message(STATUS "CMake system variables: ")
    message(STATUS "---------------------------------------------------------------")
    message(STATUS)
    message(STATUS "CMAKE_GNUtoMS            :  ${CMAKE_GNUtoMS}") 
    message(STATUS "CMAKE_CL_64              :  ${CMAKE_CL_64}") 
    message(STATUS "CMAKE_COMPILER_2005      :  ${CMAKE_COMPILER_2005}") 
    message(STATUS "CMAKE_HOST_APPLE         :  ${CMAKE_HOST_APPLE}") 
    message(STATUS "CMAKE_HOST_SOLARIS       :  ${CMAKE_HOST_SOLARIS}") 
    message(STATUS "CMAKE_HOST_UNIX          :  ${CMAKE_HOST_UNIX}") 
    message(STATUS "CMAKE_HOST_WIN32         :  ${CMAKE_HOST_WIN32}") 
    message(STATUS "ANDROID                  :  ${ANDROID}") 
    message(STATUS "APPLE                    :  ${APPLE}") 
    message(STATUS "BORLAND                  :  ${BORLAND}") 
    message(STATUS "CYGWIN                   :  ${CYGWIN}") 
    message(STATUS "GHS-MULTI                :  ${GHS-MULTI}") 
    message(STATUS "IOS                      :  ${IOS}") 
    message(STATUS "MINGW                    :  ${MINGW}") 
    message(STATUS "MSVC                     :  ${MSVC}") 
    message(STATUS "MSVC10                   :  ${MSVC10}") 
    message(STATUS "MSVC11                   :  ${MSVC11}") 
    message(STATUS "MSVC12                   :  ${MSVC12}") 
    message(STATUS "MSVC14                   :  ${MSVC14}") 
    message(STATUS "MSVC60                   :  ${MSVC60}") 
    message(STATUS "MSVC70                   :  ${MSVC70}") 
    message(STATUS "MSVC71                   :  ${MSVC71}") 
    message(STATUS "MSVC80                   :  ${MSVC80}") 
    message(STATUS "MSVC90                   :  ${MSVC90}") 
    message(STATUS "MSVC_IDE                 :  ${MSVC_IDE}") 
    message(STATUS "MSVC_TOOLSET_VERSION     :  ${MSVC_TOOLSET_VERSION}") 
    message(STATUS "MSVC_VERSION             :  ${MSVC_VERSION}") 
    message(STATUS "MSYS                     :  ${MSYS}") 
    message(STATUS "UNIX                     :  ${UNIX}") 
    message(STATUS "WIN32                    :  ${WIN32}") 
    message(STATUS "WINCE                    :  ${WINCE}") 
    message(STATUS "WINDOWS_PHONE            :  ${WINDOWS_PHONE}") 
    message(STATUS "WINDOWS_STORE            :  ${WINDOWS_STORE}") 
    message(STATUS "XCODE                    :  ${XCODE}") 
    message(STATUS "XCODE_VERSION            :  ${XCODE_VERSION}") 
    message(STATUS)
endmacro()
macro(print_project_info)
    message(STATUS "---------------------------------------------------------------")
    message(STATUS "CMake Project Info:             <<<<    ${PROJECT_NAME}    >>>>")
    message(STATUS "---------------------------------------------------------------")
    message(STATUS)
    message(STATUS "${PROJECT_NAME}_VERSION_MAJOR   :  ${${PROJECT_NAME}_VERSION_MAJOR}")
    message(STATUS "${PROJECT_NAME}_VERSION_MINOR   :  ${${PROJECT_NAME}_VERSION_MINOR}")
    message(STATUS "${PROJECT_NAME}_VERSION_PATCH   :  ${${PROJECT_NAME}_VERSION_PATCH}")
    message(STATUS "${PROJECT_NAME}_VERSION_TWEAK   :  ${${PROJECT_NAME}_VERSION_TWEAK}")
    message(STATUS "${PROJECT_NAME}_DESCRIPTION     :  ${${PROJECT_NAME}_DESCRIPTION}")
    message(STATUS "${PROJECT_NAME}_HOMEPAGE_URL    :  ${${PROJECT_NAME}_HOMEPAGE_URL}")
    message(STATUS "${PROJECT_NAME}_VERSION         :  ${${PROJECT_NAME}_VERSION}")
    message(STATUS "${PROJECT_NAME}_SOURCE_DIR      :  ${${PROJECT_NAME}_SOURCE_DIR}")
    message(STATUS "${PROJECT_NAME}_BINARY_DIR      :  ${${PROJECT_NAME}_BINARY_DIR}")
    message(STATUS)
endmacro()

macro(print_all_cmake_info)
    message(STATUS "---------------------------------------------------------------")
    message(STATUS "---------------------------------------------------------------")
    message(STATUS)
    message(STATUS "Printing... ")
    message(STATUS)
    print_general_cmake_info()
    print_cmake_x_build_info()
    print_cpack_info()
    print_system_variables()
    print_project_info()
    message(STATUS "---------------------------------------------------------------")
    message(STATUS "---------------------------------------------------------------")
endmacro()

function(print_option_wanted_status option_name)
    set(NAME ${option_name})
    message(STATUS "ASKED FOR OPTION : ${NAME} == ${${NAME}} (to enable, type -D${NAME}=1 after `cmake` cmd. ")
    unset(NAME)
endfunction()

#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#%%    Helper functions       %%%%%%%%%%%%%%%     END       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#############################################################################################
###    Version and Installation settings    #################################################
#############################################################################################

set(${PROJECT_NAME}_MAJOR_VERSION 1)
set(${PROJECT_NAME}_MINOR_VERSION 4)
set(${PROJECT_NAME}_PATCH_VERSION 0)
set(${PROJECT_NAME}_VERSION
        ${${PROJECT_NAME}_MAJOR_VERSION}.${${PROJECT_NAME}_MINOR_VERSION}.${${PROJECT_NAME}_PATCH_VERSION})

# Offer the user the choice of overriding the installation directories
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR include CACHE PATH "Installation directory for header files")

if(WIN32 AND NOT CYGWIN)
    set(DEF_INSTALL_CMAKE_DIR CMake)
else()
    set(DEF_INSTALL_CMAKE_DIR lib/CMake/${PROJECT_NAME})
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH
        "Installation directory for CMake files")


#############################################################################################
###    General Settings    ##################################################################
#############################################################################################

if (POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW) # Use <PackageName>_ROOT variables instead of (env. var.) ## Default: CMake 3.12
endif ()
if (POLICY CMP0076)
    cmake_policy(SET CMP0076 NEW) # Changes absolute paths to relative ones                 ## Default: CMake 3.13
endif ()

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_INSTALL_MESSAGE ALWAYS)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(${PROJECT_NAME}_DLL_TO_INSTALL "")

print_all_cmake_info()

#############################################################################################
###    Threads    ###########################################################################
#############################################################################################

SET(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
print_package_info("Threads")

#############################################################################################
###    Boost       ##########################################################################
#############################################################################################
if(WIN32)
    set(Boost_INSTALLED_PATH "C:/Program Files/boost")
    
    set(Boost_DETAILED_FAILURE_MSG OFF)
    set(Boost_DEBUG OFF)
    
    set(Boost_USE_STATIC_LIBS ON)
    #set(Boost_USE_STATIC_RUNTIME ON)  
    
    set(Boost_USE_MULTITHREADED ON)      
    set(Boost_MULTITHREADED_LIB_APPEND "")      
    if(Boost_USE_MULTITHREADED)
        set(Boost_MULTITHREADED_LIB_APPEND "-mt")      
    endif()
    set(Boost_VERSION_INTERNAL_USE_ONLY "-1_67")        
    set(Boost_COMPILER "-vc141")        
    set(Boost_ARCHITECTURE "-x64")
    
    set(BOOST_ROOT "${Boost_INSTALLED_PATH}")
    set(BOOST_LIBRARYDIR "${BOOST_ROOT}/lib")
    set(BOOST_INCLUDEDIR "${BOOST_ROOT}/include/boost${Boost_VERSION_INTERNAL_USE_ONLY}")

    macro(format_boost_lib_name wanted)
	    set(FORMATED_${wanted} "${BOOST_LIBRARYDIR}/libboost_${wanted}${Boost_COMPILER}${Boost_MULTITHREADED_LIB_APPEND}${Boost_ARCHITECTURE}${Boost_VERSION_INTERNAL_USE_ONLY}.lib")
    endmacro()

    format_boost_lib_name(regex)
    format_boost_lib_name(thread)
    format_boost_lib_name(system)
    format_boost_lib_name(filesystem)
    format_boost_lib_name(date_time)
    format_boost_lib_name(signals)
    list(APPEND ${PROJECT_NAME}_DLL_TO_INSTALL
	    "${FORMATED_regex}"
	    "${FORMATED_thread}"
	    "${FORMATED_system}"
	    "${FORMATED_filesystem}"
	    "${FORMATED_date_time}"
	    "${FORMATED_signals}"
    )
    
endif()

message(STATUS "Note that ${PROJECT_NAME} will not work with boost version > 1.68")
message(STATUS "For Boost installation on Windows that actually work with this CMake, see : https://gist.github.com/JBHarvey/e49348b03cf0898c929a107852c43e22")

find_package(
    Boost 
    COMPONENTS 
        regex
        thread
        system
        filesystem
        date_time
        signals 
    REQUIRED
)
print_package_info("Boost")

include_directories(${Boost_INCLUDE_DIRS})

# TODO: check if second find_package(Threads) really is necessary. It shouldn't
find_package(Threads)
print_package_info("Threads")

#############################################################################################
###    Qt       #############################################################################
#############################################################################################
if(WIN32)
    set(Qt5_INSTALLED_PATH "D:/Qt/5.12.2/msvc2017_64")
    
    # From --> https://doc.qt.io/qt-5/cmake-manual.html
    set(OLD_CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH})
    set(CMAKE_PREFIX_PATH "${Qt5_INSTALLED_PATH}")
    
    set(Qt5_DIR "${Qt5_INSTALLED_PATH}/lib/cmake/Qt5")
    set(Qt5Widgets_DIR "${Qt5_DIR}Widgets")
    set(Qt5Gui_DIR "${Qt5_DIR}Gui")
    set(Qt5Core_DIR "${Qt5_DIR}Core")

    macro(format_qt5_lib_name wanted)
	    set(FORMATED_${wanted} "${Qt5_INSTALLED_PATH}/bin/Qt5${wanted}.dll")
    endmacro()

    format_qt5_lib_name(Core)
    format_qt5_lib_name(Cored)
    format_qt5_lib_name(Gui)
    format_qt5_lib_name(Guid)
    format_qt5_lib_name(Widgets)
    format_qt5_lib_name(Widgetsd)
    format_qt5_lib_name(Svg)
    format_qt5_lib_name(Svgd)
    format_qt5_lib_name(OpenGL)
    format_qt5_lib_name(OpenGLd)
    list(APPEND ${PROJECT_NAME}_DLL_TO_INSTALL
	    "${FORMATED_Core}"
	    "${FORMATED_Cored}"
	    "${FORMATED_Gui}"
	    "${FORMATED_Guid}"
	    "${FORMATED_Widgets}"
	    "${FORMATED_Widgetsd}"
	    "${FORMATED_Svg}"
	    "${FORMATED_Svgd}"
	    "${FORMATED_OpenGL}"
	    "${FORMATED_OpenGLd}"
    )

endif()

# These code will enable the auto MOC, UIC and RCC feature.
# See http://www.cmake.org/cmake/help/v3.0/manual/cmake-qt.7.html
SET(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTOUIC ON)
SET(CMAKE_AUTORCC ON)

# From https://blog.kitware.com/cmake-finding-qt5-the-right-way/
find_package(
    Qt5 
    COMPONENTS
        Core
        Gui
        Widgets
    REQUIRED
)
if(NOT Qt5Core_FOUND)
    message(STATUS "If you need to install Qt5 ---> https://www.qt.io/download-qt-installer?hsCtaTracking=9f6a2170-a938-42df-a8e2-a9f0b1d6cdce%7C6cb0de4f-9bb5-4778-ab02-bfb62735f3e5")
endif()
print_package_info("Qt5")
print_package_info("Qt5Core")
print_package_info("Qt5Gui")
print_package_info("Qt5Widgets")
set(CMAKE_PREFIX_PATH ${OLD_CMAKE_PREFIX_PATH})

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Qt5Widgets_INCLUDES also includes header files for its dependencies (QtCore et QtGui)
include_directories(${Qt5Widgets_INCLUDES})

add_definitions(${Qt5Widgets_DEFINITIONS})

# Creation of executable fails in Qt5 with default configuration
enable_cxx_compiler_flag_if_supported("${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")

#############################################################################################
###    Qwt      #############################################################################
#############################################################################################
if(WIN32)
    set(Qwt_INSTALLED_PATH "C:/Qwt-6.1.4")
else()
    set(Qwt_INSTALLED_PATH "linux/path/to/QWT")
endif()
list(APPEND ${PROJECT_NAME}_DLL_TO_INSTALL
    "${Qwt_INSTALLED_PATH}/lib/qwt.dll"
    "${Qwt_INSTALLED_PATH}/lib/qwt.lib"
)

file(GLOB Qwt_ROOT_DIR "${Qwt_INSTALLED_PATH}")
file(GLOB Qwt_INCLUDE_DIRS "${Qwt_ROOT_DIR}/include")
file(GLOB Qwt_LIBRARY_DIRS "${Qwt_ROOT_DIR}/lib")

# This function will search in different places, but ${CMAKE_FIND_ROOT_PATH} is targetted here
# source : https://cmake.org/cmake/help/latest/command/find_library.html#command:find_library
set(OLD_CMAKE_FIND_ROOT_PATH ${CMAKE_FIND_ROOT_PATH})
set(CMAKE_FIND_ROOT_PATH ${Qwt_ROOT_DIR})
set(Qwt_LIBRARY_NAME "qwt")

find_library(
    Qwt_LIBRARIES             # Variable in which the lib will be stored
    ${Qwt_LIBRARY_NAME}       # Name to look for
)
if(NOT Qwt_LIBRARIES)
    message(STATUS "For Qwt... if you need to ")
    message(STATUS "           - download --> https://sourceforge.net/projects/qwt/files/qwt/")
    message(STATUS "           -  install --> https://qwt.sourceforge.io/qwtinstall.html")
endif()

set(CMAKE_FIND_ROOT_PATH ${OLD_CMAKE_FIND_ROOT_PATH})
include_directories(include ${Qwt_INCLUDE_DIRS} )

#############################################################################################
###    CuteDemo    ##########################################################################
#############################################################################################

set(CORE_FILES_DIR "AWLCoreClasses")
set(SCOPE_FILES_DIR "AWLQtScope")
include_directories(include ${CORE_FILES_DIR})
include_directories(include ${SCOPE_FILES_DIR})

set(CUTEDEMO_MAIN_FILES "")
list(APPEND CUTEDEMO_MAIN_FILES 
    main.cpp
    awlqtdemo.cpp
    AWLSettings.cpp
    fov_2dscan.cpp
    awlplotscan.cpp
    TableView.cpp
)

set(CUTEDEMO_QT_UI_FILES "")
list(APPEND CUTEDEMO_QT_UI_FILES 
    awlqtdemo.ui
    fov_2dscan.ui
    TableView.ui
    awlplotscan.ui
)

set(CUTEDEMO_CORE_FILES "")
list(APPEND CUTEDEMO_CORE_FILES 
    ${CORE_FILES_DIR}/awlcoord.cpp
    ${CORE_FILES_DIR}/Publisher.cpp
    ${CORE_FILES_DIR}/ReceiverPostProcessor.cpp
    ${CORE_FILES_DIR}/CoordinateSystem.cpp
        ${CORE_FILES_DIR}/ReceiverCANCapture.cpp
        ${CORE_FILES_DIR}/ReceiverSimulatorCapture.cpp
        ${CORE_FILES_DIR}/DebugPrintf.cpp
        ${CORE_FILES_DIR}/ReceiverCapture.cpp
        ${CORE_FILES_DIR}/Subscription.cpp
        ${CORE_FILES_DIR}/DetectionStruct.cpp
        ${CORE_FILES_DIR}/ThreadedWorker.cpp
        ${CORE_FILES_DIR}/LoopedWorker.cpp
)

set(CUTEDEMO_SCOPE_FILES "")
list(APPEND CUTEDEMO_SCOPE_FILES 
        ${SCOPE_FILES_DIR}/awlqtscope.cpp
        ${SCOPE_FILES_DIR}/knob.cpp
        ${SCOPE_FILES_DIR}/awlscopeplot.cpp
        ${SCOPE_FILES_DIR}/signaldata.cpp
        ${SCOPE_FILES_DIR}/curvedata.cpp
        ${SCOPE_FILES_DIR}/wheelbox.cpp
        ${SCOPE_FILES_DIR}/awlqtscope.ui
)

set(CUTEDEMO_RESOURCES CuteDemo.rc)
set(CUTEDEMO_QRESOURCES awlqtdemo.qrc)
if(WIN32)
    ADD_EXECUTABLE(
    ${PROJECT_NAME}
        WIN32
        ${CUTEDEMO_MAIN_FILES}
        ${CUTEDEMO_QT_UI_FILES}
        ${CUTEDEMO_CORE_FILES}
        ${CUTEDEMO_SCOPE_FILES}
        ${CUTEDEMO_RESOURCES}
        ${CUTEDEMO_QRESOURCES}
    )
    
    TARGET_COMPILE_DEFINITIONS(
        ${PROJECT_NAME}
            PUBLIC 
                WINDOWS 
                QWT_DLL
    )
else()
    ADD_EXECUTABLE(
    ${PROJECT_NAME}
        ${CUTEDEMO_MAIN_FILES}
        ${CUTEDEMO_QT_UI_FILES}
        ${CUTEDEMO_CORE_FILES}
        ${CUTEDEMO_SCOPE_FILES}
        ${CUTEDEMO_RESOURCES}
        ${CUTEDEMO_QRESOURCES}
    )

    TARGET_COMPILE_DEFINITIONS(
        ${PROJECT_NAME}
        PUBLIC 
            QWT_DLL
    )

endif()

target_include_directories(
    ${PROJECT_NAME}
    PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${CORE_FILE_DIR}>
        $<INSTALL_INTERFACE:${INSTALL_CMAKE_DIR}/${CORE_FILE_DIR}>
)
target_include_directories(
    ${PROJECT_NAME}
    PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${SCOPE_FILE_DIR}>
        $<INSTALL_INTERFACE:${INSTALL_CMAKE_DIR}/${SCOPE_FILE_DIR}>
)
target_include_directories(
    ${PROJECT_NAME}
    PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${DEMO_FILE_DIR}>
        $<INSTALL_INTERFACE:${INSTALL_CMAKE_DIR}/${DEMO_FILE_DIR}>
)

#############################################################################################
###    Option description       #############################################################
#############################################################################################
# Set WANT_EASY_SYNC_SUPPORT to `1` to enable EasySync support
set(WANT_EASY_SYNC_SUPPORT 1)

# Set WANT_KVASER_SUPPORT to `1` to enable Kvaser support
set(WANT_KVASER_SUPPORT 0)

# Set WANT_SOCKET_CAN_SUPPORT to `1` to enable SocketCAN support WARNING Linux Only
set(WANT_SOCKET_CAN_SUPPORT 0)

# Set WANT_POSIX_UDP_SUPPORT to `1` to enable PosixUDP support WARNING Linux Only
set(WANT_POSIX_UDP_SUPPORT 0)

# Set WANT_POSIX_TTY_SUPPORT to `1` to enable PosixTTY support WARNING Linux Only
set(WANT_POSIX_TTY_SUPPORT 0)

# Set WANT_LIBUSB_SUPPORT to `1` to enable LibUSB support
set(WANT_LIBUSB_SUPPORT 1)

# Set WANT_TCP_SUPPORT to `1` to enable TCP_SUPPORT
set(WANT_TCP_SUPPORT 0)

# Set WANT_OPENCV_VIDEO true to enable OpenCV Video (and OpenCV in general)
set(WANT_OPENCV_VIDEO 0)

# Set WANT_AP_VIDEO to `1` to enable AP video
set(WANT_AP_VIDEO 0)

#############################################################################################
###    EasySync    ##########################################################################
#############################################################################################

print_option_wanted_status("WANT_EASY_SYNC_SUPPORT")
if(${WANT_EASY_SYNC_SUPPORT})
    TARGET_COMPILE_DEFINITIONS(
        CuteDemo
        PUBLIC 
            USE_CAN_EASYSYNC 
    )
    TARGET_SOURCES(
        CuteDemo
    PRIVATE 
            ${CORE_FILES_DIR}/ReceiverEasySyncCapture.cpp 
    )
endif()

#############################################################################################
###    Kvaser (canlib)    ###################################################################
#############################################################################################

print_option_wanted_status("WANT_KVASER_SUPPORT")
if(${WANT_KVASER_SUPPORT})
    file(
        GLOB
        CAN_INCLUDE_DIR 
            "/usr/local/include/canlib"
    )
    file(
        GLOB
        CAN_LIBRARIES 
            "/usr/local/lib/libcan*"
    )
    include_directories(
        include
        ${CAN_INCLUDE_DIR}
    )


    TARGET_COMPILE_DEFINITIONS(
        CuteDemo
        PUBLIC 
            USE_CAN_KVASER 
    )

    TARGET_SOURCES(
        CuteDemo
    PRIVATE 
            ${CORE_FILES_DIR}/ReceiverKvaserCapture.cpp 
    )

    TARGET_LINK_LIBRARIES(
        CuteDemo
        ${CAN_LIBRARIES}
    )
endif()

#############################################################################################
###    socket can    ########################################################################
#############################################################################################

print_option_wanted_status("WANT_SOCKET_CAN_SUPPORT")
if(${WANT_SOCKET_CAN_SUPPORT})
    TARGET_COMPILE_DEFINITIONS(
        CuteDemo
        PUBLIC 
            USE_CAN_SOCKETCAN 
    )
    TARGET_SOURCES(
        CuteDemo
    PRIVATE 
            ${CORE_FILES_DIR}/ReceiverSocketCANCapture.cpp 
    )
endif()


#############################################################################################
###    POSIX UDP    #########################################################################
#############################################################################################

print_option_wanted_status("WANT_POSIX_UDP_SUPPORT")
if(${WANT_POSIX_UDP_SUPPORT})
    TARGET_COMPILE_DEFINITIONS(
        CuteDemo
        PUBLIC 
            USE_POSIXUDP 
    )
    TARGET_SOURCES(
        CuteDemo
    PRIVATE 
            ${CORE_FILES_DIR}/ReceiverPosixUDPCapture.cpp 
    )
endif()


#############################################################################################
###    POSIX TTY    #########################################################################
#############################################################################################

print_option_wanted_status("WANT_POSIX_TTY_SUPPORT")
if(${WANT_POSIX_TTY_SUPPORT})
    TARGET_COMPILE_DEFINITIONS(
        CuteDemo
        PUBLIC 
            USE_POSIXTTY 
    )
    TARGET_SOURCES(
        CuteDemo
    PRIVATE 
            ${CORE_FILES_DIR}/ReceiverPosixTTYCapture.cpp 
    )
endif()

#############################################################################################
###    libusb    ############################################################################
#############################################################################################

print_option_wanted_status("WANT_LIBUSB_SUPPORT")
if(${WANT_LIBUSB_SUPPORT})
    set(libusb_INSTALLED_PATH "C:/libusb")

    message(STATUS "If you need to install libusb-1.0 ---> http://libusb.sourceforge.net/api-1.0/")
    set(LIBUSB_ROOT "${libusb_INSTALLED_PATH}")
    set(LIBUSB_LIB "${LIBUSB_ROOT}/libusb-1.0.lib")
    set(LIBUSB_INCLUDE_DIRS "${LIBUSB_ROOT}/include")

    list(APPEND ${PROJECT_NAME}_DLL_TO_INSTALL
        "${LIBUSB_ROOT}/libusb-1.0.lib"
        "${LIBUSB_ROOT}/libusb-1.0.dll"
    )

    include_directories(${LIBUSB_INCLUDE_DIRS})

    TARGET_COMPILE_DEFINITIONS(
        CuteDemo
        PUBLIC 
            USE_LIBUSB 
    )
    TARGET_LINK_LIBRARIES( 
        CuteDemo 
        ${LIBUSB_LIB} 
    )
    TARGET_SOURCES(
        CuteDemo
    PRIVATE 
            ${CORE_FILES_DIR}/ReceiverPolledCapture.cpp 
    )
    TARGET_SOURCES(
        CuteDemo
    PRIVATE 
            ${CORE_FILES_DIR}/ReceiverLibUSBCapture.cpp 
    )
endif()

#############################################################################################
###    TCP    ###############################################################################
#############################################################################################

print_option_wanted_status("WANT_TCP_SUPPORT")
if(${WANT_TCP_SUPPORT})
    TARGET_COMPILE_DEFINITIONS(
        CuteDemo 
        PUBLIC 
            USE_TCP
    )
    TARGET_SOURCES(
        CuteDemo
    PRIVATE
            ${CORE_FILES_DIR}/ReceiverTCPCapture.cpp
    )
endif()

#############################################################################################
###    OpenCV    ############################################################################
#############################################################################################

print_option_wanted_status("WANT_OPENCV_VIDEO")
if(${WANT_OPENCV_VIDEO})
    TARGET_COMPILE_DEFINITIONS(
        CuteDemo 
        PUBLIC 
            USE_OPENCV_VIDEO
    )
    FIND_PACKAGE(OpenCV REQUIRED)
    TARGET_SOURCES(
        CuteDemo 
    PRIVATE 
            VideoCapture.cpp 
            VideoViewer.cpp
    )
    TARGET_LINK_LIBRARIES(
        CuteDemo 
        ${OpenCV_LIBS}
    )
endif()

#############################################################################################
###    AP Video    ##########################################################################
#############################################################################################

print_option_wanted_status("WANT_AP_VIDEO")
if(${WANT_AP_VIDEO})
    set(AP_FILES_DIR "AP")
    TARGET_COMPILE_DEFINITIONS(
        CuteDemo 
        PUBLIC
            USE_AP_VIDEO 
    )
    file(GLOB
        AP_LIBRARIES 
    "${AP_FILES_DIR}/lib/*"
    )
    TARGET_SOURCES( 
        CuteDemo 
    PRIVATE 
            APVideoCapture.cpp
            APVideoViewer.cpp 
    )
    TARGET_LINK_LIBRARIES(
        CuteDemo ${AP_LIBRARIES} 
    )

    INCLUDE_DIRECTORIES(${AP_FILES_DIR}/include)
endif()

#############################################################################################
###    Final linking...    ##################################################################
#############################################################################################

TARGET_LINK_LIBRARIES(
    CuteDemo 
    ${Qwt_LIBRARIES}
    ${Qt5Core_LIBRARIES}
    ${Qt5Gui_LIBRARIES}
    ${Qt5Widgets_LIBRARIES}
    ${Boost_LIBRARIES}
)

target_link_libraries(
    CuteDemo 
    Threads::Threads 
    Qt5::WinMain       # <-- New target available via find_package ( Qt5Core )
)


#############################################################################################
###    Installation directives    ###########################################################
#############################################################################################

set_target_properties(
    CuteDemo
    PROPERTIES
    OUTPUT_NAME "CuteDemo"
)

# Add all targets to the build-tree export set
export(TARGETS 
           ${PROJECT_NAME}
       FILE
           "${PROJECT_NAME}Targets.cmake"
)

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE ${PROJECT_NAME})

# Create the ${PROJECT_NAME}Config.cmake and ${PROJECT_NAME}Version files
file(RELATIVE_PATH REL_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/${INSTALL_CMAKE_DIR}" "${CMAKE_INSTALL_PREFIX}/${INSTALL_INCLUDE_DIR}")

# ... for the build tree
set(CONF_INCLUDE_DIRS "\${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")
configure_file(${PROJECT_NAME}Config.cmake.in
        "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" @ONLY)
# ... for the install tree
set(CONF_INCLUDE_DIRS "\${${PROJECT_NAME}_CMAKE_DIR}/${REL_INCLUDE_DIR}")
configure_file(${PROJECT_NAME}Config.cmake.in
        "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake" @ONLY)
# ... for both
configure_file(${PROJECT_NAME}ConfigVersion.cmake.in
        "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake" @ONLY)

# Install the ${PROJECT_NAME}Config.cmake and ${PROJECT_NAME}ConfigVersion.cmake
install(
    FILES
        "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake"
        "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION 
        "${INSTALL_CMAKE_DIR}" 
)

include(GNUInstallDirs)

##install(
#    TARGETS
#        ${PROJECT_NAME}
#    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} 
#    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} 
#    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} 
#)

install(
    TARGETS
        ${PROJECT_NAME}
    EXPORT
        ${PROJECT_NAME}Targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} 
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} 
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} 
)

install(
    EXPORT
        ${PROJECT_NAME}Targets
    FILE 
        ${PROJECT_NAME}Targets.cmake
    DESTINATION
        ${INSTALL_CMAKE_DIR}
)

# Configuration file... needed for adequate runtime
install(
    FILES
        AWLDemoSettings.xml
       ${CUTEDEMO_RESOURCES}
       ${CUTEDEMO_QRESOURCES}
       LICENSE.txt
       README.txt
    DESTINATION ${CMAKE_INSTALL_BINDIR}
)
    
install(
    DIRECTORY
        Images/
    DESTINATION ${CMAKE_INSTALL_BINDIR}/Images
)

# Installation of dll & other dependencies
if(WIN32)
    install(
        DIRECTORY
            ${Qt5_INSTALLED_PATH}/plugins/platforms
            ${Qt5_INSTALLED_PATH}/plugins/imageformats
        DESTINATION
            ${CMAKE_INSTALL_BINDIR}
    )

    # DLLs
    message(STATUS "${PROJECT_NAME}_DLL_TO_INSTALL ----> ${${PROJECT_NAME}_DLL_TO_INSTALL}")
    install(
        FILES
            ${${PROJECT_NAME}_DLL_TO_INSTALL}
        DESTINATION
            ${CMAKE_INSTALL_BINDIR}
    )
     
    # Custom installation of system requirements
    set(CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_SKIP TRUE)
    set(CMAKE_INSTALL_DEBUG_LIBRARIES TRUE)
    set(CMAKE_INSTALL_UCRT_LIBRARIES TRUE)
    include(InstallRequiredSystemLibraries)
        install(
            PROGRAMS
                ${CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS}
            DESTINATION 
                ${CMAKE_INSTALL_BINDIR}
        )
endif()

#############################################################################################
###    Packaging    #########################################################################
#############################################################################################
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE.txt")
set(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/README.txt")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Qt Demo application. Made with <3 at Phantom")
set(CPACK_PACKAGE_NAME "${PROJECT_NAME}")
set(CPACK_PACKAGE_VENDOR "Phantom Intelligence Inc.")
set(CPACK_PACKAGE_VERSION "${${PROJECT_NAME}_VERSION}")
set(CPACK_PACKAGE_VERSION_MAJOR "${${PROJECT_NAME}_VERSION_MAJOR}")
set(CPACK_PACKAGE_VERSION_MINOR "${${PROJECT_NAME}_VERSION_MINOR}")
set(CPACK_PACKAGE_VERSION_PATCH "${${PROJECT_NAME}_VERSION_PATCH}")
set(CPACK_MONOLITHIC_INSTALL TRUE)
set(CPACK_PACKAGE_INSTALL_DIRECTORY ${PROJECT_NAME})

string(TIMESTAMP TIMESTAMP_VALUE "%Y-%m-%dT%H-%M-%S")

if(WIN32)
    set(CPACK_GENERATOR "WIX")
    set(CPACK_PACKAGE_EXECUTABLES ${PROJECT_NAME} "CuteDemo")
    #set(CPACK_WIX_PRODUCT_GUID "") # TODO: Set those with actual values
    #set(CPACK_WIX_UPGRADE_GUID "") # TODO: Set those with actual values
    set(CPACK_PACKAGE_FILE_NAME
        "${PROJECT_NAME}-${${PROJECT_NAME}_VERSION}-${CMAKE_SYSTEM_NAME}-${CMAKE_CXX_COMPILER_ARCHITECTURE_ID}-${TIMESTAMP_VALUE}")
endif()

include(CPack)
# CPack Modules to investigate
#CPack
#CPackArchive
#CPackBundle
#CPackComponent
#CPackCygwin
#CPackDMG
#CPackDeb
#CPackFreeBSD
#CPackIFW
#CPackIFWConfigureFile
#CPackNSIS
#CPackNuGet
#CPackPackageMaker
#CPackProductBuild
#CPackRPM
#CPackWIX
