cmake_minimum_required(VERSION 3.0.2) # Mendatory because of Qt5 --> https://doc.qt.io/qt-5/cmake-manual.html#using-qt-5-with-cmake-older-than-2-8-9
project(CuteDemo C CXX)
set(CMAKE_COLOR_MAKEFILE ON)

#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#%%    Helper functions       %%%%%%%%%%%%%%%     START       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#%%     > to simplify reading and writing           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%P%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

include(CheckCXXCompilerFlag)
function(enable_cxx_compiler_flag_if_supported flag)
    message(STATUS "FLAG_CHECK! ----> Current flags    :  ${CMAKE_CXX_FLAGS}")
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
set(${PROJECT_NAME}_VERSION_MAJOR_CACHE "1" CACHE STRING "Major Version Number - First digit of version number")
set(${PROJECT_NAME}_VERSION_MINOR_CACHE "5" CACHE STRING "Minor Version Number - Second digit of version number")
set(${PROJECT_NAME}_VERSION_PATCH_CACHE "3" CACHE STRING "Patch Version Number - Third digit of version number")
set(${PROJECT_NAME}_VERSION_MAJOR ${${PROJECT_NAME}_VERSION_MAJOR_CACHE})
set(${PROJECT_NAME}_VERSION_MINOR ${${PROJECT_NAME}_VERSION_MINOR_CACHE})
set(${PROJECT_NAME}_VERSION_PATCH ${${PROJECT_NAME}_VERSION_PATCH_CACHE})

set(${PROJECT_NAME}_VERSION
        ${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR}.${${PROJECT_NAME}_VERSION_PATCH})


# Offer the user the choice of overriding the installation directories
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")


set(INSTALL_INCLUDE_DIR include CACHE PATH "Installation directory for header files")

if(WIN32 AND NOT CYGWIN)
	set(INSTALL_BIN_DIR ./ CACHE PATH "Installation directory for executables")
else()
	set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
endif()

if(WIN32 AND NOT CYGWIN)
    set(DEF_INSTALL_CMAKE_DIR CMake)
else()
    set(DEF_INSTALL_CMAKE_DIR lib/CMake/${PROJECT_NAME})
endif()

set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH "Installation directory for CMake files")


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
    set(Boost_INSTALLED_PATH  "$ENV{BOOST_DIR}")
    
    set(Boost_DETAILED_FAILURE_MSG OFF)
    set(Boost_DEBUG OFF)
    
    set(Boost_USE_STATIC_LIBS ON)
    #set(Boost_USE_STATIC_RUNTIME ON)  
    
    set(Boost_USE_MULTITHREADED ON)      
    set(Boost_MULTITHREADED_LIB_APPEND "")      
    if(Boost_USE_MULTITHREADED)
        set(Boost_MULTITHREADED_LIB_APPEND "-mt")      
    endif()
    set(Boost_VERSION_INTERNAL_USE_ONLY "-1_60")        
    set(Boost_COMPILER "-vc140")        
    set(Boost_ARCHITECTURE "32")
    
    set(BOOST_ROOT "${Boost_INSTALLED_PATH}")

     set(BOOST_LIBRARYDIR "${BOOST_ROOT}/lib${Boost_ARCHITECTURE}-msvc-14.0")

    set(BOOST_INCLUDEDIR "${Boost_INSTALLED_PATH}")


    macro(format_boost_lib_name wanted)
	#############################################################################################
	###  UNCOMMENT the following for manually compiled libraries. This is respects the nomenclature published by Boost
	###set(FORMATED_LIB_${wanted} "${BOOST_LIBRARYDIR}/libboost_${wanted}${Boost_COMPILER}${Boost_MULTITHREADED_LIB_APPEND}${Boost_ARCHITECTURE}${Boost_VERSION_INTERNAL_USE_ONLY}.lib")
	###set(FORMATED_DLL_${wanted} "${BOOST_LIBRARYDIR}/libboost_${wanted}${Boost_COMPILER}${Boost_MULTITHREADED_LIB_APPEND}${Boost_ARCHITECTURE}${Boost_VERSION_INTERNAL_USE_ONLY}.dll")
	###  Some versions of the BOOST precompiled Libraries do not respect the nomenclature published By Boost.  In this case uncomment the following 
		set(FORMATED_LIB_${wanted} "${BOOST_LIBRARYDIR}/libboost_${wanted}${Boost_COMPILER}${Boost_MULTITHREADED_LIB_APPEND}${Boost_VERSION_INTERNAL_USE_ONLY}.lib")
		set(FORMATED_DLL_${wanted} "${BOOST_LIBRARYDIR}/boost_${wanted}${Boost_COMPILER}${Boost_MULTITHREADED_LIB_APPEND}${Boost_VERSION_INTERNAL_USE_ONLY}.dll")
	#############################################################################################
    endmacro()


    format_boost_lib_name(regex)
    format_boost_lib_name(thread)
    format_boost_lib_name(system)
    format_boost_lib_name(filesystem)
    format_boost_lib_name(date_time)
    format_boost_lib_name(chrono)
    format_boost_lib_name(signals)

    list(APPEND ${PROJECT_NAME}_DLL_TO_INSTALL
	    "${FORMATED_DLL_regex}"
	    "${FORMATED_DLL_thread}"
	    "${FORMATED_DLL_system}"
	    "${FORMATED_DLL_filesystem}"
	    "${FORMATED_DLL_date_time}"
	    "${FORMATED_DLL_chrono}"
	    "${FORMATED_DLL_signals}"
       )
    
endif()

message(STATUS "Note that ${PROJECT_NAME} will not work with boost version > 1.68")
message(STATUS "For Boost installation on Windows that actually work with this CMake, see : https://gist.github.com/JBHarvey/e49348b03cf0898c929a107852c43e22")

list(APPEND Boost_LIBRARIES
	    "${FORMATED_LIB_regex}"
	    "${FORMATED_LIB_thread}"
	    "${FORMATED_LIB_system}"
	    "${FORMATED_LIB_filesystem}"
	    "${FORMATED_LIB_date_time}"
	    "${FORMATED_LIB_chrono}"
	    "${FORMATED_LIB_signals}"
       )
 
# TODO: check if second find_package(Threads) really is necessary. It shouldn't
#find_package(Threads)
#print_package_info("Threads")

#############################################################################################
###    Qt       #############################################################################
#############################################################################################
if(WIN32)
    set(Qt5_INSTALLED_PATH "$ENV{QTDIR}" CACHE STRING "Root path Path of the files")
    
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
    format_qt5_lib_name(PrintSupport)
    format_qt5_lib_name(PrintSupportd)
    list(APPEND ${PROJECT_NAME}_DLL_TO_INSTALL
	    "${FORMATED_Core}"
	    "${FORMATED_Gui}"
	    "${FORMATED_OpenGL}"
	    "${FORMATED_PrintSupport}"
	    "${FORMATED_Svg}"
	    "${FORMATED_Widgets}"
    )

endif()

# These code will enable the auto MOC, UIC and RCC feature.
# See http://www.cmake.org/cmake/help/v3.0/manual/cmake-qt.7.html
SET(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTOUIC ON)
SET(CMAKE_AUTORCC ON)

# From https://blog.kitware.com/cmake-finding-qt5-the-right-way/
find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

if(NOT Qt5Core_FOUND)
    message(STATUS "If you need to install Qt5 ---> https://www.qt.io/download-qt-installer?hsCtaTracking=9f6a2170-a938-42df-a8e2-a9f0b1d6cdce%7C6cb0de4f-9bb5-4778-ab02-bfb62735f3e5")
endif()
print_package_info("Qt5")
print_package_info("Qt5Core")
print_package_info("Qt5Gui")
print_package_info("Qt5Widgets")
set(CMAKE_PREFIX_PATH ${OLD_CMAKE_PREFIX_PATH})

# Creation of executable fails in Qt5 with default configuration
enable_cxx_compiler_flag_if_supported("${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
enable_cxx_compiler_flag_if_supported("-o2")
enable_cxx_compiler_flag_if_supported("-fp:fast")
enable_cxx_compiler_flag_if_supported("-Qfast_transcendentals")
enable_cxx_compiler_flag_if_supported("-opt:REF")

#############################################################################################
###    CuteDemo    ##########################################################################
#############################################################################################

set(CUTEDEMO_FILES_NAME "SensorCoreClasses")
set(CUTEDEMO_FILES_DIR "../AWLQtDemo")

set(CUTEDEMO_MAIN_FILES "")
list(APPEND CUTEDEMO_MAIN_FILES 
    ${CUTEDEMO_FILES_DIR}/main.cpp
    ${CUTEDEMO_FILES_DIR}/awlqtdemo.cpp
    ${CUTEDEMO_FILES_DIR}/AWLSettings.cpp
    ${CUTEDEMO_FILES_DIR}/fov_2dscan.cpp
    ${CUTEDEMO_FILES_DIR}/awlplotscan.cpp
    ${CUTEDEMO_FILES_DIR}/TableView.cpp
)

set(CUTEDEMO_QT_UI_FILES "")
list(APPEND CUTEDEMO_QT_UI_FILES 
    ${CUTEDEMO_FILES_DIR}/awlqtdemo.ui
    ${CUTEDEMO_FILES_DIR}/fov_2dscan.ui
    ${CUTEDEMO_FILES_DIR}/TableView.ui
    ${CUTEDEMO_FILES_DIR}/awlplotscan.ui
)

set(CUTEDEMO_RESOURCES ${CUTEDEMO_FILES_DIR}/CuteDemo.rc)
set(CUTEDEMO_QRESOURCES ${CUTEDEMO_FILES_DIR}/awlqtdemo.qrc)
#qt5_add_resources(CUTEDEMO_QT_UI_FILES CUTEDEMO_QRESOURCES)

if(WIN32)
    ADD_EXECUTABLE(
    ${PROJECT_NAME}
        WIN32
        ${CUTEDEMO_MAIN_FILES}
        ${CUTEDEMO_QT_UI_FILES}
        ${CUTEDEMO_RESOURCES}
        ${CUTEDEMO_QRESOURCES}
    )
    
    TARGET_COMPILE_DEFINITIONS(
        ${PROJECT_NAME}
            PUBLIC 
                WINDOWS 
    )
else()
    ADD_EXECUTABLE(
    ${PROJECT_NAME}
        ${CUTEDEMO_MAIN_FILES}
        ${CUTEDEMO_QT_UI_FILES}
        ${CUTEDEMO_RESOURCES}
        ${CUTEDEMO_QRESOURCES}
    )

endif()

target_include_directories(
   ${PROJECT_NAME}
    PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${CUTEDEMO_FILES_DIR}>
        $<INSTALL_INTERFACE:${INSTALL_CMAKE_DIR}/${CUTEDEMO_FILES_DIR}>
)

target_include_directories(
    ${PROJECT_NAME}
    PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${DEMO_FILE_DIR}>
        $<INSTALL_INTERFACE:${INSTALL_CMAKE_DIR}/${DEMO_FILE_DIR}>
)

target_include_directories(
   ${PROJECT_NAME}
    PRIVATE
        ${BOOST_INCLUDEDIR}
)

# Qt5Widgets_INCLUDES also includes header files for its dependencies (QtCore et QtGui)
target_include_directories(
   CuteDemo
    PRIVATE
        ${Qt5Widgets_INCLUDES}
)

#add_definitions(${Qt5Widgets_DEFINITIONS})
target_compile_definitions (
   CuteDemo
    PRIVATE
        ${Qt5Widgets_DEFINITIONS}
)

#############################################################################################
###    CuteDemo SensorCore Library   ########################################################
#############################################################################################

set(CORE_FILES_NAME "SensorCoreClasses")
set(CORE_FILES_LIB "SensorCoreClasses.lib")
set(CORE_FILES_DIR "../SensorCoreClasses")

set(CUTEDEMO_CORE_FILES "")
list(APPEND CUTEDEMO_CORE_FILES 
	${CORE_FILES_DIR}/CoordinateSystem.cpp
	${CORE_FILES_DIR}/DebugPrintf.cpp
	${CORE_FILES_DIR}/DetectionStruct.cpp
    ${CORE_FILES_DIR}/LoopedWorker.cpp
    ${CORE_FILES_DIR}/Publisher.cpp
    ${CORE_FILES_DIR}/ReceiverCANCapture.cpp
    ${CORE_FILES_DIR}/ReceiverCapture.cpp
    ${CORE_FILES_DIR}/ReceiverLibUSBCapture.cpp
    ${CORE_FILES_DIR}/ReceiverLibUSB2Capture.cpp
    ${CORE_FILES_DIR}/ReceiverPolledCapture.cpp
	${CORE_FILES_DIR}/ReceiverPostProcessor.cpp
    ${CORE_FILES_DIR}/ReceiverSimulatorCapture.cpp
	${CORE_FILES_DIR}/Sensorcoord.cpp
	${CORE_FILES_DIR}/SensorSettings.cpp
	${CORE_FILES_DIR}/Subscription.cpp
    ${CORE_FILES_DIR}/ThreadedWorker.cpp
    ${CORE_FILES_DIR}/pch.cpp
 )

if(WIN32)
    ADD_LIBRARY(
    ${CORE_FILES_NAME}
        STATIC
        ${CUTEDEMO_CORE_FILES}
<    )
    
    TARGET_COMPILE_DEFINITIONS(
        ${CORE_FILES_NAME}
            PUBLIC 
                WINDOWS 
    )
else()
    ADD_LIBRARY(
    ${CORE_FILES_NAME} STATIC
        ${CUTEDEMO_CORE_FILES}
    )
endif()

target_include_directories(
    ${CORE_FILES_NAME}
    PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${CORE_FILES_DIR}>
        $<INSTALL_INTERFACE:${INSTALL_CMAKE_DIR}/${CORE_FILES_DIR}>
)
target_include_directories(
    CuteDemo
    PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${CORE_FILES_DIR}>
        $<INSTALL_INTERFACE:${INSTALL_CMAKE_DIR}/${CORE_FILES_DIR}>
)

target_include_directories(
   ${CORE_FILES_NAME}
    PRIVATE
        ${BOOST_INCLUDEDIR}
)

   TARGET_LINK_LIBRARIES( 
        CuteDemo 
        ${CORE_FILES_NAME} 
    )


#############################################################################################
###    Option description       #############################################################
#############################################################################################

# Set WANT_LIBUSB_SUPPORT to `1` to enable LibUSB support
set(WANT_LIBUSB_SUPPORT 1)

# TODO: complete this one 
### (PS : opencv needs to be recompiled with the option that activates XIMEA in it (see -DWITH_XIMEA:BOOL=ON in CMakeLists.txt file of "D:\opencv\opencv\CMakeLists.txt"))
# Set WANT_OPENCV_VIDEO true to enable OpenCV Video (and OpenCV in general)
set(WANT_OPENCV_VIDEO 1)

#

#############################################################################################
###    libusb    ############################################################################
#############################################################################################

print_option_wanted_status("WANT_LIBUSB_SUPPORT")
if(${WANT_LIBUSB_SUPPORT})
    set(libusb_INSTALLED_PATH "$ENV{LIBUSB_DIR}")

    message(STATUS "If you need to install libusb-1.0 ---> http://libusb.sourceforge.net/api-1.0/")
    set(LIBUSB_ROOT "${libusb_INSTALLED_PATH}")
    set(LIBUSB_LIB "${LIBUSB_ROOT}/Win32/release/lib/libusb-1.0.lib")
	set(LIBUSB_DLL "${LIBUSB_ROOT}/Win32/release/dll/libusb-1.0.dll")
    set(LIBUSB_INCLUDE_DIRS "${LIBUSB_ROOT}/libusb")

    list(APPEND ${PROJECT_NAME}_DLL_TO_INSTALL
        "${LIBUSB_DLL}"
    )

    target_include_directories(
    ${CORE_FILES_NAME}
		PRIVATE
		 ${LIBUSB_INCLUDE_DIRS}
	)
	target_include_directories(
    ${PROJECT_NAME}
		PRIVATE
			${LIBUSB_INCLUDE_DIRS}
	)

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

    TARGET_SOURCES(
        CuteDemo
    PRIVATE 
            ${CORE_FILES_DIR}/ReceiverLibUSB2Capture.cpp 
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

    FIND_PACKAGE(OpenCV REQUIRED PATHS D:/opencv-3.1.0/Build)

    SET( OpenCV_LIBRARIES "${OpenCV_INSTALL_PATH}/lib/release/opencv_world346.lib")
   
 
    SET( OpenCV_INCLUDE_DIRS
       ${OpenCV_DIR}/;
       ${OpenCV_DIR}/include
       ${OpenCV_DIR}/include/opencv
       ${OpenCV_DIR}/include/opencv2
       ${OpenCV_DIR}/modules/flann/include
       ${OpenCV_DIR}/modules/core/include
       ${OpenCV_DIR}/modules/imgproc/include
       ${OpenCV_DIR}/modules/ml/include
       ${OpenCV_DIR}/modules/photo/include
       ${OpenCV_DIR}/modules/video/include
       ${OpenCV_DIR}/modules/dnn/include
       ${OpenCV_DIR}/modules/imgcodecs/include
       ${OpenCV_DIR}/modules/shape/include
       ${OpenCV_DIR}/modules/videoio/include
       ${OpenCV_DIR}/modules/highgui/include
       ${OpenCV_DIR}/modules/superres/include
       ${OpenCV_DIR}/modules/features2d/include
       ${OpenCV_DIR}/modules/calib3d/include
       ${OpenCV_DIR}/modules/objdetect/include
       ${OpenCV_DIR}/modules/stitching/include
       ${OpenCV_DIR}/modules/videostab/include
       ${OpenCV_DIR}/modules/world/include
       ${OpenCV_DIR}/modules/ts/include
     )

     target_include_directories(
        ${PROJECT_NAME}
  	    PRIVATE
			${OpenCV_INCLUDE_DIRS}
	)
  

 TARGET_SOURCES(
        CuteDemo 
    PRIVATE 
            ${CUTEDEMO_FILES_DIR}/VideoCapture.cpp 
            ${CUTEDEMO_FILES_DIR}/VideoViewer.cpp
    )

    TARGET_LINK_LIBRARIES(
        CuteDemo 
	${OpenCV_LIBRARIES}
    )

    list(APPEND ${PROJECT_NAME}_DLL_TO_INSTALL
	"${OpenCV_INSTALL_PATH}/bin/release/opencv_world346.dll"
)

    print_package_info(OpenCV)


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

include(GNUInstallDirs)

install(
    TARGETS
        ${PROJECT_NAME}
    EXPORT
        ${PROJECT_NAME}Targets
    LIBRARY DESTINATION ${INSTALL_LIB_DIR} 
    ARCHIVE DESTINATION ${INSTALL_LIB_DIR} 
    RUNTIME DESTINATION ${INSTALL_BIN_DIR} 
)

set_property(
    INSTALL
        "${INSTALL_BIN_DIR}/$<TARGET_FILE_NAME:${PROJECT_NAME}>"
    PROPERTY CPACK_START_MENU_SHORTCUTS "${PROJECT_NAME}"
)
# Configuration file... needed for adequate runtime
install(
    FILES
        ${CUTEDEMO_FILES_DIR}/AWLDemoSettings.xml
		${CUTEDEMO_RESOURCES}
        ${CUTEDEMO_QRESOURCES}
#       ${CUTEDEMO_FILES_DIR}/LICENSE.txt
 #      ${CUTEDEMO_FILES_DIR}/README.txt
    DESTINATION ${INSTALL_BIN_DIR}
)
    
install(
    DIRECTORY
        ${CUTEDEMO_FILES_DIR}/Images/
    DESTINATION ${INSTALL_BIN_DIR}/Images
)

install(
    DIRECTORY
        ${CUTEDEMO_FILES_DIR}/../Licenses/
    DESTINATION ${INSTALL_BIN_DIR}/Licenses
)

# Installation of dll & other dependencies
if(WIN32)
    install(
        DIRECTORY
            ${Qt5_INSTALLED_PATH}/plugins/platforms
            ${Qt5_INSTALLED_PATH}/plugins/platformthemes
            ${Qt5_INSTALLED_PATH}/plugins/imageformats
            ${Qt5_INSTALLED_PATH}/plugins/styles
        DESTINATION
            ${INSTALL_BIN_DIR}
    )

    # DLLs
    message(STATUS "${PROJECT_NAME}_DLL_TO_INSTALL ----> ${${PROJECT_NAME}_DLL_TO_INSTALL}")
    install(
        FILES
            ${${PROJECT_NAME}_DLL_TO_INSTALL}
        DESTINATION
            ${INSTALL_BIN_DIR}
    )
     
    # Custom installation of system requirements
    set(CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_SKIP TRUE)
    set(CMAKE_INSTALL_DEBUG_LIBRARIES TRUE)
    set(CMAKE_INSTALL_UCRT_LIBRARIES TRUE)

	# under Windows 10, all API-MS*.dll are installed by default.  Do not install
 #JYD   include(InstallRequiredSystemLibraries)
 #JYD       install(
 #JYD           PROGRAMS
 #JYD               ${CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS}
 #JYD           DESTINATION 
 #JYD               ${INSTALL_BIN_DIR}
 #JYD       )
endif()

#############################################################################################
###    Packaging    #########################################################################
#############################################################################################
#set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE.txt")
#set(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/README.txt")

set(PACKAGE_VENDOR_NAME "Phantom Intelligence" CACHE STRING "Company Making the install")
set(PACKAGE_VENDOR_PATH "Phantom Intelligence" CACHE STRING "Company Installation Path")

set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Qt Demo application. Made with <3 at ${PACKAGE_VENDOR_NAME}")
set(CPACK_PACKAGE_NAME "${PROJECT_NAME}")
set(CPACK_PACKAGE_VENDOR "${PACKAGE_VENDOR_NAME}")
set(CPACK_PACKAGE_VERSION "${${PROJECT_NAME}_VERSION}")
set(CPACK_PACKAGE_VERSION_MAJOR "${${PROJECT_NAME}_VERSION_MAJOR}")
set(CPACK_PACKAGE_VERSION_MINOR "${${PROJECT_NAME}_VERSION_MINOR}")
set(CPACK_PACKAGE_VERSION_PATCH "${${PROJECT_NAME}_VERSION_PATCH}")
set(CPACK_MONOLITHIC_INSTALL FALSE)
set(CPACK_PACKAGE_INSTALL_DIRECTORY ${PROJECT_NAME})

string(TIMESTAMP TIMESTAMP_VALUE "%Y-%m-%dT%H-%M-%S")

if(WIN32)
    set(CPACK_GENERATOR "WIX")

    #set(CPACK_WIX_PRODUCT_GUID "") # TODO: Set those with actual values
    #set(CPACK_WIX_UPGRADE_GUID "") # TODO: Set those with actual values

    #TODO : check if this effectively changes the "C:\Program Files\CuteDemo" for "C:\Program Files\PhantomIntelligence\CuteDemo"
    #set(CPACK_PACKAGE_EXECUTABLES ${PROJECT_NAME} "${PACKAGE_VENDOR_PATH}/CuteDemo")
    set(CPACK_PACKAGE_EXECUTABLES ${PROJECT_NAME} "CuteDemo")
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
