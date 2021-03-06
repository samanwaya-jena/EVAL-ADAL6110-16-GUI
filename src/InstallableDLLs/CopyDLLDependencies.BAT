
REM This batch file copies all DLLs required by the application 
REM to the current folder
REM
REM This assumes that the following environment variables have been set:
REM    QTDIR
REM    QWTDir
REM    OPENCV_DIR
REM 
MD platforms
COPY "%QTDir%"\bin\QT5core.dll
COPY "%QTDir%"\bin\QT5Gui.dll
COPY "%QTDir%"\bin\QT5OpenGL.dll
COPY "%QTDir%"\bin\QT5PrintSupport.dll
COPY "%QTDir%"\bin\QT5SVG.dll
COPY "%QTDir%"\bin\QT5Widgets.dll
COPY "%QTDir%"\plugins\platforms\qwindows.dll platforms
COPY "%OPENCV_DIR%"\bin\release\opencv_world346.dll
del *d.dll

COPY "%LIBUSB_DIR%"\Win32\Release\dll\libusb-1.0.dll"
