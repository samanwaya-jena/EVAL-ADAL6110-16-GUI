
REM This batch file copies all DLLs required by the application 
REM to the current folder
REM
REM This assumes that the following environment variables have been set:
REM    QTDIR
REM    QWTDir
REM    OPENCV_DIR
REM    XIMEA_DIR
REM    CANLIB_DIR
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
COPY "%QWTDir%"\lib\qwt.dll
del *d.dll
COPY "%XIMEA_DIR%"\API\x86\m3api.dll
COPY "%XIMEA_DIR%"\API\x86\xiapi32.dll
COPY "%CANLIB_DIR%"\Bin\canlib32.dll
COPY "%LIBUSB_DIR%"\Win32\Release\dll\libusb-1.0.dll"
