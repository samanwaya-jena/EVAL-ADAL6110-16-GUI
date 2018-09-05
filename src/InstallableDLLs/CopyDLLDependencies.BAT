
REM This batch file copies all DLLs required by the application 
REM to the current folder
REM
REM This assumes that the following environment variables have been set:
REM    QTDIR
REM    QWTDir
REM    OPENCV_DIR
REM    XIMEA_DIR
REM 
MD platforms
COPY "%QTDir%"\bin\QT5core.dll
COPY "%QTDir%"\bin\QT5Gui.dll
COPY "%QTDir%"\bin\QT5OpenGL.dll
COPY "%QTDir%"\bin\QT5PrintSupport.dll
COPY "%QTDir%"\bin\QT5SVG.dll
COPY "%QTDir%"\bin\QT5Widgets.dll
COPY "%QTDir%"\plugins\platforms\qwindows.dll platforms
COPY "%OPENCV_DIR%"\bin\release\opencv_world310.dll
COPY "%QWTDir%"\lib\qwt.dll
del *d.dll
COPY "%XIMEA_DIR%"\API\x86\m3api.dll"
COPY "%XIMEA_DIR%"\API\x86\xiapi32.dll"
COPY "C:\Program Files (x86)\Kvaser\Canlib\Bin\canlib32.dll"
COPY "%LIBUSB_DIR%"\MS32\dll\libusb-1.0.dll"
