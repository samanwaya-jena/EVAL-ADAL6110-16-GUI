
REM This batch file copies all DLLs required by the application 
REM to the current folder
REM
REM This assumes that the following environment variables have been set:
REM    QTDIR
REM    QWTDir
REM    OPENCV_DIR
REM 

COPY "%QTDir%"\bin\QT5core.dll
COPY "%QTDir%"\bin\QT5Gui.dll
COPY "%QTDir%"\bin\QT5OpenGL.dll
COPY "%QTDir%"\bin\QT5PrintSupport.dll
COPY "%QTDir%"\bin\QT5SVG.dll
COPY "%QTDir%"\bin\QT5Widgets.dll
COPY "%OPENCV_DIR%"\x86\vc12\bin\opencv_world300.dll
COPY "%QWTDir%"\build-qwt-Desktop_Qt_5_5_1_MSVC2013_32bit-Release\lib\qwt.dll
del *d.dll
COPY "C:\XIMEA\API\x86\m3api.dll"
COPY "C:\Program Files (x86)\Kvaser\Canlib\Bin\canlib32.dll"