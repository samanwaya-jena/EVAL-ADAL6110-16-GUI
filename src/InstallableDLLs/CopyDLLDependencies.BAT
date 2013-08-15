
REM This batch file copies all DLLs required by the application 
REM to the current folder
REM
REM This assumes that the following environment variables have been set:
REM    QTDIR
REM    QWTDir
REM    PCL_Root
REM    OPEN_NI_BIN
REM    OPENCV_DIR
REM 

COPY "%OPENCV_DIR%"\bin\Debug\*.dll
COPY "%OPENCV_DIR%"\bin\Release\*.dll
COPY "%OPEN_NI_BIN%"\*.dll"
COPY "%OPEN_NI_BIN%"\*.config"
COPY "%PCL_Root%"\bin\*.dll
COPY "%QTDir%"\bin\*.dll
COPY "%QWTDir%"\lib\*.dll
COPY "C:\FTDI\DLLs\*.dll"
