Phantom Intelligence 2018-05-10
This is the README file for AWLQtDemo
https://github.com/PhantomIntelligence/awlcutedemo.git
*** LinuxPort branche ***

1-CONTENTS OF THE PACKAGE:

see treefile.txt

2-SYSTEM REQUIREMENTS:

OpenCV library
Qt 5.0 library
Qwt library
Boost library
Canbus support
gcc - GNU Compiler Colection

3-INSTALLATION:

$git clone https://github.com/PhantomIntelligence/awlcutedemo.git
$git checkout LinuxPort


4-CODE COMPILATION:

For linux pc std architecture:
  $cp CMakeLists.txt.linux CMakeLists.txt
For petalinux on ultrascale board
  $cp CMakeLists.txt.ultrascale CMakeLists.txt
  $export QT_BIN_PATH=/usr/bin/qt5	

$cmake .
$make

5-RUNNING APPLICATION:

For linux pc std architecture:
  $cp AWLDemoSettings.xml.linux AWLDemoSettings.xml
For petalinux on ultrascale board
  $cp AWLDemoSettings.xml.ultrascale  AWLDemoSettings.xml

$./LinuxCute
