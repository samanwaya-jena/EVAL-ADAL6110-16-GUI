# Phantom Intelligence 2018-05-10  
# Phantom Intelligence 2020-04-16 Compile from ADI_Develop branche

### This is the README file for [AWLQtDemo](https://github.com/PhantomIntelligence/awlcutedemo.git)  

*** ADI_Develop branche ***

## 1 CONTENTS OF THE PACKAGE:

see [treefile.txt](treefile.txt)

## 2 SYSTEM REQUIREMENTS:

 - OpenCV library
 - Qt 5.0 library
 - Qwt library
 - Boost library
 - Canbus support
 - gcc - GNU Compiler Colection

## 3 INSTALLATION:

```bash
git clone https://github.com/PhantomIntelligence/awlcutedemo.git
git checkout ADI_Develop 
```

## 4 CODE COMPILATION:

#### For linux pc std architecture:
```bash
cp ../CMakeList/CMakeLists.txt.linux CMakeLists.txt  
cmake .
make
```

#### For petalinux on ultrascale board
```bash
cp ../CMakeList/CMakeLists.txt.ultrascale CMakeLists.txt
export QT_BIN_PATH=/usr/bin/qt5	
cmake .
make
```


## 5 RUNNING APPLICATION:

#### For linux pc std architecture:
```bash
cp AWLDemoSettings.xml.linux AWLDemoSettings.xml
./LinuxCute
```
#### For petalinux on ultrascale board
```bash
cp AWLDemoSettings.xml.ultrascale AWLDemoSettings.xml
./LinuxCute
```
