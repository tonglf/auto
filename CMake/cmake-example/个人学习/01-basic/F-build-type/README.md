# Build Type

## Introduction

CMake有许多内置配置，可用于编译项目。它们指定了优化级别，以及二进制文件中是否包含调试信息。

The levels provided are:

- Release - Adds the `-O3 -DNDEBUG` flags to the compiler
- Debug - Adds the `-g` flag
- MinSizeRel - Adds `-Os -DNDEBUG`
- RelWithDebInfo - Adds `-O2 -g -DNDEBUG` flags

The files in this tutorial are below:

```bash
$ tree
.
├── CMakeLists.txt
├── main.cpp
```

- CMakeLists.txt - Contains the CMake commands you wish to run
- main.cpp - The source file with main

## Concepts

### Set Build Type

The build type can be set using the following methods.

- Using a gui tool such as ccmake / 命令：`cmake-gui`

![cmake-gui-build-type.png)](cmake-gui-build-type.png)

- Passing into cmake

```bash
cmake .. -DCMAKE_BUILD_TYPE=Release
```

### Set Default Build Type

CMake提供的默认构建类型不包括用于优化的编译器标志。对于某些项目，您可能需要设置默认的生成类型，这样就不必记得设置它。

To do this you can add the following to your top level CMakeLists.txt

```cmake
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message("Setting build type to 'RelWithDebInfo' as none was specified.")
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()
```

## Building the Example

```bash
$ mkdir build

$ cd build/

$ cmake ..

$ make VERBOSE=1
```