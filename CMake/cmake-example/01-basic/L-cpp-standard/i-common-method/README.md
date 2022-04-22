# C++ Standard Common Method

## Introduction

This example shows a common method to set the C++ Standard. This can be used with most versions of CMake. However, if you are targeting a recent version of CMake there are more convenient methods available.

The files in this tutorial are below:

```bash
A-hello-cmake$ tree
.
├── CMakeLists.txt
├── main.cpp
```

- CMakeLists.txt - Contains the CMake commands you wish to run
- main.cpp - A simple "Hello World" cpp file targeting C++11.

## Concepts

### Checking Compile flags

CMake has support for attempting to compile a program with any flags you pass into the function `CMAKE_CXX_COMPILER_FLAG`. The result is then stored in a variable that you pass in.

For example:

```bash
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
```

This example will attempt to compile a program with the flag `-std=c++11` and store the result in the variable `COMPILER_SUPPORTS_CXX11`.

The line `include(CheckCXXCompilerFlag)` tells CMake to include this function to make it available for use.

### Adding the flag

Once you have determined if the compile supports a flag, you can then use the [standard cmake methods](https://github.com/ttroy50/cmake-examples/blob/master/01-basic/G-compile-flags) to add this flag to a target. In this example we use the `CMAKE_CXX_FLAGS` to propegate the flag to all targets .

```bash
if(COMPILER_SUPPORTS_CXX11)#
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)#
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()
```

The above example only checks for the gcc version of the compile flags and supports fallback from C++11 to the pre-standardisation C+\+0x flag. In real usage you may want to check for C14, or add support for different methods of setting the compile, e.g. `-std=gnu11`

## Building the Examples

Below is sample output from building this example.

```bash
$ mkdir build
$ cd build

$ cmake ..

$ make VERBOSE=1
```