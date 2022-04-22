# Set C++ Standard

## Introduction

This example shows how to set the C++ standard using the `target_compile_features` function. This is available since CMake v3.1

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

### Using target_compile_features

在目标上调用 target_compile_features 函数将查看传入的特性，并确定用于目标的正确编译器标志。

```cmake
target_compile_features(hello_cpp11 PUBLIC cxx_auto_type)
```

As with other `target_*` functions, you can specify the scope of the feature for the selected target. This populates the [INTERFACE_COMPILE_FEATURES](https://cmake.org/cmake/help/v3.1/prop_tgt/INTERFACE_COMPILE_FEATURES.html#prop_tgt:INTERFACE_COMPILE_FEATURES) property for the target.

The list of available features can be found from the [CMAKE_CXX_COMPILE_FEATURES](https://cmake.org/cmake/help/v3.1/variable/CMAKE_CXX_COMPILE_FEATURES.html#variable:CMAKE_CXX_COMPILE_FEATURES) variable. You can obtain a list of the available features using the following code:

```cmake
message("List of compile features: ${CMAKE_CXX_COMPILE_FEATURES}")
```

## Building the Examples

Below is sample output from building this example.

```bash
$ mkdir build

$ cd build

$ cmake ..

$ make VERBOSE=1
```