# Set C++ Standard

## Introduction

This example shows how to set the C++ standard using the `CMAKE_CXX_STANDARD` variable. This is available since CMake v3.1

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

### Using CXX_STANDARD property

Setting the [CMAKE_CXX_STANDARD](https://cmake.org/cmake/help/v3.1/variable/CMAKE_CXX_STANDARD.html) variable causes the `CXX_STANDARD` property on all targets. This causes CMake to set the appropriate flag at compille time.

>Note
>
>The `CMAKE_CXX_STANDARD` variable falls back to the closest appropriate standard without a failure. For example, if you request `-std=gnu11` you may end up with `-std=gnu0x`.This can result in an unexpected failure at compile time.

## Building the Examples

Below is sample output from building this example.

```bash
$ mkdir build

$ cd build

$ cmake ..

$ make VERBOSE=1
```