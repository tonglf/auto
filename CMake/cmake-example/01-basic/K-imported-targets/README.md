# Imported Targets

## Introduction

As previously mentioned in the [third party library](https://github.com/ttroy50/cmake-examples/blob/master/01-basic/H-third-party-library), newer versions of CMake allow you to link third party libraries using [imported](https://cmake.org/cmake/help/v3.6/prop_tgt/IMPORTED.html#prop_tgt:IMPORTED) ALIAS targets.

The files in this tutorial are below:

```bash
$ tree
.
├── CMakeLists.txt
├── main.cpp
```

- CMakeLists.txt - Contains the CMake commands you wish to run
- main.cpp - The source file with main

## Requirements

This example requires the following:

- CMake v3.5+
- The boost libraries to be installed in a default system location.

## Concepts

### Imported Target

Imported targets are read-only targets that are exported by FindXXX modules.

To include boost filesystem you can do the following:

```cmake
  target_link_libraries( imported_targets
      PRIVATE
          Boost::filesystem
  )
```

This will automtaically link the Boost::filesystem and Boost::system libraries while also including the Boost include directories.

## Building the Example

```bash
$ mkdir build

$ cd build/

$ cmake ..

$ make

$ ./imported_targets
Hello Third Party Include!
Path is not relative
```