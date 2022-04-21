# Compile Flags(编译标志)

## Introduction

CMake支持以多种不同的方式设置编译标志：

- using target_compile_definitions() function
- using the CMAKE_C_FLAGS and CMAKE_CXX_FLAGS variables.

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

### Set Per-Target C++ Flags

在现代CMake中设置C++标志的推荐方法是使用可以通过 target_compile_definitions() 函数填充到其他目标的每个目标标志。这将填充库的 INTERFACE_COMPILE_DEFINITIONS，并根据范围将定义推送到链接目标。

```cmake
target_compile_definitions(cmake_examples_compile_flags
    PRIVATE EX3
)
```

这将导致编译器在编译目标时添加定义-DEX3。

如果目标是一个库，并且选择了范围 PUBLIC or INTERFACE，那么定义也将包含在链接此目标的任何可执行文件中。

对于编译器选项，还可以使用 target_compile_options() 函数。

#### Set Default C++ Flags

The default `CMAKE_CXX_FLAGS` is either empty 或者包含适合生成类型的标志。

To set additional default compile flags you can add the following to your top level CMakeLists.txt

```cmake
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DEX2" CACHE STRING "Set C++ Compiler Flags" FORCE)
```

Similarly to CMAKE_CXX_FLAGS other options include:

- Setting C compiler flags using CMAKE_C_FLAGS
- Setting linker flags using CMAKE_LINKER_FLAGS.

> Note
>
> The values `CACHE STRING "Set C++ Compiler Flags" FORCE` from the above command are used to force this variable to be set in the CMakeCache.txt file.For more details, see [here](https://cmake.org/cmake/help/v3.0/command/set.html)

Once set the CMAKE_C_FLAGS and CMAKE_CXX_FLAGS will set a compiler flag / definition globally for all targets in this directory or any included sub-directories. This method is not recommended for general usage now and the target_compile_definitions function is preferred.

##### Set CMake Flags

Similar to the build type a global C++ compiler flag can be set using the following methods.

- Using a gui tool such as ccmake / cmake-gui

- Passing into cmake

```
cmake .. -DCMAKE_CXX_FLAGS="-DEX3"
```

## Building the Example

```bash
$ mkdir build

$ cd build/

$ cmake ..

$ make VERBOSE=1
```