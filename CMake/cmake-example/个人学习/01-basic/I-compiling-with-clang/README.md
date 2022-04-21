# Compiling with clang

Introduction

When building with CMake it is possible to set the C and C++ compiler. This example is the same as the [hello-cmake](https://github.com/ttroy50/cmake-examples/blob/master/01-basic/A-hello-cmake) example except that it shows the most basic method of changing the compiler from the default gcc to [clang](http://clang.llvm.org/).

The files in this tutorial are below:

```bash
$ tree
.
├── CMakeLists.txt
├── main.cpp
```

- CMakeLists.txt - Contains the CMake commands you wish to run
- main.cpp - A simple "Hello World" cpp file.

## Concepts

### Compiler Option

CMake exposes options to control the programs used to compile and link your code. These programs include:

- CMAKE_C_COMPILER - The program used to compile c code.
- CMAKE_CXX_COMPILER - The program used to compile c++ code.
- CMAKE_LINKER - The program used to link your binary.

> Note
>
> In this example clang-3.6 is installed via the command `sudo apt-get install clang-3.6`

> Note
>
> 这是调用clang最基本、最简单的方法。未来的示例将展示调用编译器的更好方法。

### Setting Flags

As described in the [Build Type](https://github.com/ttroy50/cmake-examples/blob/master/01-basic/F-build-type) example, you can set CMake options using either a cmake gui or by passing from the command line.

Below is an example of passing the compiler via the command line.

```bash
cmake .. -DCMAKE_C_COMPILER=clang-3.6 -DCMAKE_CXX_COMPILER=clang++-3.6
```

After setting these options, when your run `make` clang will be used to compile your binary. This can be seen from the following lines in the make output.

```bash
/usr/bin/clang++-3.6     -o CMakeFiles/hello_cmake.dir/main.cpp.o -c /home/matrim/workspace/cmake-examples/01-basic/I-compiling-with-clang/main.cpp
Linking CXX executable hello_cmake

/usr/bin/cmake -E cmake_link_script CMakeFiles/hello_cmake.dir/link.txt --verbose=1

/usr/bin/clang++-3.6       CMakeFiles/hello_cmake.dir/main.cpp.o  -o hello_cmake -rdynamic
```

# Building the Examples

Below is sample output from building this example.

```bash
$ mkdir build.clang

$ cd build.clang/

$ cmake .. -DCMAKE_C_COMPILER=clang-3.6 -DCMAKE_CXX_COMPILER=clang++-3.6

$ make VERBOSE=1

$ ./hello_cmake
Hello CMake!
```