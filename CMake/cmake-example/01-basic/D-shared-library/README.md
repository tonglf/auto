# Shared Library

## Introduction

显示了一个hello world示例，该示例首先创建并链接一个共享库。

这还显示了如何创建别名目标

本教程中的文件如下：

```bash
$ tree
.
├── CMakeLists.txt
├── include
│   └── shared
│       └── Hello.h
└── src
    ├── Hello.cpp
    └── main.cpp
```

  * CMakeLists.txt - Contains the CMake commands you wish to run
  * include/shared/Hello.h - The header file to include
  * src/Hello.cpp - A source file to compile
  * src/main.cpp - The source file with main


## Concepts

### Adding a Shared Library

与前面关于静态库的示例一样，add_library() 函数也用于从一些源文件创建共享库。这被称为

```cmake
add_library(hello_library SHARED
    src/Hello.cpp
)
```

这将用于创建名为 libhello_library.so 的共享库，并将源代码传递给 add_library() 函数。

### Alias Target

顾名思义，别名目标是目标的替代名称，可以在只读上下文中代替真实的目标名称。

```cmake
add_library(hello::library ALIAS hello_library)
```

如下所示，这允许您在将目标链接到其他目标时使用别名来引用该目标。

> 注：不使用别名也可以，可以将这句注释掉，在下面链接动态库时，库名不是`hello::library`而是`hello_library`。

### Linking a Shared Library

链接共享库与链接静态库相同。创建可执行文件时，使用 target_link_library() 函数指向您的库

```cmake
add_executable(hello_binary
    src/main.cpp
)

target_link_libraries(hello_binary
    PRIVATE
        hello::library
)
```

这会告诉CMake使用别名将 hello_library 与 hello_binary 可执行文件链接。

链接器调用的一个例子是：

```bash
/usr/bin/c++ CMakeFiles/hello_binary.dir/src/main.cpp.o -o hello_binary -rdynamic libhello_library.so -Wl,-rpath,/home/matrim/workspace/cmake-examples/01-basic/D-shared-library/build
```

## Building the Example

```bash
$ mkdir build

$ cd build

$ cmake ..

$ make

$ ls
CMakeCache.txt  CMakeFiles  cmake_install.cmake  hello_binary  libhello_library.so  Makefile

$ ./hello_binary
Hello Shared Library!
```

