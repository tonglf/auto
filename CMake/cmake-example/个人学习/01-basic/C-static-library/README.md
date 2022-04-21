# Static Library

## Introduction

显示了一个hello world示例，该示例首先创建并链接一个静态库。这是一个简化的示例，显示了同一文件夹中的库和二进制文件。通常情况下，这些将在第02节“子项目”中所述的子项目中进行

本教程中的文件如下：

```bash
$ tree
.
├── CMakeLists.txt
├── include
│   └── static
│       └── Hello.h
└── src
    ├── Hello.cpp
    └── main.cpp
```

  * CMakeLists.txt - Contains the CMake commands you wish to run
  * include/static/Hello.h- The header file to include
  * src/Hello.cpp - A source file to compile
  * src/main.cpp - The source file with main


## Concepts

### Adding a Static Library

add_library() 函数用于从一些源文件创建库。这被称为：

```cmake
add_library(hello_library STATIC
    src/Hello.cpp
)
```

这将用于创建名为 libhello_library.a 的静态库与 add_library 调用中的源代码关联。

> [NOTE]
>
> 如前一个示例所述，我们将源文件直接传递给 add_library 调用，这是现代CMake的建议。

### Populating Including Directories

在本例中，我们使用 target_include_directories() 函数将目录包含在库中，其作用域设置为PUBLIC。

```cmake
target_include_directories(hello_library
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)
```

这将导致在以下位置使用包含的目录：

* 在编译库时
* 编译链接库的任何其他目标时。

范围的含义是：

* PRIVATE - 该目录被添加到此目标的include目录中
* INTERFACE - 该目录被添加到链接此库的任何目标的包含目录中
* PUBLIC - 如上所述，它包括在此库中，也包括链接此库的任何目标

> [TIP]
>
> 对于公共标题，最好将 include 文件夹与子目录“命名”。
>
> 传递到 target_include_directories 的目录将是包含目录树的根，C++文件应该包含从那里到您的头的路径。
>
> 在这个例子中，你可以看到我们的做法如下：
>
> ```cpp
> #include "static/Hello.h"
> ```
>
> 使用此方法意味着，在项目中使用多个库时，头文件名冲突的可能性较小。

### Linking a Library

当创建一个将使用你的库的可执行文件时，你必须告诉编译器关于库的信息。这可以使用target_link_libraries() 函数完成。

```cmake
add_executable(hello_binary
    src/main.cpp
)

target_link_libraries( hello_binary
    PRIVATE
        hello_library
)
```

这会告诉CMake在链接期间将 hello_library 与 hello_binary 二进制可执行文件链接。它还将从链接库目标传播任何具有 PUBLIC 或 INTERFACE 作用域的include目录。

编译器调用的一个例子是：

```bash
/usr/bin/c++ CMakeFiles/hello_binary.dir/src/main.cpp.o -o hello_binary -rdynamic libhello_library.a
```


## Building the Example

```bash
$ mkdir build

$ cd build

$ cmake ..

$ make

$ ls
CMakeCache.txt  CMakeFiles  cmake_install.cmake  hello_binary  libhello_library.a  Makefile

$ ./hello_binary
Hello Static Library!
```

