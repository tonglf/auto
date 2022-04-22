# Hello Headers

## Introduction

显示了一个hello world示例，该示例使用不同的文件夹作为源文件和包含文件。

本教程中的文件包括：

```bash
B-hello-headers$ tree
.
├── CMakeLists.txt
├── include
│   └── Hello.h
└── src
    ├── Hello.cpp
    └── main.cpp
```

  * CMakeLists.txt - Contains the CMake commands you wish to run.
  * include/Hello.h - The header file to include.
  * src/Hello.cpp - A source file to compile.
  * src/main.cpp - The source file with main.


## Concepts

### Directory Paths

CMake语法指定了许多变量，可用于帮助在项目或源代码树中找到有用的目录。其中包括：

| Variable                 |                    Info                    |
| ------------------------ | :----------------------------------------: |
| CMAKE_SOURCE_DIR         |                  根源目录                  |
| CMAKE_CURRENT_SOURCE_DIR |     当前源目录（如果使用子项目和目录）     |
| PROJECT_SOURCE_DIR       |           当前cmake项目的源目录            |
| CMAKE_BINARY_DIR         | 根二进制/编译目录。这是运行cmake命令的目录 |
| CMAKE_CURRENT_BINARY_DIR |            您当前所在的生成目录            |
| PROJECT_BINARY_DIR       |             当前项目的生成目录             |

### Source Files Variable

创建一个包含源文件的变量，可以更清楚地了解这些文件，并轻松地将它们添加到多个命令中，例如add_executable() 函数。

```cmake
# Create a sources variable with a link to all cpp files to compile
set(SOURCES
    src/Hello.cpp
    src/main.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})
```

> [NOTE]
>
> 在SOURCES变量中设置特定文件名的另一种方法是使用GLOB命令使用通配符模式匹配查找文件。
>
> ```cmake
> file(GLOB SOURCES "src/*.cpp")
> ```
>

> [TIP]
>
> 对于现代CMake，不建议对源使用变量。相反，通常在add_xxx函数中直接声明源。
>
> 这对于glob命令尤其重要，如果添加新的源文件，glob命令可能不会始终显示正确的结果。

### Including Directories

当您有不同的包含文件夹时，可以使用 target_include_directories() 函数让编译器知道它们。编译此目标时，这将使用 -I 标志将这些目录添加到编译器中，例如 `-I/directory/path`

```cmake
target_include_directories(target
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)
```

`PRIVATE` 指定包含的范围。这对于库来说很重要，下一个例子将对此进行解释。有关该功能的更多详细信息，请参见[此处](https://cmake.org/cmake/help/v3.0/command/target_include_directories.html)

## Building the Example

### Standard Output

下面给出了构建此示例的标准输出。

```bash
$ mkdir build

$ cd build

$ cmake ..

$ make

$ ./hello_headers
Hello Headers!
```


### Verbose Output(详细输出)

在前面的示例中，当运行 make 命令时，输出仅显示生成的状态。要查看完整输出以进行调试，可以在运行 make 时添加 VERBOSE=1 标志。

详细的输出显示在下面，并且对输出的检查表明，将添加目录添加到C++编译器命令中。

```bash
$ make clean

$ make VERBOSE=1
```