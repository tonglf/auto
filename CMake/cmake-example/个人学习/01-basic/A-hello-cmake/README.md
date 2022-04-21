# Hello CMake

## 介绍

显示了一个非常基本的hello world示例。

本教程中的文件如下：

```bash
A-hello-cmake$ tree
.
├── CMakeLists.txt
├── main.cpp
```

  * CMakeLists.txt - 包含要运行的CMake命令
  * main.cpp - 一个简单的 "Hello World" cpp 文件

## 概念

### CMakeLists.txt

CMakeLists.txt 是存储所有CMake命令的文件。当cmake在文件夹中运行时，它将查找该文件，如果该文件不存在，cmake将以错误退出。

### Minimum CMake version

使用CMake创建项目时，可以指定支持的CMake的最低版本。

```cmake
cmake_minimum_required(VERSION 3.5)
```


### Projects

CMake构建可以包含一个项目名称，以便在使用多个项目时更容易引用某些变量。

```cmake
project (hello_cmake)
```


### Creating an Executable

add_executable() 命令指定应从指定的源文件生成可执行文件，在本例中为main.cpp。add_executable() 函数的第一个参数是要生成的可执行文件的名称，第二个参数是要编译的源文件列表。

```cmake
add_executable(hello_cmake main.cpp)
```



> [NOTE]
>
> 有些人使用的一种简写方法是将项目名称和可执行文件名称相同。这允许您指定 CmakeList.txt 如下：
>
> ```cmake
> cmake_minimum_required(VERSION 2.6)
> project (hello_cmake)
> add_executable(${PROJECT_NAME} main.cpp)
> ```
>
> 在本例中，project() 函数将创建一个值为 hello_cmake 的变量 ${project_NAME}。然后可以将其传递给add_executable() 函数，以输出 “hello_cmake” 可执行文件。


### Binary Directory

运行cmake命令的根文件夹或顶级文件夹称为cmake_BINARY_DIR，是所有二进制文件的根文件夹。CMake支持就地和异地构建和生成二进制文件。


#### In-Place Build

就地生成在与源代码相同的目录结构中生成所有临时生成文件。这意味着所有的makefile和object文件都散布在普通代码中。要创建就地构建目标，请在根目录中运行cmake命令。例如：

```bash
A-hello-cmake$ cmake .
```


#### Out-of-Source Build（推荐）

源代码外版本允许您创建单个版本文件夹，该文件夹可以位于文件系统的任何位置。所有临时生成和对象文件都位于该目录中，以保持源代码树的干净。要创建源代码外构建，请在build文件夹中运行cmake命令，并将其指向根所在的目录CMakeLists.txt文件。如果你想从头开始重新创建你的cmake环境，你只需要删除你的构建目录，然后重新运行cmake。

For example:

```bash
A-hello-cmake$ mkdir build

A-hello-cmake$ cd build/

A-hello-cmake/build$ cmake ..
```

所有的例子都将使用 out-of-source 方式编译。


# Building the Examples

下面是构建此示例的示例输出。

```bash
$ mkdir build

$ cd build

$ cmake ..

$ make

$ ./hello_cmake
Hello CMake!
```

