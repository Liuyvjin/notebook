# Cmake常用变量

#### 目录

[TOC]

## 一、cmake变量的引用方式

使用${}进行变量的引用。在 IF 等语句中,是直接使用变量名而不通过${}取值

## 二、cmake常用变量

### 1. CMAKE_BINARY_DIR| PROJECT_BINARY_DIR| <projectname\>_BINARY_DIR
这三个变量指代的内容是一致的,如果是 in source 编译,指得就是工程顶层目录,如果是 out-of-source 编译,指的是工程编译发生的目录。PROJECT_BINARY_DIR 跟其他指令稍有区别,现在,你可以理解为他们是一致的。

### 2. CMAKE_SOURCE_DIR|PROJECT_SOURCE_DIR|<projectname\>_SOURCE_DIR
这三个变量指代的内容是一致的,不论采用何种编译方式,都是工程顶层目录。
也就是在 in source 编译时,他跟 CMAKE_BINARY_DIR 等变量一致。
PROJECT_SOURCE_DIR 跟其他指令稍有区别,现在,你可以理解为他们是一致的。

### 3. CMAKE_CURRENT_SOURCE_DIR

指的是当前处理的 CMakeLists.txt 所在的路径,比如上面我们提到的 src 子目录。

### 4. CMAKE_CURRRENT_BINARY_DIR

如果是 in-source 编译,它跟 CMAKE_CURRENT_SOURCE_DIR 一致,如果是 out-of-source 编译,他指的是 target 输出目录。

使用我们上面提到的 ADD_SUBDIRECTORY(src bin)可以更改这个变量的值。

使用 SET(EXECUTABLE_OUTPUT_PATH <新路径>)并不会对这个变量造成影响,它仅仅修改了最终目标文件存放的路径。

### 5. CMAKE_CURRENT_LIST_FILE

调用这个变量的 CMakeLists.txt 的完整路径 

### 6. CMAKE_CURRENT_LIST_LINE

输出这个变量所在的行

### 7. CMAKE_MODULE_PATH

这个变量用来定义自己的 cmake 模块所在的路径。如果你的工程比较复杂,有可能会自己编写一些 cmake 模块,这些 cmake 模块是随你的工程发布的,为了让 cmake 在处理CMakeLists.txt 时找到这些模块,你需要通过 SET 指令,将自己的 cmake 模块路径设置一下。

比如

```cmake
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
```

这时候你就可以通过 INCLUDE 指令来调用自己的模块了。

### 8. EXECUTABLE_OUTPUT_PATH | LIBRARY_OUTPUT_PATH

分别用来重新定义最终结果的存放目录,前面我们已经提到了这两个变量。

### 9. PROJECT_NAME

返回通过 PROJECT 指令定义的项目名称。

## 三、cmake调用环境变量的方式

使用`$ENV{NAME}`指令就可以调用系统的环境变量了。

设置环境变量的方式是: `SET(ENV{变量名} 值)`

1.CMAKE_INCLUDE_CURRENT_DIR
自动添加 CMAKE_CURRENT_BINARY_DIR 和 CMAKE_CURRENT_SOURCE_DIR 到当前处理
的 CMakeLists.txt。相当于在每个 CMakeLists.txt 加入:

```cmake
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
```

2.CMAKE_INCLUDE_DIRECTORIES_PROJECT_BEFORE
将工程提供的头文件目录始终至于系统头文件目录的前面,当你定义的头文件确实跟系统发生冲突时可以提供一些帮助。

3.CMAKE_INCLUDE_PATH 和 CMAKE_LIBRARY_PATH 我们在上一节已经提及。

## 四、系统信息

1. CMAKE_MAJOR_VERSION,CMAKE 主版本号,比如 2.4.6 中的 2
2. CMAKE_MINOR_VERSION,CMAKE 次版本号,比如 2.4.6 中的 4
3. CMAKE_PATCH_VERSION,CMAKE 补丁等级,比如 2.4.6 中的 6
4. CMAKE_SYSTEM,系统名称,比如 Linux-2.6.22
5. CMAKE_SYSTEM_NAME,不包含版本的系统名,比如 Linux
6. CMAKE_SYSTEM_VERSION,系统版本,比如 2.6.22
7. CMAKE_SYSTEM_PROCESSOR,处理器名称,比如 i686.
8. UNIX,在所有的类 UNIX 平台为 TRUE,包括 OS X 和 cygwin
9. WIN32,在所有的 win32 平台为 TRUE,包括 cygwin

## 五、主要的开关选项

1. CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS 用来控制 IF ELSE 语句的书写方式,在下一节语法部分会讲到。

2. BUILD_SHARED_LIBS:这个开关用来控制默认的库编译方式,如果不进行设置,使用 ADD_LIBRARY 并没有指定库类型的情况下,默认编译生成的库都是静态库。如果 SET(BUILD_SHARED_LIBS ON)后,默认生成的为动态库。

3. CMAKE_C_FLAGS: 设置 C 编译选项,也可以通过指令 ADD_DEFINITIONS()添加。

4. CMAKE_CXX_FLAGS: 设置 C++编译选项,也可以通过指令 ADD_DEFINITIONS()添加。