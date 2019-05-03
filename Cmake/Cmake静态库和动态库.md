# Cmake构建静态库和动态库

#### 目录

[TOC]

## 1. 库的名称 

动态库：lib[name].so

静态库：lib[name].a

其中中间部分为target名。

## 2. 添加要编译的共享库

```cmake
ADD_LIBRARY(libname [SHARED|STATIC|MODULE][EXCLUDE_FROM_ALL] source1 source2 ... sourceN))
```

* 第一个参数libname是要生成的库的target名称，不需要填写全名，如果填hello，则生成的文件名为libname.so。

* 第二个参数是类型，有三种：
  * SHARED，动态库(扩展名为.so)
  * STATIC，静态库(扩展名为.a)
  * MODULE，在使用dyld的系统有效，如果不支持dyld，则被当作SHARED对待。

* 第三个参数意思是这个库不会被默认构建，除非有其他组件依赖或者手工构建。

编译后输出默认在主cmakelists通过add_sublib指定的输出目录中。

## 3. 解决静态库和动态库同名冲突

按照惯例，静态库和动态库应该同名，但是默认不允许同名。

```cmake
ADD_LIBRARY(hello SHARED ${LIBHELLO_SRC})
ADD_LIBRARY(hello STATIC ${LIBHELLO_SRC})
```

编译后发现没有生成静态库，因为lib文件夹下面已经有一个同名的动态库了，而默认的库名是不能相同的，因此构建指令无效。

**解决方案**：改target名字-->改target的输出文件名-->阻止清理同名target

```cmake
ADD_LIBRARY(hello_static STATIC ${LIBHELLO_SRC})  # 改target名
```

接下来要用到SET_TARGET_PROPERTIES修改输出文件名，它的用法是：

```cmake
SET_TARGET_PROPERTIES(target1 target2 ...PROPERTIES prop1 value1 prop2 value2 ...)  # 作用是设置一个目标属性的值
GET_TARGET_PROPERTY(var target property) # 获得一个目标属性的值存在var中
```

更改**OUTPUT_NAME**属性，将静态库库的名称设置为hello

```cmake
SET_TARGET_PROPERTIES(hello_static PROPERTIES OUTPUT_NAME "hello")
```

但是这还会引发一个问题，来检查一下最终的构建结果，我们发现，libhello.a已经构建完成，位于build/lib目录中，但是libhello.so消失了。这个问题的原因是：cmake在构建一个新的target时，会尝试清理掉其他使用这个名字的库，因为，在构建libhello.a时，就会清理掉libhello.so.为了回避这个问题，比如再次使用SET_TARGET_PROPERTIES定义**CLEAN_DIRECT_OUTPUT**属性，阻止清理。

向lib/CMakeLists.txt中添加:

```cmake
SET_TARGET_PROPERTIES(hello PROPERTIES CLEAN_DIRECT_OUTPUT 1)
SET_TARGET_PROPERTIES(hello_static PROPERTIES CLEAN_DIRECT_OUTPUT 1)
```

## 3. 控制版本号

动态库包含版本号

设置版本号属性：

```cmake
SET_TARGET_PROPERTIES(hello PROPERTIES VERSION 1.2 SOVERSION 1)
```

VERSION指代动态库版本，SOVERSION指代API版本。soversion也可以是除里数字的其他字符。

## 4. 安装库和头文件

将hello的共享库安装到<prefix>/lib目录，将hello.h安装到<prefix>/include/hello目录。

```cmake
INSTALL(TARGETS hello hello_static
    LIBRARY DESTINATION lib  # 动态库
    ARCHIVE DESTINATION lib)  # 静态库用archive关键字
INSTALL(FILES hello.h DESTINATION include/hello)  # 头文件
```

默认路径前缀

```cmake
cmake -CMAKE_INSTALL_PREFIX=/tmp/myusr ..
或者 SET(CMAKE_INSTALL_PREFIX /tmp/myusr)必须写在主CMakeLists.txt中
```

## 5. 添加头文件搜索路径

```cmake
INCLUDE_DIRECTORIES([AFTER|BEFORE] [SYSTEM] dir1 dir2 ...)
```

相当于`gcc -I`；或者CMAKE_INCLUDE_PATH和CMAKE_LIBRARY_PATH

参数`[AFTER|BEFORE]`指定自己添加的路径在默认路径前面还是后面

## 6. 添加库搜索路径，链接库

添加查找库的路径。相当于`-L`。

LINK_DIRECTORIES的全部语法是：

```cmake
LINK_DIRECTORIES(directory1 directory2 ...)  # 添加非标准库的路径
```

TARGET_LINK_LIBRARIES：链接库，相当于`-l`

```cmake
TARGET_LINK_LIBRARIES(target library1<debug | optimized> library2...)
```

在src/cmakelists.txt添加

```cmake
TARGET_LINK_LIBRARIES(main hello)  # 在link_directories指定目录找
也可以写成
TARGET_LINK_LIBRARIES(main libhello.so)  # 在默认路径中查找
```



## 7. 代码实例

[参考代码](./cmake_lib实例)

```cmake
PROJECT_BINARY_DIR  # 外部编译目录的绝对路径
PROJECT_SOURCE_DIR  # 主cmakelists所在的绝对路径
```

src/cmakelists

```cmake
ADD_EXECUTABLE(main main.c)  # 编译主程序
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)  # 更改输出路径
INCLUDE_DIRECTORIES(BEFORE ${PROJECT_SOURCE_DIR}/lib)  # 设置查找文件路径
LINK_DIRECTORIES(${PROJECT_BINARY_DIR}/lib)  # 链接库路径，程序运行的时候也在这里找
####
#注意，实际应用中有时是要写绝对路径的，因为库被安装在默认文件夹下，而不是编译路径下；这个时候就要FIND_PATH()
## find_path(
#            <VAR>  # 用来保存搜到的路径，如果没搜索到，就是notfound
#            name | NAMES name1 [name2 ...]  # 要搜索的文件名
#            [PATHS path1 [path2 ... ENV var]])  # 在哪里搜索
TARGET_LINK_LIBRARIES(main libhello.so) # (或hello)，链接哪个库，
#### 
#注意，如果用全名就是去系统默认目录查找库，比如/usr/lib/，如果找不到，就会报错，例如“/usr/bin/ld: 找不到 -lhello”,如果要到LINK_DIRECTORIES指定目录找，就要用target名，比如hello_static(静态)或hello(动态)，之后如果把动态库移动，程序会无法运行。
```

lib/cmakelists

```cmake
SET(LIBHELLO_SRC hello.c)  # 设置源代码变量
ADD_LIBRARY(hello SHARED ${LIBHELLO_SRC})  # 编译共享库(默认在当前文件夹下)
ADD_LIBRARY(hello_static STATIC ${LIBHELLO_SRC})  # 编译静态库
SET_TARGET_PROPERTIES(hello_static PROPERTIES OUTPUT_NAME hello)# 静态库输出名字改成hello
# 防止同名库被删除
SET_TARGET_PROPERTIES(hello_static PROPERTIES CLEAN_DIRECT_OUTPUT 1) 
SET_TARGET_PROPERTIES(hello PROPERTIES CLEAN_DIRECT_OUTPUT 1)
# 设置版本信息
SET_TARGET_PROPERTIES(hello PROPERTIES VERSION 1.2 SOVERSION 3)# 版本号可以非数字如s，链接的时候其实链接的是libname.so.3文件
#、安装库
INSTALL(TARGETS hello hello_static
    LIBRARY DESTINATION lib   # 动态库，安装在 CMAKE_INSTALL_PREFIX/lib
    ARCHIVE DESTINATION lib)  # 静态库用archive关键字
INSTALL(FILES hello.h DESTINATION include/hello) # 头文件，安装在 CMAKE_INSTALL_PREFIX/include/hello
```

cmakelists

```cmake
SET(CMAKE_INSTALL_PREFIX /tmp/myusr)  # 设置安装目录前缀
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)  
PROJECT(HELLOLIB) 
MESSAGE(STATUS "Binary dir is ${PROJECT_BINARY_DIR} ${HELLOLIB_BINARY_DIR}\n")  # 在camke ..的时候会输出提示，两者都是基于
MESSAGE(STATUS "Source dir is ${PROJECT_SOURCE_DIR}\n")
ADD_SUBDIRECTORY(lib)  # 默认输入是sourcedir/lib,输出是binary/lib
ADD_SUBDIRECTORY(src)  # 默认输出是source/lib ,输出是binary/lib
```

最后的文件结构

```shell
├── build  # 在此目录外部编译
│   ├── bin  
│   │   └── main
│   └── lib
│       ├── libhello.a
│       ├── libhello.so -> libhello.so.s
│       ├── libhello.so.1.2
│       └── libhello.so.s -> libhello.so.1.2
├── CMakeLists.txt
├── lib
│   ├── CMakeLists.txt
│   ├── hello.c
│   └── hello.h
└── src
    ├── CMakeLists.txt
    └── main
```

