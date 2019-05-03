# Cmake常用指令

#### 目录

[TOC]

## 一、基本指令

### 1. ADD_DEFINITIONS

向 C/C++编译器添加-D 定义,比如:

```cmake
ADD_DEFINITIONS(-DENABLE_DEBUG  -DABC)  # 参数之间用空格分割。
```

如果你的代码中定义了这个，则`#ifdef ENABLE_DEBUG #endif`代码块就会生效。
如果要添加其他的编译器开关,可以通过 CMAKE_C_FLAGS 变量和 CMAKE_CXX_FLAGS 变量设置。

### 2. ADD_DEPENDENCIES
定义 target 依赖的其他 target,确保在编译本 target 之前,其他的 target 已经被构建。
```cmake
ADD_DEPENDENCIES(target-name depend-target1 depend-target2 ...)
```


### 3. ADD_EXECUTABLE、ADD_LIBRARY、ADD_SUBDIRECTORY 

添加可执行程序，库和子目录

### 4. ADD_TEST 与 ENABLE_TESTING 
`ENABLE_TESTING` 指令用来控制 Makefile 是否构建 test 目标,涉及工程所有目录。语法很简单,没有任何参数`ENABLE_TESTING()`,一般情况这个指令放在工程的主CMakeLists.txt 中。

`ADD_TEST` 指令的语法是:

```cmake
ADD_TEST(testname Exename arg1 arg2 ...)
```

testname 是自定义的 test 名称,Exename 可以是构建的目标文件也可以是外部脚本等等。后面连接传递给可执行文件的参数。如果没有在同一个 CMakeLists.txt 中打开ENABLE_TESTING()指令,任何 ADD_TEST 都是无效的。
比如我们前面的 Helloworld 例子,可以在工程主 CMakeLists.txt 中添加
ADD_TEST(mytest ${PROJECT_BINARY_DIR}/bin/main)
ENABLE_TESTING()
生成 Makefile 后,就可以运行 make test 来执行测试了。


### 5. AUX_SOURCE_DIRECTORY
基本语法是:
```cmake
AUX_SOURCE_DIRECTORY(dir VARIABLE)
```
作用是发现一个目录下所有的源代码文件并将列表存储在一个变量中,这个指令临时被用来自动构建源文件列表，因为目前 cmake 还不能自动发现新添加的源文件。
比如
```cmake
AUX_SOURCE_DIRECTORY(. SRC_LIST)
ADD_EXECUTABLE(main ${SRC_LIST})
```
你也可以通过后面提到的 FOREACH 指令来处理这个 LIST

### 6. CMAKE_MINIMUM_REQUIRED
其语法为 CMAKE_MINIMUM_REQUIRED(VERSION versionNumber [FATAL_ERROR])
比如 CMAKE_MINIMUM_REQUIRED(VERSION 2.5 FATAL_ERROR)
如果 cmake 版本小与 2.5,则出现严重错误,整个过程中止。

### 7. EXEC_PROGRAM
在 CMakeLists.txt 处理过程中执行命令,并不会在生成的 Makefile 中执行。

具体语法为:
```cmake
EXEC_PROGRAM(Executable [directory in which to run]
                [ARGS <arguments to executable>]
                [OUTPUT_VARIABLE <var>]
                [RETURN_VALUE <var>])
```
用于在指定的目录运行某个程序,通过 ARGS 添加参数,如果要获取输出和返回值,可通过OUTPUT_VARIABLE 和 RETURN_VALUE 分别定义两个变量.

这个指令可以帮助你在 CMakeLists.txt 处理过程中支持任何命令,比如根据系统情况去修改代码文件等等。
举个简单的例子,我们要在 src 目录执行 ls 命令,并把结果和返回值存下来。
可以直接在 src/CMakeLists.txt 中添加:
```cmake
EXEC_PROGRAM(ls ARGS "*.c" OUTPUT_VARIABLE LS_OUTPUT RETURN_VALUE
LS_RVALUE)
IF(not LS_RVALUE)
MESSAGE(STATUS "ls result: " ${LS_OUTPUT})
ENDIF(not LS_RVALUE)
```


在 cmake 生成 Makefile 的过程中,就会执行 ls 命令,如果返回 0,则说明成功执行,那么就输出 ls *.c 的结果。关于 IF 语句,后面的控制指令会提到。

### 8. FILE 指令
文件操作指令,基本语法为:
```cmake
FILE(WRITE filename "message to write"... )
FILE(APPEND filename "message to write"... )
FILE(READ filename variable)
FILE(GLOB variable [RELATIVE path] [globbing expression_r_rs]...)
FILE(GLOB_RECURSE variable [RELATIVE path] [globbing expression_r_rs]...)
FILE(REMOVE [directory]...)
FILE(REMOVE_RECURSE [directory]...)
FILE(MAKE_DIRECTORY [directory]...)
FILE(RELATIVE_PATH variable directory file)
FILE(TO_CMAKE_PATH path result)
FILE(TO_NATIVE_PATH path result)
```
这里的语法都比较简单,不在展开介绍了。

### 9. INCLUDE 指令

用来载入 CMakeLists.txt 文件,也用于载入预定义的 cmake 模块.

```cmake
INCLUDE(file1 [OPTIONAL])
INCLUDE(module [OPTIONAL])
```

OPTIONAL 参数的作用是文件不存在也不会产生错误。
你可以指定载入一个文件,如果定义的是一个模块,那么将在 CMAKE_MODULE_PATH 中搜索这个模块并载入。
载入的内容将在处理到 INCLUDE 语句时直接执行。

### 10. FIND_<\>指令
FIND_系列指令主要包含以下指令:

```cmake
FIND_FILE(<VAR> name1 path1 path2 ...)
# VAR 变量代表找到的文件全路径,包含文件名
FIND_LIBRARY(<VAR> name1 path1 path2 ...)
# VAR 变量表示找到的库全路径,包含库文件名
FIND_PATH(<VAR> name1 path1 path2 ...)
# VAR 变量代表包含这个文件的路径。
FIND_PROGRAM(<VAR> name1 path1 path2 ...)
# VAR 变量代表包含这个程序的全路径。
FIND_PACKAGE(<name> [major.minor] [QUIET] [NO_MODULE]
                [[REQUIRED|COMPONENTS] [componets...]])
```

用来调用预定义在 CMAKE_MODULE_PATH 下的 Find<name\>.cmake 模块,你也可以自己定义 Find<name\>模块,通过 SET(CMAKE_MODULE_PATH dir)将其放入工程的某个目录中供工程使用,我们在后面的章节会详细介绍 FIND_PACKAGE 的使用方法和 Find 模块的编写。

FIND_LIBRARY 示例:

```cmake
FIND_LIBRARY(libX X11 /usr/lib)
IF(NOT libX)
MESSAGE(FATAL_ERROR “libX not found”)
ENDIF(NOT libX)
```

 [回到目录](#目录)

## 二、控制指令:

### 1. IF 指令

#### (1) 基本语法为：

```cmake
IF(expression_r_r)
# THEN section.
COMMAND1(ARGS ...)
COMMAND2(ARGS ...)
...
ELSE(expression_r_r)
# ELSE section.
COMMAND1(ARGS ...)
COMMAND2(ARGS ...)
...
ENDIF(expression_r_r)
```
另外一个指令是 `ELSEIF`,总体把握一个原则,凡是出现 IF 的地方一定要有对应的ENDIF.出现 ELSEIF 的地方,ENDIF 是可选的。

#### (2) 表达式的使用方法如下:

```cmake
IF(var)  # 如果变量不是:空,0,N, NO, OFF, FALSE, NOTFOUND 或<var>_NOTFOUND 时,表达式为真。
IF(NOT var)  # 与上述条件相反。
IF(var1 AND var2)  # 当两个变量都为真是为真。
IF(var1 OR var2)  # 当两个变量其中一个为真时为真。
IF(COMMAND cmd)  # 当给定的 cmd 确实是命令并可以调用是为真。
IF(EXISTS dir)、IF(EXISTS file)  # 当目录名或者文件名存在时为真。
IF(file1 IS_NEWER_THAN file2)  # 当 file1 比 file2 新,或者 file1/file2 其中有一个不存在时为真,文件名请使用完整路径。
IF(IS_DIRECTORY dirname)  # 当 dirname 是目录时,为真。
IF(DEFINED variable)  # 如果变量被定义,为真。
```
正则表达式
```cmake
IF(variable MATCHES regex)、IF(string MATCHES regex)
# 当给定的变量或者字符串能够匹配正则表达式 regex 时为真。比如:
IF("hello" MATCHES "ell")
MESSAGE("true")
ENDIF("hello" MATCHES "ell")
```
数字比较表达式
```cmake
IF(variable LESS number)
IF(string LESS number)
IF(variable GREATER number)
IF(string GREATER number)
IF(variable EQUAL number)
IF(string EQUAL number)
```
按照字母序的排列进行比较
```cmake
IF(variable STRLESS string)
IF(string STRLESS string)
IF(variable STRGREATER string)
IF(string STRGREATER string)
IF(variable STREQUAL string)
IF(string STREQUAL string)
```
一个小例子,用来判断平台差异:
```cmake
IF(WIN32)
MESSAGE(STATUS “This is windows.”)
#作一些 Windows 相关的操作
ELSE(WIN32)
MESSAGE(STATUS “This is not windows”)
#作一些非 Windows 相关的操作
ENDIF(WIN32)

### 上述代码用来控制在不同的平台进行不同的控制,但是,阅读起来却并不是那么舒服,
## ELSE(WIN32)之类的语句很容易引起歧义。
## 这就用到了我们在“常用变量”一节提到的 CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS 开关:
SET(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS ON)
# 这时候就可以写成:
IF(WIN32)
ELSE()
ENDIF()

### 如果配合 ELSEIF 使用,可能的写法是这样:
IF(WIN32)
#do something related to WIN32
ELSEIF(UNIX)
#do something related to UNIX
ELSEIF(APPLE)
#do something related to APPLE
ENDIF(WIN32)
```
### 2. WHILE指令

WHILE 指令的语法是:

```cmake
WHILE(condition)
    COMMAND1(ARGS ...)
    COMMAND2(ARGS ...)
    ...
ENDWHILE(condition)
```

其真假判断条件可以参考 IF 指令。

### 3. FOREACH指令

FOREACH 指令的使用方法有三种形式:
#### (1)列表
```cmake
FOREACH(loop_var arg1 arg2 ...)
    COMMAND1(ARGS ...)
    COMMAND2(ARGS ...)
    ...
ENDFOREACH(loop_var)
```
像我们前面使用的 AUX_SOURCE_DIRECTORY 的例子
```cmake
AUX_SOURCE_DIRECTORY(. SRC_LIST)
FOREACH(F ${SRC_LIST})
    MESSAGE(${F})
ENDFOREACH(F)
```
#### (2)范围
```cmake
FOREACH(loop_var RANGE total)
	...
ENDFOREACH(loop_var)
```
从 0 到 total 以1为步进
举例如下:
```Cmake
FOREACH(VAR RANGE 3)
	MESSAGE(${VAR})
ENDFOREACH(VAR)
```
最终得到的输出是:
0
1
2
3

#### (3)范围和步进
```
FOREACH(loop_var RANGE start stop [step])
	...
ENDFOREACH(loop_var)
```
从 start 开始到 stop 结束,以 step 为步进,
举例如下:
```cmake
FOREACH(A RANGE 5 15 3)
	MESSAGE(${A})
ENDFOREACH(A)
```
最终得到的结果是:
5
8
11
14

这个指令需要注意的是,直到遇到 ENDFOREACH 指令,整个语句块才会得到真正的执行。

[回到目录](#目录)