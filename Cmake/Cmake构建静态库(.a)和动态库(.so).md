# Cmake构建静态库(.a)和动态库(.so)

#### 目录

[TOC]

## 一. C/C++静态链接库和动态链接库介绍

### 1. 概念简介



### 1. 添加共享库

```cmake
ADD_LIBRARY(libname [SHARED|STATIC|MODULE][EXCLUDE_FROM_ALL]source1 source2 ... sourceN))
```

第一个参数libname是要生成的共享库的名称，不需要填写全名，如果填hello，则生成的文件名为libname.so。

第二个参数是类型，有三种：

+ SHARED，动态库(扩展名为.so)
+ STATIC，静态库(扩展名为.a)

第三个参数意思是这个库不会被默认构建，除非有其他组件依赖或者手工构建。