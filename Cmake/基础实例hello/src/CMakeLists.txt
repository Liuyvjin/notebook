ADD_EXECUTABLE(hello hello.c) # 编译hello.c生成hello
INSTALL(TARGETS hello RUNTIME DESTINATION bin) # hello的安装目录
