CMAKE_MINIMUm_REQUIRED(VERSION 2.6) # 最低版本说明
PROJECT(HELLO)	# 项目名称
SET(CMAKE_INSTALL_PREFIX  /tmp/myhello)  # 设置项目前缀
ADD_SUBDIRECTORY(src bin)	# 添加子目录src，输出目录bin
INSTALL(FILES COPYRIGHT README DESTINATION doc) # 安装文本
INSTALL(PROGRAMS runhello.sh DESTINATION bin) # 安装脚本，注意这个和前面的bin不同,是/tmp/myhello/bin
INSTALL(DIRECTORY doc/ DESTINATION doc)  # 安装目录
