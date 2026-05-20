cmake_minimum_required(VERSION 3.8)
project(simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 安装 launch 文件
install(DIRECTORY
  src/launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装 worlds 文件
install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# 安装 config 文件
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# 安装 Python 脚本 - 只安装存在的文件
install(PROGRAMS
  src/scripts/path_planner.py
  DESTINATION lib/${PROJECT_NAME}
)

# 禁用测试
set(BUILD_TESTING OFF)

ament_package()