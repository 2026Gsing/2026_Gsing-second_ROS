cmake_minimum_required(VERSION 3.8)
project(description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# 安装 urdf 文件 - 从 car_test 目录安装
install(DIRECTORY
  src/car_test/urdf  # 改为从 car_test 安装
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

# 安装其他模型文件
install(DIRECTORY
  src/camera
  src/food
  src/instrument
  src/medicine
  src/tool
  DESTINATION share/${PROJECT_NAME}/src
  USE_SOURCE_PERMISSIONS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()