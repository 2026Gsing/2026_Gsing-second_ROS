cmake_minimum_required(VERSION 3.8)
project(cv)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- 1. 查找 ROS 依赖 ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED) 
# (cv_bridge 会自动帮我们找到并链接 OpenCV::core 和 OpenCV::imgproc)

# --- 2. 查找我们【额外】需要的 OpenCV 组件 ---
# 我们只查找 highgui
find_package(OpenCV REQUIRED COMPONENTS
  highgui 
)
# (这会为我们填充 ${OpenCV_INCLUDE_DIRS} 和 ${OpenCV_LIBRARIES} 变量)

# --- 3. 测试依赖 (保持原样) ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# --- 4. 添加可执行文件 ---
add_executable(cv_node src/cv_node.cpp)

# --- 5. 链接依赖 (最终修正) ---

# 步骤 5a: 链接 ROS (Ament) 依赖
# (这会自动链接 rclcpp, sensor_msgs, cv_bridge, 
#  以及 cv_bridge 依赖的 OpenCV::core 和 OpenCV::imgproc)
ament_target_dependencies(cv_node
  rclcpp
  sensor_msgs
  cv_bridge
)

# 步骤 5b: 链接我们【额外】需要的外部依赖
# 【已修正】: 我们链接 highgui 导出的【变量】，而不是【目标】
# 这是为了避免与 cv_bridge 导出的目标冲突
target_include_directories(cv_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(cv_node
  ${OpenCV_LIBRARIES}
)

# --- 6. 安装和打包 ---
install(TARGETS
  cv_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()