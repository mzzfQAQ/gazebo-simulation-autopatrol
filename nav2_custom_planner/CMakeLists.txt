cmake_minimum_required(VERSION 3.8)
project(nav2_custom_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 查找依赖
find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED) # 插件通常需要 rclcpp

# ... 保持 BUILD_TESTING 部分不变 ...

# 2. 包含头文件目录
include_directories(include)

# 3. 创建第一个插件库 (Custom Planner)
add_library(nav2_custom_planner_plugin SHARED src/nav2_custom_planner.cpp)
ament_target_dependencies(nav2_custom_planner_plugin nav2_core pluginlib rclcpp)

# 4. 创建第二个插件库 (RRT Origin Planner)
add_library(nav2_rrt_origin_planner_plugin SHARED src/nav2_rrt_origin_planner.cpp)
ament_target_dependencies(nav2_rrt_origin_planner_plugin nav2_core pluginlib rclcpp)

# 5. 安装所有库文件
install(TARGETS 
  nav2_custom_planner_plugin
  nav2_rrt_origin_planner_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 6. 安装头文件
install(DIRECTORY include/ DESTINATION include/)

# 7. 导出各自的插件描述文件 (关键点)
# 这里的第一个参数必须是插件对应的基类包名 (nav2_core)
# 第二个参数是你的 XML 文件路径
pluginlib_export_plugin_description_file(nav2_core custom_planner_plugin.xml)
pluginlib_export_plugin_description_file(nav2_core rrt_origin_planner_plugin.xml)

ament_package()