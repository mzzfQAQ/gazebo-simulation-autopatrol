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

# 5. 创建第三个插件库 (RRT Dynamic Biased Planner)
add_library(nav2_rrt_dynamic_biased_planner_plugin SHARED src/nav2_rrt_dynamic_biased_planner.cpp)
ament_target_dependencies(nav2_rrt_dynamic_biased_planner_plugin nav2_core pluginlib rclcpp)

# 6. 创建第四个插件库 (RRT APF Guided Planner)
add_library(nav2_rrt_apf_guided_planner_plugin SHARED src/nav2_rrt_apf_guided_planner.cpp)
ament_target_dependencies(nav2_rrt_apf_guided_planner_plugin nav2_core pluginlib rclcpp)

# 7. 创建第五个插件库 (RRT Adaptive Step Planner)
add_library(nav2_rrt_adaptive_step_planner_plugin SHARED src/nav2_rrt_adaptive_step_planner.cpp)
ament_target_dependencies(nav2_rrt_adaptive_step_planner_plugin nav2_core pluginlib rclcpp)

# 8. 创建第六个插件库 (RRT Pruning Planner)
add_library(nav2_rrt_pruning_planner_plugin SHARED src/nav2_rrt_pruning_planner.cpp)
ament_target_dependencies(nav2_rrt_pruning_planner_plugin nav2_core pluginlib rclcpp) 

# 9. 创建第七个插件库 (RRT BSpline Smooth Planner)
add_library(nav2_rrt_bspline_smooth_planner_plugin SHARED src/nav2_rrt_bspline_smooth_planner.cpp)
ament_target_dependencies(nav2_rrt_bspline_smooth_planner_plugin nav2_core pluginlib rclcpp)

# 10. 创建第八个插件库 (RRT Connect Planner)
add_library(nav2_rrt_connect_planner_plugin SHARED src/nav2_rrt_connect_planner.cpp)
ament_target_dependencies(nav2_rrt_connect_planner_plugin nav2_core pluginlib rclcpp)

# 11. 创建第九个插件库 (RRT Connect Smooth Planner)
add_library(nav2_rrt_connect_smooth_planner_plugin SHARED src/nav2_rrt_connect_smooth_planner.cpp)
ament_target_dependencies(nav2_rrt_connect_smooth_planner_plugin nav2_core pluginlib rclcpp)

# 12. 创建第十个插件库 (RRT Connect Auto Pruning Planner)
add_library(nav2_rrt_connect_auto_pruning_planner_plugin SHARED src/nav2_rrt_connect_auto_pruning_planner.cpp)
ament_target_dependencies(nav2_rrt_connect_auto_pruning_planner_plugin nav2_core pluginlib rclcpp)

# 13. 安装所有库文件
install(TARGETS 
  nav2_custom_planner_plugin
  nav2_rrt_origin_planner_plugin
  nav2_rrt_dynamic_biased_planner_plugin
  nav2_rrt_apf_guided_planner_plugin
  nav2_rrt_bspline_smooth_planner_plugin
  nav2_rrt_adaptive_step_planner_plugin
  nav2_rrt_pruning_planner_plugin
  nav2_rrt_connect_planner_plugin
  nav2_rrt_connect_smooth_planner_plugin
  nav2_rrt_connect_auto_pruning_planner_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 13. 安装头文件
install(DIRECTORY include/ DESTINATION include/)

# 14. 导出各自的插件描述文件 (关键点)
# 这里的第一个参数必须是插件对应的基类包名 (nav2_core)
# 第二个参数是你的 XML 文件路径
pluginlib_export_plugin_description_file(nav2_core custom_planner_plugin.xml)
pluginlib_export_plugin_description_file(nav2_core rrt_origin_planner_plugin.xml)
pluginlib_export_plugin_description_file(nav2_core rrt_dynamic_biased_planner_plugin.xml)
pluginlib_export_plugin_description_file(nav2_core rrt_apf_guided_planner_plugin.xml)
pluginlib_export_plugin_description_file(nav2_core rrt_bspline_smooth_planner_plugin.xml)
pluginlib_export_plugin_description_file(nav2_core rrt_adaptive_step_planner_plugin.xml)
pluginlib_export_plugin_description_file(nav2_core rrt_pruning_planner_plugin.xml)
pluginlib_export_plugin_description_file(nav2_core rrt_connect_planner_plugin.xml)
pluginlib_export_plugin_description_file(nav2_core rrt_connect_smooth_planner_plugin.xml)
pluginlib_export_plugin_description_file(nav2_core rrt_connect_auto_pruning_planner_plugin.xml)

ament_package()