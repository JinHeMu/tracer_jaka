cmake_minimum_required(VERSION 3.8)
project(jaka_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =============================================
# 1. 寻找依赖包
# =============================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)   # 之前的代码用到了 JointState
find_package(geometry_msgs REQUIRED) # 之前的代码用到了 PoseStamped
find_package(tf2 REQUIRED)           # 之前的代码用到了 Quaternion

# =============================================
# 2. 设置头文件路径
# =============================================
# 使得代码中可以直接 #include "JAKAZuRobot.h"
# 假设你的目录结构是: source_dir/include/jaka_driver/JAKAZuRobot.h
include_directories(
  include
  include/jaka_driver 
)

# =============================================
# 3. 定义可执行文件
# =============================================
add_executable(jaka_edg_node src/jaka_edg_node.cpp)
add_executable(jaka_login src/jaka_login.cpp)
add_executable(jaka_logout src/jaka_logout.cpp)
add_executable(jaka_tcp_sensor src/jaka_tcp_sensor.cpp)
# =============================================
# 4. 链接依赖
# =============================================
# 4.1 链接 ROS 2 的标准库
ament_target_dependencies(jaka_edg_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
)


# 4.2 链接 JAKA 的第三方库 (libjakaAPI.so)
# 假设库文件位于: source_dir/lib/libjakaAPI.so
# 我们使用 CMAKE_CURRENT_SOURCE_DIR 获取绝对路径
target_link_libraries(jaka_edg_node
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libjakaAPI.so
)

target_link_libraries(jaka_login
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libjakaAPI.so
)

target_link_libraries(jaka_logout
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libjakaAPI.so
)

target_link_libraries(jaka_tcp_sensor
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libjakaAPI.so
)
# =============================================
# 5. 安装规则 (Install)
# =============================================
# 安装可执行文件到 lib/jaka_driver/ 下
install(TARGETS 
  jaka_edg_node
  jaka_login
  jaka_logout
  jaka_tcp_sensor
  DESTINATION lib/${PROJECT_NAME}
)

# 可选：将头文件也安装过去（如果其他包需要引用这个包）
install(DIRECTORY include/
  DESTINATION include
)

# 可选：将 libjakaAPI.so 安装到 ROS 的 lib 目录
# 这样运行时就不容易报 "cannot open shared object file"
install(FILES lib/libjakaAPI.so
  DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# 操作总结

# 当你下次想添加一个名为 my_new_node 的节点时，只需要做三步：

# 写代码：在 src/ 下创建 my_new_node.cpp。

# 改 CMake：

# 添加 add_executable(my_new_node src/my_new_node.cpp)

# 添加 ament_target_dependencies(my_new_node ...)

# 添加 target_link_libraries(my_new_node ${JAKA_SDK_LIB})

# 在底部的 install(TARGETS ...) 列表里加上 my_new_node。

# 编译：colcon build。