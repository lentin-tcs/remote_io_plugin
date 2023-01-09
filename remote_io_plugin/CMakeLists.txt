cmake_minimum_required(VERSION 3.5)
project(remote_io_plugin)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(rqt_gui REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)


set(remote_io_plugin_SRCS
  src/remote_io_plugin/my_plugin.cpp)

set(remote_io_plugin_HDRS
  include/remote_io_plugin/my_plugin.h)

set(remote_io_plugin_UIS
  src/remote_io_plugin/my_plugin.ui)

set(remote_io_plugin_INCLUDE_DIRS
  include
  ${rqt_gui_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${control_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${rqt_gui_cpp_INCLUDE_DIRS}
  )

qt5_wrap_cpp(remote_io_plugin_MOCS ${remote_io_plugin_HDRS})

qt5_wrap_ui(remote_io_plugin_UIS_H ${remote_io_plugin_UIS})

include_directories(${remote_io_plugin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED
  ${remote_io_plugin_SRCS}
  ${remote_io_plugin_MOCS}
  ${remote_io_plugin_UIS_H})


target_link_libraries(
  ${PROJECT_NAME}
  ${rqt_gui_LIBRARIES}
  ${rqt_gui_cpp_LIBRARIES}
  Qt5::Widgets
  ${QT_LIBRARIES}
  ${QT_QTCORE_LIBRARY}
  ${QT_QTGUI_LIBRARY}
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${control_msgs_LIBRARIES}

)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

install(PROGRAMS scripts/remote_io_plugin
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(rqt_gui "plugin.xml")
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
