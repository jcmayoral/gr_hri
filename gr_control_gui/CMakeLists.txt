cmake_minimum_required(VERSION 2.8.3)
project(gr_control_gui)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS rviz roscpp visualization_msgs
                    geometry_msgs mongodb_store navigation_msgs
                    tf2_geometry_msgs std_msgs gr_map_utils nav_msgs
                    move_base_msgs  warehouse_ros_mongo 
                    gr_action_msgs actionlib)


#add_definitions(-Wno-deprecated-declarations)

catkin_package(
    CATKIN_DEPENDS
      rviz
      roscpp
      visualization_msgs
      geometry_msgs
      mongodb_store
      navigation_msgs
      tf2_geometry_msgs
      std_msgs
      gr_map_utils
      gr_action_msgs
      nav_msgs
      actionlib
      move_base_msgs
      warehouse_ros_mongo
      gr_action_msgs
)

link_directories(${catkin_LIBRARY_DIRS})

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)

include_directories(${catkin_INCLUDE_DIRS})

add_library(offline_gui offline.cpp)
qt5_use_modules(offline_gui Widgets)
qt5_use_modules(offline_gui Core)
target_link_libraries(offline_gui ${catkin_LIBRARIES} ${Qt5Widgets_LIBRARIES} )

add_library(online_gui online.cpp)
qt5_use_modules(online_gui Widgets)
qt5_use_modules(online_gui Core)
target_link_libraries(online_gui ${catkin_LIBRARIES} ${Qt5Widgets_LIBRARIES})
add_compile_options(-std=c++11)

add_library(common_gui common.cpp)
qt5_use_modules(common_gui Widgets)
qt5_use_modules(common_gui Core)
target_link_libraries(common_gui ${catkin_LIBRARIES} ${Qt5Widgets_LIBRARIES})

add_library(app_online app_online.cpp)
qt5_use_modules(app_online Widgets)
qt5_use_modules(app_online Core)
target_link_libraries(app_online ${catkin_LIBRARIES} ${Qt5Widgets_LIBRARIES})

add_executable(online main_online.cpp)
target_link_libraries(online online_gui common_gui )

add_executable(offline main_offline.cpp)
target_link_libraries(offline offline_gui common_gui)

add_executable(common main_common.cpp)
target_link_libraries(common common_gui)

add_executable(app_online_node app_online_node.cpp)
target_link_libraries(app_online_node app_online common_gui)

# Instalv
install(TARGETS online offline common DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
