cmake_minimum_required(VERSION 3.0.2)
project(uav_controller)


# находим необходимые пакеты для сборки
# компоненты ROS
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  mavros_msgs
)

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES wheelPlatform 
  CATKIN_DEPENDS roscpp 

)

# указываем в проекте все дериктории с заголовочными файлами
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


# добавляем библиотеки для сборки
# библиотека систем упра  вления
add_library(control_sys_lib STATIC src/uav_controller.cpp)

# добавляем исполняемые файлы
# нода для запуска навигационной системы
add_executable(testing src/uav_controller_node.cpp src/uav_controller.cpp)
add_executable(circle src/uav_circle_fly_example.cpp)
# пример с полетом БЛА(взаимодействие с аппаратом, управление целевым положением)

# libraris link for visual navigation system
target_link_libraries(testing  ${catkin_LIBRARIES}  )
target_link_libraries(circle  ${catkin_LIBRARIES}  )