cmake_minimum_required(VERSION 3.0.2)
project(VFH)


# находим необходимые пакеты для сборки
# компоненты ROS
find_package(catkin REQUIRED COMPONENTS
sensor_msgs
tf2
tf2_ros
tf2_msgs
tf2_sensor_msgs
geometry_msgs 
sensor_msgs
roscpp
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



# добавляем исполняемые файлы
# нода для запуска навигационной системы
add_executable(flight_with_avoidance src/FlightWithAvoidanceNode.cpp src/FlightCommander.cpp src/Histogram.cpp src/Avoidance.cpp)
# пример с полетом БЛА(взаимодействие с аппаратом, управление целевым положением)

# libraris link for visual navigation system
target_link_libraries(flight_with_avoidance  ${catkin_LIBRARIES}  )