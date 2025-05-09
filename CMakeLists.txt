cmake_minimum_required(VERSION 3.8)
project(kpos_node)

# 필수 패키지
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# include 경로에 include 폴더 추가
include_directories(
  include
)

# 라이브러리 경로 지정
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

# 노드 실행 파일 추가
add_executable(kpos_node src/main.cpp)

# 라이브러리 링크 (이름에서 lib 접두사 및 .so 확장자 제외)
target_link_libraries(kpos_node
  KPOS
  Eigen3::Eigen

)

# 의존성 연결
ament_target_dependencies(kpos_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
)

# 설치
install(TARGETS
  kpos_node
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  lib/libKPOS.so
  DESTINATION lib
)

install(DIRECTORY
  include/
  DESTINATION include
)

ament_package()
