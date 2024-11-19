cmake_minimum_required(VERSION 3.8)
project(abv_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

set(SIMULATOR_SOURCES 
  ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/UdpServer.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/VehicleSimulator.cpp
)

add_executable(${PROJECT_NAME} ${SIMULATOR_SOURCES})

include_directories(${PROJECT_NAME} 
  ${CMAKE_CURRENT_LIST_DIR}/include
)

# Install the executable
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
