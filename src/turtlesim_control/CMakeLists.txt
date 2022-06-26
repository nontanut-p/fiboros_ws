cmake_minimum_required(VERSION 3.5)
project(turtlesim_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

################ BUILD DEPENDENCIES  #################
find_package(ament_cmake_python REQUIRED)   # python (optional)
find_package(rclpy REQUIRED)                # python (optional)
######################################################

# Install Python Module
ament_python_install_package(python_lib)

# Install Python executables
install(PROGRAMS
  scripts/controller.py
  scripts/scheduler.py
  DESTINATION lib/${PROJECT_NAME}
)

################ INSTALL LAUNCH, ETC #################
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})


ament_package()
