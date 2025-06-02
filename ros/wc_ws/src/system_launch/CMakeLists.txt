cmake_minimum_required(VERSION 3.5)
project(system_launch)

find_package(ament_cmake REQUIRED)

# Install the entire launch/ directory under share/system_launch/launch
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Also install package.xml so ament_index picks up this package
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

