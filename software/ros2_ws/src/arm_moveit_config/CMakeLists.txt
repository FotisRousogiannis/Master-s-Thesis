cmake_minimum_required(VERSION 3.8)
project(arm_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_config test/test_config.py)
endif()

ament_package()
