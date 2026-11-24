cmake_minimum_required(VERSION 3.18)
project(arm_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch rviz meshes_collision
  DESTINATION share/${PROJECT_NAME}
)

# Meshes (docs/decisions/0026): the full ones, only drawn, stay in the Fusion export zip in the repo (114 MB, too big
# for git one by one) and are extracted here at configure time, so a fresh clone builds without a manual step; a
# new export (a new zip) re-runs the extraction. Collisions use the light meshes_collision/ (scripts/simplify_meshes.py).
set(FUSION_ZIP "${CMAKE_CURRENT_SOURCE_DIR}/../../../cad_export/fusion_urdf/URDF.zip")
if(NOT EXISTS "${FUSION_ZIP}")
  message(FATAL_ERROR "Fusion export not found: ${FUSION_ZIP}")
endif()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${FUSION_ZIP}")
file(REMOVE_RECURSE "${CMAKE_CURRENT_BINARY_DIR}/fusion")
file(ARCHIVE_EXTRACT INPUT "${FUSION_ZIP}" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/fusion" PATTERNS "URDF/meshes/*")
install(
  DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/fusion/URDF/meshes/"
  DESTINATION share/${PROJECT_NAME}/meshes
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_urdf test/test_urdf.py)
endif()

ament_package()
