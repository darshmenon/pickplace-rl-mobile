cmake_minimum_required(VERSION 3.5)
project(robotiq_2f_85_gripper_visualization)

find_package(ament_cmake REQUIRED)

# Install the 'meshes' and 'urdf' directories to the share folder
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
