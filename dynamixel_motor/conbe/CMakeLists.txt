cmake_minimum_required(VERSION 2.8.3)
project(conbe)
find_package(catkin REQUIRED)

catkin_package(
    CATKIN_DEPENDS dynamixel_controllers
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
