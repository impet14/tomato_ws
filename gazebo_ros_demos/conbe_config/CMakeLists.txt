cmake_minimum_required(VERSION 2.8.3)
project(conbe_config)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#for moveit_python_interface
install(PROGRAMS
  scripts/move_group_python_interface_tutorial.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

