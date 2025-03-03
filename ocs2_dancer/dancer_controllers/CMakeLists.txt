cmake_minimum_required(VERSION 3.0.2)
project(dancer_controllers)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(SIM "Use raisim backend as physics" ON)

if (DEFINED SIM)
    message(STATUS "Using raisim: " ${SIM})
    add_definitions(-DUSE_SIM=\"${SIM}\")
else ()
    message(STATUS "Using Real robot backend by default.")
endif ()

set(CATKIN_PACKAGE_DEPENDENCIES
  rospack
  dancer_interface
  dancer_interface_ros
  dancer_raisim
  #dancer_hw_ros
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  LIBRARIES
    ${PROJECT_NAME}
)

###########
## Build ##
###########

# Add directories for all targets
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# dancer mpc mrt
add_executable(dancer_mpc_mrt
  src/dancerMpcMrtNode.cpp
)

add_dependencies(dancer_mpc_mrt
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(dancer_mpc_mrt
  ${catkin_LIBRARIES}
)
target_compile_options(dancer_mpc_mrt PRIVATE ${OCS2_CXX_FLAGS})

# # dancer nmpc controller
# add_executable(dancer_nmpc_controller 
#   src/dancerNmpcNode.cpp
#   src/NmpcController.cpp
# )
# add_dependencies(dancer_nmpc_controller
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
# )
# target_link_libraries(dancer_nmpc_controller -lstdc++ -lpthread -lm -lrt -ldl)
# target_link_libraries(dancer_nmpc_controller
#   ${catkin_LIBRARIES}
#   yaml-cpp
# )
# target_compile_options(dancer_nmpc_controller PRIVATE ${OCS2_CXX_FLAGS})
# add_dependencies(dancer_nmpc_controller merge_compile_commands)

# # dancer mrt controller
# add_executable(dancer_hw_ros_node
#   src/dancerMrtNode.cpp
# )
# add_dependencies(dancer_hw_ros_node
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
# )
# target_link_libraries(dancer_hw_ros_node -lstdc++ -lpthread -lm -lrt -ldl)
# target_link_libraries(dancer_hw_ros_node
#   ${catkin_LIBRARIES}
#   yaml-cpp
# )
# target_compile_options(dancer_hw_ros_node PRIVATE ${OCS2_CXX_FLAGS})
# add_dependencies(dancer_hw_ros_node merge_compile_commands)

set(CMD "jq -s 'add' \$\(find ${CMAKE_BINARY_DIR}/.. -name compile_commands.json\) > ${CMAKE_BINARY_DIR}/../compile_commands.json")
add_custom_target(
    merge_compile_commands ALL
    COMMAND /bin/bash -c "${CMD}"
    VERBATIM 
    COMMENT "Merging compile_commands.json files"
)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS 
  dancer_mpc_mrt 
  #dancer_nmpc_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

