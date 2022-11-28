cmake_minimum_required(VERSION 3.10.2)
project(ros_nvml_diagnostics)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

find_package(Nvml REQUIRED)
find_package(catkin REQUIRED COMPONENTS diagnostic_updater roscpp)

catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS roscpp)

# ##############################################################################
# Build ##
# ##############################################################################

add_executable(
  ${PROJECT_NAME}_node src/device.cc src/general_information.cc src/main.cc
                       src/utilities.cc src/device_diagnostic_publisher.cc)

target_include_directories(${PROJECT_NAME}_node PRIVATE include)

target_include_directories(${PROJECT_NAME}_node SYSTEM
                           PRIVATE ${catkin_INCLUDE_DIRS} ${NVML_INCLUDE_DIRS})

target_compile_options(${PROJECT_NAME}_node PRIVATE -Wall -Wextra -pedantic
                                                    -Werror)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES}
                      ${NVML_LIBRARIES})

# ##############################################################################
# Install ##
# ##############################################################################
install(TARGETS ${PROJECT_NAME}_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
