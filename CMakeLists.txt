cmake_minimum_required(VERSION 2.8.3)
project(staticoma)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rosbag_storage
    std_msgs
)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS include
#    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp std_msgs
#    DEPENDS system_lib
    CFG_EXTRAS ${PROJECT_NAME}_cmake_path.cmake
)


###########
## Build ##
###########
include(CMakeParseArguments)


# --------------
# CCW flags
# --------------
if (CCW_CLANG_TIDY)
    set (CMAKE_CXX_CLANG_TIDY "${CCW_CLANG_TIDY}")
endif()
if (CCW_CXX_FLAGS)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CCW_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${CCW_LINKER_FLAGS}")
else()
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra -Werror=pedantic -pedantic-errors -Wshadow -Werror=return-type -Werror -fPIC -fstack-protector-strong")
endif()
# --------------


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    SYSTEM ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
#add_library(${PROJECT_NAME}
#    src/${PROJECT_NAME}/${PROJECT_NAME}.cpp
#)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(server src/server.cpp)
add_executable(replayer src/replayer.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
#set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(server
    ${catkin_LIBRARIES}
)
target_link_libraries(replayer
    ${catkin_LIBRARIES}
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
#install(PROGRAMS
#    scripts/my_python_script
#    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

# Mark executables and/or libraries for installation
install(TARGETS replayer server
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
#install(FILES
#    # myfile1
#    # myfile2
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
#foreach(DIR launch)
#    install(DIRECTORY ${DIR}
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#        USE_SOURCE_PERMISSIONS
#    )
#endforeach()
#install(FILES ${CMAKE_SOURCE_DIR}/plugin.xml
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
install (DIRECTORY "${PROJECT_SOURCE_DIR}/cmake/"
         DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake"
         FILES_MATCHING PATTERN "*.cmake")


#==============================================
# Uncomment to enable tests.
#==============================================
if(CATKIN_ENABLE_TESTING)
    add_subdirectory(test)
endif()
#==============================================
