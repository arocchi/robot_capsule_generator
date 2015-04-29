# Copyright: (C) 2014 Walkman Consortium
# Authors: Alessio Rocchi
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8.12)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 OLD)
    cmake_policy(SET CMP0005 NEW)
    cmake_policy(SET CMP0017 NEW)
endif(COMMAND cmake_policy)
include(ExternalProject)
find_package(PkgConfig)
PROJECT(robot_capsule_generator)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=return-type -Werror=address -Werror=parentheses " )
endif(CMAKE_BUILD_TYPE STREQUAL "Debug")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")

FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(Boost COMPONENTS system filesystem program_options REQUIRED)
FIND_PACKAGE(urdf REQUIRED)
FIND_PACKAGE(srdfdom REQUIRED)
FIND_PACKAGE(moveit_core REQUIRED)
pkg_check_modules(roboptim_capsule roboptim-capsule)
if(NOT roboptim_capsule_FOUND)
    message( FATAL_ERROR "roboptim-capsule not found. Aborting")
endif()

# add include directories
INCLUDE_DIRECTORIES(include ${roboptim_capsule_INCLUDE_DIRS}
                            ${EIGEN3_INCLUDE_DIR}
                            ${Boost_INCLUDE_DIRS})
LINK_DIRECTORIES(${roboptim_capsule_LIBRARY_DIRS})

# Find package qpOASES or build it using ExternalProject
find_package(trimesh2 QUIET)
if(NOT trimesh2_FOUND)
    set(trimesh2_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external/trimesh2/")
    set(trimesh2_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/external/src/trimesh2-ext-build/")

    ExternalProject_Add(trimesh2-ext SOURCE_DIR "${trimesh2_SOURCE_DIR}"
                                    PREFIX "${CMAKE_CURRENT_BINARY_DIR}/external"
                                    CONFIGURE_COMMAND ""
                                    BUILD_COMMAND "make"
                                    INSTALL_COMMAND "")
    link_directories("${trimesh2_BINARY_DIR}/lib/")
    set(trimesh2_INCLUDE_DIRS "${trimesh2_SOURCE_DIR}/include")
    set(trimesh2_LIBRARIES trimesh gomp)
    add_custom_target(copy_trimesh2_to_build ALL 
                    ${CMAKE_COMMAND} -E copy_directory ${trimesh2_SOURCE_DIR} ${trimesh2_BINARY_DIR})
    add_dependencies(trimesh2-ext copy_trimesh2_to_build)
    message("copying folder ${trimesh2_SOURCE_DIR} in ${trimesh2_BINARY_DIR})")
endif()

include_directories("${trimesh2_INCLUDE_DIRS}")

# for every file in generator_INCLUDES CMake already sets the property HEADER_FILE_ONLY
file(GLOB_RECURSE generator_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/include" *.h)

ADD_EXECUTABLE(robot_capsule_generator 
                    src/robot_capsule_generator.cpp
                    ${generator_INCLUDES})

# we must wait for trimesh2-ext to be ready before building robot_capsule_generator
if(TARGET trimesh2-ext)
    add_dependencies(robot_capsule_generator trimesh2-ext)
endif()


# we now add the YARP and iCub libraries to our project.
TARGET_LINK_LIBRARIES(robot_capsule_generator  PUBLIC  ${roboptim_capsule_LIBRARIES}
                                                       ${Boost_LIBRARIES}
                                               PRIVATE ${trimesh2_LIBRARIES})



install(TARGETS robot_capsule_generator
   ARCHIVE DESTINATION lib
   LIBRARY DESTINATION lib
   RUNTIME DESTINATION bin
)


#add_subdirectory(doc)

#######################
# Add Testing target  #
#######################
#enable_testing()
#add_subdirectory(tests)
