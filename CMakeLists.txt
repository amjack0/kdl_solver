cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_BUILD_TYPE Debug)
set (CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(kdl_solver)

find_package(kdl_parser REQUIRED)

find_package(orocos_kdl REQUIRED)
IF (orocos_kdl_FOUND)
    message(STATUS "[SOLVER] orocos_kdl found. Version: ${orocos_kdl_VERSION}")
    include_directories(BEFORE SYSTEM ${orocos_kdl_INCLUDE_DIRS})
    add_definitions(${orocos_kdl_CFLAGS})
    link_directories(${orocos_kdl_LIBRARY_DIRS})
ENDIF()

#find_package(Boost REQUIRED COMPONENTS system)

#find_package(Eigen3 REQUIRED)
#IF (Eigen3_FOUND)
#    message(STATUS "[SOLVER] Eigen3 found. Version: ${Eigen3_VERSION}")
#    include_directories(BEFORE SYSTEM ${Eigen3_INCLUDE_DIRS})
#    add_definitions(${Eigen3_CFLAGS})
#    link_directories(${Eigen3_LIBRARY_DIRS})
#ENDIF()

add_executable(${PROJECT_NAME} src/solver.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
   ${orocos_kdl_LIBRARIES}
   #Eigen3::Eigen
   ${kdl_parser_LIBRARIES})
