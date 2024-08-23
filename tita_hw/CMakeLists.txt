cmake_minimum_required(VERSION 3.10)
project(tita_hw)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#add_definitions(-Wall -Werror -Wno-inconsistent-missing-override)
set(CATKIN_PACKAGE_DEPENDENCIES
        roscpp
        hardware_interface
        imu_complementary_filter
        imu_filter_madgwick
)

find_package(Boost REQUIRED COMPONENTS
        system
        filesystem
)
find_package(catkin REQUIRED COMPONENTS
        ${CATKIN_PACKAGE_DEPENDENCIES}
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
        ${CATKIN_PACKAGE_DEPENDENCIES}
        DEPENDS
        Boost
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE sources "src/*.cpp")

add_library(${PROJECT_NAME} SHARED ${sources})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
