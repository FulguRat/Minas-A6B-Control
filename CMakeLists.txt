cmake_minimum_required(VERSION 3.0)

# Project Name
set(PROJ_NAME EtherCAT_Test)
project(${PROJ_NAME})

# Binary output path
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

FILE(GLOB HEADER_LIST "${CMAKE_SOURCE_DIR}/src/*.h")
FILE(GLOB SOURCE_LIST "${CMAKE_SOURCE_DIR}/src/*.cpp")

add_executable(${PROJ_NAME} ${SOURCE_LIST} ${HEADER_LIST})

#Boost
find_package(Boost COMPONENTS thread REQUIRED)
if (Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
    target_link_libraries(${PROJ_NAME} ${Boost_LIBRARIES})
    message("Boost Ready.")
endif()

# SOEM
set(SOEM_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/include")
set(SOEM_LIB "${CMAKE_SOURCE_DIR}/lib/libsoem.a")

set(OS_LIBS pthread rt)

include_directories(${SOEM_INCLUDE_DIR})

target_link_libraries(${PROJ_NAME} ${SOEM_LIB} ${OS_LIBS})
message("SOEM Ready.")
