cmake_minimum_required(VERSION 3.10)
project(odrive_master)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../lib/)

find_package(Boost REQUIRED COMPONENTS thread chrono system)
find_package(Curses REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

find_package(Doxygen)
if(DOXYGEN_FOUND)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
    add_custom_target(doc
        ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen" VERBATIM
    )
endif()

add_compile_definitions(BOOST_LOG_DYN_LINK=1)

include_directories(
    ${Boost_INCLUDE_DIRS} 
    ./include/
    ${CURSES_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
) 

# Defines every source file in the folder 'src'
file(GLOB SRC_FILES src/*.cpp src/*.c)

add_library(odrivemaster STATIC
  ${SRC_FILES}
)

add_executable(${PROJECT_NAME}
  main.cpp
  control_loop.cpp
  ${SRC_FILES}
)
target_link_libraries(${PROJECT_NAME} 
  ${Boost_LIBRARIES} 
  ${CURSES_LIBRARIES} 
  rt
)

add_dependencies(doc ${PROJECT_NAME})
