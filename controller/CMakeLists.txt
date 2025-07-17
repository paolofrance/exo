cmake_minimum_required(VERSION 3.10)
project(high-level)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions(-g)

find_package(Boost REQUIRED thread system)

add_compile_definitions(BOOST_LOG_DYN_LINK=1)

include_directories(
    ${Boost_INCLUDE_DIRS} 
    include
) 

add_executable(high_level
  src/ref_generator.cpp
)
target_link_libraries(high_level 
  ${Boost_LIBRARIES} 
  rt 
  pthread
)

add_executable(test
  src/test.cpp
)
target_link_libraries(test 
  ${Boost_LIBRARIES} 
  rt 
  pthread
)

