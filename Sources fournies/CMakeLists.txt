cmake_minimum_required(VERSION 3.4)
project(TP2)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}) #for the executable
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}) #for static library
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}) #for shared library

add_library(TP2 STATIC ReseauGTFS.cpp)

add_library(GRAPHE STATIC graphe.cpp)

link_directories(${PROJECT_SOURCE_DIR})

add_executable(main main.cpp)
target_link_libraries(main TP2 GRAPHE TP1)

#add_subdirectory(Tests)