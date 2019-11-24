cmake_minimum_required(VERSION 3.5)

project(LynxmotionLSS)

include(CMakeChecks.txt)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_subdirectory(lib-lss)
add_subdirectory(examples)
#add_subdirectory(tests)
