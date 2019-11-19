cmake_minimum_required(VERSION 3.5)

project(LynxmotionLSS)

include(CMakeChecks.txt)

add_subdirectory(lib-lss)
add_subdirectory(examples)
#add_subdirectory(tests)
