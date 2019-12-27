cmake_minimum_required(VERSION 3.5)

project(LynxmotionLSS)

option(WITH_SHARED "enable building of the shared library" ON)
option(WITH_STATIC "enable building of the static library" ON)
option(STATIC_LINKAGE "link examples and tests to the static lib (default: shared linkage)" ON)
option(WITH_DOCS "enable doxygen for generation of code documentation" OFF)
option(SANITIZE "enable sanitize flags if available providing memory leak and other bad programmer errors" OFF)
option(EFENCE "enable Electric Fence debugging by linking to lfence library" OFF)
option(WITH_LIBFTDI "enable building of the libftdi user-space serial driver for LssChannel" ON)
option(WITH_COMPLIANCE "include Compliant Joint functionality" ON)


include(CMakeChecks.txt)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(WITH_LIBFTDI)
    find_library(LIBUSB usb)
    find_library(LIBFTDI ftdi)

    if(LIBFTDI AND LIBUSB)
        set(HAS_LIBFTDI ON)
        add_compile_definitions(HAS_LIBFTDI)
        message(STATUS "enabling the FTDI user-space LssChannel driver")
    else()
        message(STATUS "libftdi or libusb not found, disabling the FTDI user-space LssChannel driver")
    endif()
endif()

# find our debugger
get_filename_component(COMPILER_DIRECTORY "${CMAKE_CXX_COMPILER}" DIRECTORY)
find_program(GDB gdb PATHS ${COMPILER_DIRECTORY})
find_program(LLDB lldb PATHS ${COMPILER_DIRECTORY})
if(GDB)
    set(DEBUGGER ${GDB})
elseif(LLDB)
    set(DEBUGGER ${LLDB})
endif()

# try to enable valgrid (optional)
find_program(VALGRIND "valgrind")
if(VALGRIND)
    add_custom_target(memcheck)

    find_path(HAVE_VALGRIND_MEMCHECK_H memcheck.h
            /usr/local/include
            /usr/local/include/valgrind
            /usr/include
            /usr/include/valgrind
            )
    if(HAVE_VALGRIND_MEMCHECK_H)
        add_definitions(-DVALGRIND="${VALGRIND}" -DHAVE_VALGRIND_MEMCHECK_H)
        include_directories(${HAVE_VALGRIND_MEMCHECK_H})
        message(STATUS "Valgrind features enabled, found at ${VALGRIND} and ${HAVE_VALGRIND_MEMCHECK_H}/memcheck.h")
    else()
        message(STATUS "Valgrind features enabled, found at ${VALGRIND}")
    endif()
endif()

# doxygen setup (optional)
set(README_MD "README.md")
if(WITH_DOCS)
    find_package(Doxygen)
    if(DOXYGEN_FOUND)
        configure_file(${CMAKE_CURRENT_SOURCE_DIR}/doxygen/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
        add_custom_target(doc
                ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
                WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
                COMMENT "Generating API documentation with Doxygen" VERBATIM
                )
    else()
        # disable docs since we cannot find Doxygen
        set(WITH_DOCS false)
    endif(DOXYGEN_FOUND)
endif(WITH_DOCS)

if(EFENCE)
    add_definitions(-DEFENCE)
    link_libraries(efence)
endif()

if(SANITIZE)
    set(SANITIZE_OPTIONS -fsanitize=address)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${SANITIZE_OPTIONS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${SANITIZE_OPTIONS}")
    message(STATUS "warning: enabling sanitize testing, remember to disable for deployment")
endif()


add_subdirectory(lib)
add_subdirectory(examples)
#add_subdirectory(tests)
