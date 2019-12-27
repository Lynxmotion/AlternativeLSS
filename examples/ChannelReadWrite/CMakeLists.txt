
# we first add our intermediate object library
# which will them be linked into an archive and a shared library
#add_library(sigint-objlib OBJECT ${SOURCE_FILES} $<TARGET_OBJECTS:sigint-matrix> $<TARGET_OBJECTS:sigint-streams> $<TARGET_OBJECTS:sigint-numerical>)
add_executable(lss-channelrw ChannelReadWrite.cpp)

# currently no dependencies
add_dependencies(lss-channelrw lss-static)
include_directories(${LSS_INCLUDE_DIR})
link_directories(${LSS_LIBRARY_DIR})
target_link_libraries(lss-channelrw lss-static pthread)

# set compilation properties on the object library
set_property(TARGET lss-objlib PROPERTY POSITION_INDEPENDENT_CODE 1)
#set_property(TARGET lss-objlib PROPERTY C_STANDARD 99)
#target_include_directories(lss-objlib )

if(HAS_LIBFTDI)
    target_link_libraries(lss-channelrw ftdi)
endif()

if(VALGRIND)
    add_dependencies(memcheck memcheck-lss-channelrw)
    add_custom_target(memcheck-lss-channelrw
            COMMAND "${VALGRIND}" --tool=memcheck --leak-check=yes --show-reachable=yes --num-callers=20 --track-fds=yes $<TARGET_FILE:lss-channelrw>)
endif()

install(TARGETS lss-channelrw
        RUNTIME DESTINATION "bin"
        LIBRARY DESTINATION "lib"
        ARCHIVE DESTINATION "lib")
