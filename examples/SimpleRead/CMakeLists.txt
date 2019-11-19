
# we first add our intermediate object library
# which will them be linked into an archive and a shared library
#add_library(sigint-objlib OBJECT ${SOURCE_FILES} $<TARGET_OBJECTS:sigint-matrix> $<TARGET_OBJECTS:sigint-streams> $<TARGET_OBJECTS:sigint-numerical>)
add_executable(lss-simpleread SimpleRead.cpp)

# currently no dependencies
add_dependencies(lss-simpleread lss-static)
message(STATUS "lsss at ${LSS_LIBRARY_DIR}")
include_directories(${LSS_INCLUDE_DIR})
link_directories(${LSS_LIBRARY_DIR})
target_link_libraries(lss-simpleread lss-static pthread)

# set compilation properties on the object library
set_property(TARGET lss-objlib PROPERTY POSITION_INDEPENDENT_CODE 1)
#set_property(TARGET lss-objlib PROPERTY C_STANDARD 99)
#target_include_directories(lss-objlib )

install(TARGETS lss-simpleread
        RUNTIME DESTINATION "bin"
        LIBRARY DESTINATION "lib"
        ARCHIVE DESTINATION "lib")
