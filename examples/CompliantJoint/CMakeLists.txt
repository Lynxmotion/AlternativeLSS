
# we first add our intermediate object library
# which will them be linked into an archive and a shared library
#add_library(sigint-objlib OBJECT ${SOURCE_FILES} $<TARGET_OBJECTS:sigint-matrix> $<TARGET_OBJECTS:sigint-streams> $<TARGET_OBJECTS:sigint-numerical>)
add_executable(lss-compliantjoint CompliantJoints.cpp)

# currently no dependencies
include_directories(${LSS_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/eigen)

# set compilation properties on the object library
set_property(TARGET lss-objlib PROPERTY POSITION_INDEPENDENT_CODE 1)
#set_property(TARGET lss-objlib PROPERTY C_STANDARD 99)
#target_include_directories(lss-objlib )

# example of allowing static or dynamic linkage
if(STATIC_LINKAGE)
    add_dependencies(lss-compliantjoint lss-static)
    link_directories(lss-compliantjoint $<TARGET_FILE_DIR:lss-static>)
    target_link_libraries(lss-compliantjoint lss-static)
else()
    add_dependencies(lss-compliantjoint lss-shared)
    link_directories(lss-compliantjoint $<TARGET_FILE_DIR:lss-shared>)
    target_link_libraries(lss-compliantjoint lss-shared)

    # copy library to output directory
    add_custom_command(TARGET lss-compliantjoint POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:lss-shared> $<TARGET_FILE_DIR:lss-compliantjoint>)
endif()

target_link_libraries(lss-compliantjoint pthread)

if(HAS_LIBFTDI)
    target_link_libraries(lss-compliantjoint ftdi)
endif()

if(VALGRIND)
    add_dependencies(memcheck memcheck-lss-compliantjoint)
    add_custom_target(memcheck-lss-compliantjoint
            COMMAND "${VALGRIND}" --tool=memcheck --leak-check=yes --show-reachable=yes --num-callers=20 --track-fds=yes $<TARGET_FILE:lss-compliantjoint>)
endif()

install(TARGETS lss-compliantjoint
        RUNTIME DESTINATION "bin"
        LIBRARY DESTINATION "lib"
        ARCHIVE DESTINATION "lib")
