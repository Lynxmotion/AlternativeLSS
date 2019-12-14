
set(SOURCE_FILES
        LynxmotionLSS-Config.h
        MaskSet.h
        AsyncToken.h
        LssPromise.h
        LssCommon.h LssCommon.cpp
        LssTransaction.h LssTransaction.cpp
        LssCommunication.h LssCommunication.cpp
        LssChannel.h LssChannel.cpp
        platform/posix/LssPosixChannel.h platform/posix/LssPosixChannel.cpp
        analytics/aggregate.h analytics/MovingAverage.h analytics/numerics.h analytics/range.h
)

if(HAS_LIBFTDI)
    set(SOURCE_FILES ${SOURCE_FILES} platform/libftdi/LssFtdiChannel.h platform/libftdi/LssFtdiChannel.cpp)
endif()

if(WITH_COMPLIANCE)
    set(SOURCE_FILES ${SOURCE_FILES} compliance/parameter.h compliance/CompliantJoint.h compliance/CompliantJoint.cpp)
endif()


# we first add our intermediate object library
# which will them be linked into an archive and a shared library
#add_library(sigint-objlib OBJECT ${SOURCE_FILES} $<TARGET_OBJECTS:sigint-matrix> $<TARGET_OBJECTS:sigint-streams> $<TARGET_OBJECTS:sigint-numerical>)
add_library(lss-objlib OBJECT ${SOURCE_FILES})

# currently no dependencies
#add_dependencies(sigint-objlib rxcpp)

# set compilation properties on the object library
set_property(TARGET lss-objlib PROPERTY POSITION_INDEPENDENT_CODE 1)
# target_compile_features(lss-objlib PRIVATE cxx_std_14)
#set_property(TARGET lss-objlib PROPERTY C_STANDARD 99)
#target_include_directories(lss-objlib )

# now link the object lib into static archive and shared library versions
set(LSSLIB_TARGET_OBJECTS $<TARGET_OBJECTS:lss-objlib>)
add_library(lss-static STATIC ${LSSLIB_TARGET_OBJECTS})
add_library(lss-shared SHARED ${LSSLIB_TARGET_OBJECTS})

#SET_TARGET_PROPERTIES(lss-static PROPERTIES OUTPUT_NAME lss CLEAN_DIRECT_OUTPUT 1)
#SET_TARGET_PROPERTIES(lss-shared PROPERTIES OUTPUT_NAME lss CLEAN_DIRECT_OUTPUT 1)
#target_link_libraries(lss-shared ...)

# store location of sigint variables in the cache for other projects to reference
set(LSS_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}" CACHE PATH "path to sigint library includes")
set(LSS_LIBRARY_DIR "${CMAKE_CURRENT_BINARY_DIR}" CACHE PATH "path to sigint library binaries")
#set(LSS_LINK_LIBRARIES ${Boost_LIBRARIES} ${JSON-C_LIBRARY} CACHE INTERNAL "")

# attach version number to SO shared file
#set_target_properties(sigint-shared PROPERTIES
#        VERSION ${sigintSDK_VERSION_STRING}
#        SOVERSION ${sigintSDK_VERSION_MAJOR})

install(TARGETS lss-static lss-shared
        RUNTIME DESTINATION "bin"
        LIBRARY DESTINATION "lib"
        ARCHIVE DESTINATION "lib")
