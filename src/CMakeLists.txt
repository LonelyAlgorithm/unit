# Find the geometry library
find_package(geometry REQUIRED)

# Set path to Eigen
set(EIGEN_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../external/eigen")

# Library
add_library(SimpleUnit STATIC SimpleUnit.cpp)

# Include directories
target_include_directories(SimpleUnit PRIVATE ${EIGEN_PATH})
target_include_directories(SimpleUnit PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link with the geometry library
target_link_libraries(SimpleUnit PRIVATE geometry::geometry)

# Installation
include(GNUInstallDirs)

install(TARGETS SimpleUnit
        EXPORT unitTargets
)

install(DIRECTORY ../include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT unitTargets
    FILE unitTargets.cmake
    NAMESPACE unit::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/unit
)

configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/../unitConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/unitConfig.cmake"
    @ONLY
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/unitConfig.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/unit
)