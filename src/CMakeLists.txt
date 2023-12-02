find_package(geometry REQUIRED)
find_package(controllers REQUIRED)

set(EIGEN_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../external/eigen")

add_library(SimpleUnit STATIC SimpleUnit.cpp)
target_include_directories(SimpleUnit PRIVATE ${EIGEN_PATH})
target_link_libraries(SimpleUnit PUBLIC geometry::geometry)

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