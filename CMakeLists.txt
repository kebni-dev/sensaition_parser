cmake_minimum_required(VERSION 3.8)

project(kebni_driver
  VERSION 1.0.0
  LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(KEBNI_DRIVER_BUILD_TESTS "Build unit tests" OFF)
option(KEBNI_DRIVER_ENABLE_WARNINGS "Enable extra compiler warnings" ON)

add_library(kebni_driver
    src/kebni_driver.cpp
)

# Namespaced alias, so users can link against it like this:
# target_link_libraries(my_app PRIVATE Kebni::driver)
add_library(Kebni::driver ALIAS kebni_driver)

target_include_directories(kebni_driver
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

if (KEBNI_DRIVER_ENABLE_WARNINGS)
  if (MSVC)
    target_compile_options(kebni_driver PRIVATE /W4 /permissive-)
  else()
    target_compile_options(kebni_driver PRIVATE
      -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion
    )
  endif()
endif()

if (KEBNI_DRIVER_BUILD_TESTS)
  enable_testing()
  add_subdirectory(test)
endif()
