cmake_minimum_required(VERSION 3.10)

project(sensaition_parser
  VERSION 2.0.0
  LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(SENSAITION_PARSER_BUILD_TESTS "Build unit tests" OFF)

# eventually turn this option ON as default (after fixing remaining warnings)
option(SENSAITION_PARSER_ENABLE_WARNINGS "Enable extra compiler warnings" OFF)

add_library(sensaition_parser
    src/packet_assembler.cpp
    src/sensor_data_backend.cpp
    src/sensor_sample.cpp
)

# Namespaced alias, so users can link against it like this:
# target_link_libraries(my_app PRIVATE Kebni::sensaition_parser)
add_library(Kebni::sensaition_parser ALIAS sensaition_parser)

target_include_directories(sensaition_parser
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

if (SENSAITION_PARSER_ENABLE_WARNINGS)
  if (MSVC)
    target_compile_options(sensaition_parser PRIVATE /W4 /permissive-)
  else()
    target_compile_options(sensaition_parser PRIVATE
      -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion
    )
  endif()
endif()

if (SENSAITION_PARSER_BUILD_TESTS)
  enable_testing()
  add_subdirectory(test)
endif()
