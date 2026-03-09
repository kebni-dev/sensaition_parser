cmake_minimum_required(VERSION 3.8)

project(sensaition_parser
  VERSION 1.0.0
  LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(SENSAITION_PARSER_BUILD_TESTS "Build unit tests" OFF)
option(SENSAITION_PARSER_ENABLE_WARNINGS "Enable extra compiler warnings" ON)

add_library(sensaition_parser
    src/kebni_driver.cpp
)

# Namespaced alias, so users can link against it like this:
# target_link_libraries(my_app PRIVATE Kebni::parser)
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
