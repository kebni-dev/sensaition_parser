cmake_minimum_required(VERSION 3.16)

find_package(Boost 1.74.0 REQUIRED COMPONENTS unit_test_framework)
include_directories(${Boost_INCLUDE_DIRS})

# libsensaition_parser2.a
add_executable(test_parser
  test_packet_assembler.cpp
  test_sensor_data.cpp
  test_sensor_sample.cpp
  test_main.cpp
)

target_link_libraries(test_parser
  PRIVATE Kebni::sensaition_parser
          Boost::unit_test_framework
)

target_include_directories(test_parser
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

if (WIN32)
  target_link_libraries(test_parser PRIVATE ws2_32)
endif()

add_test(NAME kebni_parser_tests COMMAND test_parser)
