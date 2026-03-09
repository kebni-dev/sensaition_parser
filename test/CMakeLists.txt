find_package(GTest REQUIRED)

add_executable(test_parser
  test_config.cpp
  test_measurements.cpp
)

target_link_libraries(test_parser
  PRIVATE Kebni::sensaition_parser
          GTest::gtest
          GTest::gtest_main
)

add_test(NAME kebni_parser_tests COMMAND test_parser)