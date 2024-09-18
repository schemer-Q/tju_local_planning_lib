add_executable(test_data_buffer test_data_buffer.cpp)
target_link_libraries(test_data_buffer gtest gtest pthread)

include(GoogleTest)
gtest_discover_tests(test_data_buffer)
