cmake_minimum_required(VERSION 3.10)
project(OpenPID CXX)

# Option to enable test build
option(RUN_TESTS "Build and run tests" OFF)

# Include the header directory
include_directories(include)

# Create the core library
add_library(pid_lib
    src/pid.cpp
)

# Create the example executable
add_executable(example_pid examples/example_pid.cpp)
target_link_libraries(example_pid pid_lib)

# Conditionally build the test executable
if(RUN_TESTS)
    enable_testing()
    add_executable(test_pid tests/test_pid.cpp)
    target_link_libraries(test_pid pid_lib)
    add_test(NAME pid_test COMMAND test_pid)
endif()

# Add pybind11 as subdirectory if you cloned it locally
add_subdirectory(pybind11)

# Build Python module
pybind11_add_module(openpid bindings.cpp)
target_include_directories(openpid PRIVATE include)