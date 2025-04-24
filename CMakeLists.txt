cmake_minimum_required(VERSION 3.10)
project(OpenPID CXX)

# Option to enable test build
option(RUN_TESTS "Build and run tests" OFF)

# Include the header directory for your project
include_directories(include)

# Add Eigen from submodule
add_subdirectory(external/eigen)  # Make sure Eigen is added here

# Create the core library
add_library(pid_lib
    src/pid.cpp
    src/missile_6dof_quat.cpp  # moved to src/
)

# Link Eigen to pid_lib (modern CMake)
target_link_libraries(pid_lib PUBLIC Eigen3::Eigen)

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

# Add pybind11 as subdirectory (assuming you have it cloned locally)
add_subdirectory(pybind11)

# Build the Python module
pybind11_add_module(openpid bindings.cpp)

# Link your core library to the Python module
target_link_libraries(openpid PRIVATE pid_lib)

# Include headers
target_include_directories(openpid PRIVATE include)
