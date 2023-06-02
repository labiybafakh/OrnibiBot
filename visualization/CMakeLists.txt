cmake_minimum_required(VERSION 3.12)
project(visualization)

set(CMAKE_CXX_STANDARD 17)


#Add the C++ header files
include_directories(
    include
)

# Add the C++ source files
# set(SOURCE_FILES src/sender.cpp src/receiver.cpp)


# Add the executable for the sender program
add_executable(test src/test.cpp)


# Link the required libraries (e.g., pthread, rt)
target_link_libraries(test ${boost_LIBRARIES})


# Conditionally link the rt library if not building on macOS
if (NOT APPLE)
    target_link_libraries(test pthread ${boost_LIBRARIES})
endif()
