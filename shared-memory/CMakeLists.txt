cmake_minimum_required(VERSION 3.12)
project(shared-memory)

if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
  MESSAGE("It using Clang")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 17)

#Add the C++ header files
include_directories(
    include
)

# Add the C++ source files
# set(SOURCE_FILES src/sender.cpp src/receiver.cpp)


# Add the executable for the sender program
add_executable(sender src/sender.c)

# Add the executable for the receiver program
add_executable(receiver src/receiver.c)

# Link the required libraries (e.g., pthread, rt)
target_link_libraries(sender pthread)
target_link_libraries(receiver pthread)

# Conditionally link the rt library if not building on macOS
if (NOT APPLE)
    target_link_libraries(sender rt)
    target_link_libraries(receiver rt)
endif()
