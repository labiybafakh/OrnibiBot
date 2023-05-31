cmake_minimum_required(VERSION 3.12)
project(serial-handler)

set(CMAKE_CXX_STANDARD 11)

find_package(Boost REQUIRED COMPONENTS)

#Add the C++ header files
include_directories(
    include
    ${Boost_INCLUDE_DIRS}
)

# Add the C++ source files
# set(SOURCE_FILES src/sender.cpp src/receiver.cpp)


# Add the executable for the sender program
add_executable(comm src/comm.cpp)

# Link the required libraries (e.g., pthread, rt)
target_link_libraries(comm pthread ${boost_LIBRARIES})

# Conditionally link the rt library if not building on macOS
if (NOT APPLE)
    target_link_libraries(sender rt)
endif()
