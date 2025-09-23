cmake_minimum_required(VERSION 3.15)
project(Odometry)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find WPILib and REV libraries
find_package(wpilib REQUIRED)
find_package(rev REQUIRED)

# Add all your source files
file(GLOB_RECURSE SRC_FILES src/main/cpp/*.cpp)

add_executable(${PROJECT_NAME} ${SRC_FILES})

# Link WPILib and REV
target_link_libraries(${PROJECT_NAME} wpilibc rev)
