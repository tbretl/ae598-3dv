# Use:
#
# mkdir build
# cd build
# cmake .. -DCMAKE_BUILD_TYPE=Release
# cmake --build .
#
# It is very important to compile in release mode. You
# get a 20x speed-up.

# Set the minimum required version of CMake
cmake_minimum_required(VERSION 3.10)

# Create a project named BretlOptimize with version number 1.0
project(BretlOptimize VERSION 1.0)

# Chose a C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add an executable called BretlOptimize to the project
add_executable(BretlOptimize
  bretl_optimize.cc
)

# Find non-symforce libraries automatically
find_package (Eigen3 3.4 REQUIRED NO_MODULE)
find_package (spdlog REQUIRED)

#
# Find symforce libraries by hand.
#
# The problem is described here ("SymForce does not currently integrate with
# CMake's find_package..."):
#  https://github.com/symforce-org/symforce#build-with-cmake
#
# One method of solution is described here (along with an explanation of why
# the problem is so annoying):
#  https://stackoverflow.com/a/65046283)
#

message(STATUS "Finding symforce_gen")
list(APPEND CMAKE_MESSAGE_INDENT "  ")
find_path(symforce_gen_DIR sym/pose3.h REQUIRED)
message(STATUS "DIR: ${symforce_gen_DIR}")
find_library (symforce_gen_LIB symforce_gen REQUIRED)
message(STATUS "LIB: ${symforce_gen_LIB}")
list(POP_BACK CMAKE_MESSAGE_INDENT)

message(STATUS "Finding symforce_opt")
list(APPEND CMAKE_MESSAGE_INDENT "  ")
find_path(symforce_opt_DIR symforce/opt/values.h REQUIRED)
message(STATUS "DIR: ${symforce_opt_DIR}")
find_library (symforce_opt_LIB symforce_opt REQUIRED)
message(STATUS "LIB: ${symforce_opt_LIB}")
list(POP_BACK CMAKE_MESSAGE_INDENT)

# Specify where to find symforce headers
include_directories(
  "${symforce_gen_DIR}"
  "${symforce_opt_DIR}"
)

# Where to find libraries
target_link_libraries (BretlOptimize
  ${symforce_gen_LIB}
  ${symforce_opt_LIB}
  Eigen3::Eigen
  spdlog::spdlog
)

# Where to find headers
target_include_directories(BretlOptimize PUBLIC
  "${PROJECT_BINARY_DIR}"
)



