cmake_minimum_required(VERSION 3.0.2)
project(lcm_test)
#catkin config -DCMAKE_BUILD_TYPE=Release
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fPIC -march=native -funroll-loops")
set(CMAKE_BUILD_TYPE release)

find_package(lcm REQUIRED)

catkin_package(
   INCLUDE_DIRS include
)

include_directories(
  /usr/local/include/
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(lcm_test 
          src/main.cpp
          src/lcm_net.cpp)
target_link_libraries(lcm_test  ${catkin_LIBRARIES} lcm)