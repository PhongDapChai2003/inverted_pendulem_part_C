cmake_minimum_required(VERSION 3.16)
project(pendulum_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(pendulum_control src/controller.cpp src/estimator.cpp)
target_include_directories(pendulum_control PUBLIC include)
target_compile_options(pendulum_control PRIVATE -Wall -Wextra -Wpedantic)

add_executable(pendulum_sim src/sim_main.cpp)
target_link_libraries(pendulum_sim PRIVATE pendulum_control)
target_compile_options(pendulum_sim PRIVATE -Wall -Wextra -Wpedantic)

add_executable(controller_tests tests/controller_tests.cpp)
target_link_libraries(controller_tests PRIVATE pendulum_control)
target_compile_options(controller_tests PRIVATE -Wall -Wextra -Wpedantic)

enable_testing()
add_test(NAME controller_tests COMMAND controller_tests)
