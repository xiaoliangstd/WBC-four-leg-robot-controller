cmake_minimum_required(VERSION 3.4...3.18)
project(TEST CXX)
INCLUDE_DIRECTORIES(/usr/include/eigen3/Eigen)
INCLUDE_DIRECTORIES(/usr/local/include)
INCLUDE_DIRECTORIES(./KFStateEstimator/inc)
INCLUDE_DIRECTORIES(./TOOLS/inc)

INCLUDE_DIRECTORIES(./)
INCLUDE_DIRECTORIES(./FloatingBasedDynamics/inc)
INCLUDE_DIRECTORIES(./WBC/inc)


add_subdirectory(pybind11)
add_subdirectory(WBC)
ADD_SUBDIRECTORY(./KFStateEstimator)
# ADD_SUBDIRECTORY(./VMC)

# add_subdirectory(TOOLS)
# add_subdirectory(FloatingBasedDynamics)
# add_subdirectory(WBC)
# add_subdirectory(Goldfarb_Optimizer)

# add_executable(test_program main.cpp)


# target_link_libraries(test_program KFStateEstimator)

# target_link_libraries(test_program wbc_lib FloatingBasedDynamics)